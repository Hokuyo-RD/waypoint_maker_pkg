#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sys
import json
from geometry_msgs.msg import PoseArray, PoseStamped, Twist, Pose
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg
import time
import rclpy.parameter
import math
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from rcl_interfaces.srv import SetParameters
import threading
import argparse

class Nav2WaypointManager(Node):
    def __init__(self, filename, is_looping=True):
        super().__init__('nav2_waypoint_manager_executor')

        bool_descriptor = ParameterDescriptor(
            name='use_gnss_switch',
            type=ParameterType.PARAMETER_BOOL,
            description='Enable or disable the feature',
            read_only=False
        )
        self.declare_parameter("use_gnss_switch", False, bool_descriptor)
        self.declare_parameter('yaw_goal_tolerance', 0.25) 
        self.declare_parameter('xy_goal_tolerance', 0.25)

        self.yaw_tolerance = self.get_parameter('yaw_goal_tolerance').get_parameter_value().double_value
        self.xy_tolerance = self.get_parameter('xy_goal_tolerance').get_parameter_value().double_value

        self.use_gnss_switch_flg = self.get_parameter("use_gnss_switch").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.initialize_cmd_vel_linear_x = self.declare_parameter("initialize_cmd_vel_linear_x", 0.1).value
        self.initialize_radius = self.declare_parameter("initialize_radius", 3.0).value

        self.waypoints = []
        self.attributes = []
        self.filename = filename
        self.is_looping = is_looping
        self.last_waypoint_index = -1
        self.current_attr_type = "normal"
        self.current_attr_value = 0
        self.last_attr_time = self.get_clock().now()


        self.current_waypoint_index = 0
        self.is_navigating = False
        self.arrival_check_count = 0
        self.shutdown_flag = threading.Event()
        self.odometry_switch_type = "LIO raw"
        self.goal_handle = None

        self.waypoint_pub = self.create_publisher(PoseArray, 'waypoints', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'waypoint_marker_array', 10)

        self.set_parameters_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_parameters service not available, waiting again...')

        self.original_speed = self.declare_parameter('original_speed', 1.12).value
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odometry_switch_type_sub = self.create_subscription(String, '/odometry/switch/type', self.odometry_switch_type_callback, 10)
        self.initialize_cmdvel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.stop_command_pub = self.create_publisher(Empty, '/wizurg/stop_cmd_vel', 10)
        self.start_command_pub = self.create_publisher(Empty, '/wizurg/start_cmd_vel', 10)
        self.slow_command_pub = self.create_publisher(Float32, '/wizurg/slow_cmd_vel', 10)

        self.load_waypoints_from_json()
        self.update_waypoint_visualization()

        self.wait_for_stable_odometry_switch_type()
        self.main_loop_timer = self.create_timer(0.1, self.main_loop)
        self.get_logger().info("Execute mode enabled. Starting navigation...")

    def reset_attribute_state(self):
        self.start_command_pub.publish(Empty())
        self.current_attr_type = "normal"
        self.current_attr_value = 0
        self.last_attr_time = self.get_clock().now()

    def load_waypoints_from_json(self):
        try:
            with open(self.filename, 'r') as f:
                data = json.load(f)
                self.waypoints = []
                self.attributes = []
                for item in data:
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose.position.x = float(item[0][0])
                    pose_stamped.pose.position.y = float(item[0][1])
                    pose_stamped.pose.position.z = float(item[0][2]) # Z座標をJSONから読み込む
                    pose_stamped.pose.orientation.x = float(item[1][0]) # X成分をJSONから読み込む
                    pose_stamped.pose.orientation.y = float(item[1][1]) # Y成分をJSONから読み込む
                    pose_stamped.pose.orientation.z = float(item[1][2])
                    pose_stamped.pose.orientation.w = float(item[1][3])
                    self.waypoints.append(pose_stamped)

                    # 属性のロード時にデフォルト値を設定
                    attribute = item[2] if len(item) > 2 else {"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14}
                    attribute["xy_tolerance"] = attribute.get("xy_tolerance", 1.0)
                    attribute["yaw_tolerance"] = attribute.get("yaw_tolerance", 3.14)
                    self.attributes.append(attribute)

            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.filename}")
        except FileNotFoundError:
            self.get_logger().warn(f"Waypoint file {self.filename} not found. Creating a new one.")
            self.save_waypoints_to_json()
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON in {self.filename}. Please check the file format.")
        except IndexError:
            self.get_logger().error(f"Invalid JSON format in {self.filename}. Expected [[x, y, 0.0], [0.0, 0.0, z, w], {{'type': '...', 'value': ...}}].")

    def save_waypoints_to_json(self):
        data = []
        for i, pose_stamped in enumerate(self.waypoints):
            position = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z] # Z座標を保存
            orientation = [pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w] # X, Y成分を保存
            attribute = self.attributes[i] if i < len(self.attributes) else {"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14}
            data.append([position, orientation, attribute])
        try:
            with open(self.filename, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {self.filename}")
        except IOError as e:
            self.get_logger().error(f"Failed to write waypoint to JSON: {e}")

    def update_waypoint_visualization(self):
            """GUIからの操作後にウェイポイントの可視化を更新する"""
            pose_array = PoseArray()
            pose_array.header.frame_id = "map"
            pose_array.header.stamp = self.get_clock().now().to_msg()
            for pose_stamped in self.waypoints:
                pose_array.poses.append(pose_stamped.pose)
            self.waypoint_pub.publish(pose_array)

            marker_array = MarkerArray()
            # 既存のマーカーをすべて削除するためのマーカーをパブリッシュ
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            delete_marker.header.frame_id = "map"
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            delete_marker.ns = "waypoint_markers"
            marker_array.markers.append(delete_marker)
            self.marker_array_pub.publish(marker_array)
            time.sleep(0.1)

            # 新しいマーカーを作成してパブリッシュ
            marker_array = MarkerArray()
            for i, pose_stamped in enumerate(self.waypoints):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "waypoint_markers"
                marker.id = i
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = pose_stamped.pose.position.x # X座標
                marker.pose.position.y = pose_stamped.pose.position.y # Y座標
                marker.pose.position.z = pose_stamped.pose.position.z + 0.5 # Z座標 + 0.5m (テキストが地面から浮くように)
                
                # ウェイポイントの向きを設定
                marker.pose.orientation.x = pose_stamped.pose.orientation.x
                marker.pose.orientation.y = pose_stamped.pose.orientation.y
                marker.pose.orientation.z = pose_stamped.pose.orientation.z
                marker.pose.orientation.w = pose_stamped.pose.orientation.w
                
                marker.scale.z = 0.5
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0

                # 属性情報を取得
                attr = self.attributes[i] if i < len(self.attributes) else {"type": "normal", "value": 0}
                attr_type = attr.get('type', 'normal')
                attr_value = attr.get('value', 0)
                
                unit = ""
                value_display = f"{attr_value:.2f}"
                
                if attr_type == "stop":
                    unit = "[s]"
                elif attr_type == "slow":
                    unit = "[m/s]"
                
                # 'normal' 属性の場合は、元の速度と単位を表示
                if attr_type == "normal":
                    value_display = f"{self.original_speed:.2f}"
                    unit = "[m/s]" 

                # テキスト文字列をカスタマイズして、x, y座標、属性情報、向き(qz, qw)を表示
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                qz = pose_stamped.pose.orientation.z
                qw = pose_stamped.pose.orientation.w

                marker.text = (
                    f"[{i}]"
                    f"\nx:{x:.2f}"
                    f"\ny:{y:.2f}"
                    f"\nType:{attr_type}"
                    f"\nValue:{value_display}{unit}"
                )

                marker.lifetime = DurationMsg()
                marker_array.markers.append(marker)

            self.marker_array_pub.publish(marker_array)

    def wait_for_stable_odometry_switch_type(self):
        if self.use_gnss_switch_flg:
            while self.odometry_switch_type == "LIO raw":
                msg = Twist()
                msg.linear.x = self.initialize_cmd_vel_linear_x
                # msg.angular.z = msg.linear.x / self.initialize_radius
                msg.angular.z = 0.0
                self.initialize_cmdvel_pub.publish(msg)
                self.get_logger().info("gnss-lio-switch initializing...")
                time.sleep(1)
                rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().info("gnss-lio-switch is stable now.")
            time.sleep(10)
        else:
            return

    def odometry_switch_type_callback(self, msg):
        self.odometry_switch_type = msg.data

    def send_goal(self, pose_stamped: PoseStamped):
        """Nav2 Action Server に移動目標を送信する"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info("Waiting for Nav2 action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error('Nav2 action server not available after waiting!')
             self.is_navigating = False
             return

        self.get_logger().info(f'Sending goal for waypoint {self.current_waypoint_index}...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal was rejected by action server')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted. Starting custom arrival check...')
        self.is_navigating = True

    def angle_diff(self, target, current):
        """-piからpiの範囲で角度の差を計算する"""
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff <= -math.pi:
            diff += 2 * math.pi
        return abs(diff)

    def callback_behavior_timer(self):
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_attr_time
        if self.current_attr_type == "stop":
            if time_diff.nanoseconds / 1e9 > self.current_attr_value:
                self.reset_attribute_state()
                self.get_logger().info("Stop duration completed. Resuming navigation.")

    def main_loop(self):
        """メインループ (カスタムの到着判定ロジックを実行)"""
        self.update_waypoint_visualization()
        
        self.callback_behavior_timer()

        # 1. ナビゲーション中でない場合は、次のゴールを送信
        if not self.is_navigating and self.current_waypoint_index < len(self.waypoints):
            # self.is_navigating は goal_response_callback でゴールが受理されたときに True に設定される
            next_pose = self.waypoints[self.current_waypoint_index]
            self.send_goal(next_pose)

        # 2. ナビゲーション中の場合、到着判定ロジックを実行
        elif self.is_navigating:
            try:
                # TF (現在位置) を取得
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform('map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=0.1))
                
                # 現在の自己位置と目標位置を取得
                pos = transform.transform.translation
                rot = transform.transform.rotation
                current_euler = tf_transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
                
                goal_pose = self.waypoints[self.current_waypoint_index].pose
                goal_rot = goal_pose.orientation
                goal_euler = tf_transformations.euler_from_quaternion([goal_rot.x, goal_rot.y, goal_rot.z, goal_rot.w])

                # 現在のウェイポイントの属性から許容誤差を取得
                current_attribute = self.attributes[self.current_waypoint_index]
                xy_tolerance = current_attribute.get('xy_tolerance', self.get_parameter('xy_goal_tolerance').get_parameter_value().double_value)
                yaw_tolerance = current_attribute.get('yaw_tolerance', self.get_parameter('yaw_goal_tolerance').get_parameter_value().double_value)

                # デバッグ情報の表示
                attr_type = current_attribute.get('type', 'normal')
                attr_value = current_attribute.get('value', 0)
                self.get_logger().info(
                    f"Target WP[{self.current_waypoint_index}]: xy_tol={xy_tolerance:.2f}, yaw_tol={yaw_tolerance:.2f}, attr='{attr_type}', val={attr_value}"
                )
                # 距離と角度の差を計算
                dist_err = math.sqrt((pos.x - goal_pose.position.x)**2 + (pos.y - goal_pose.position.y)**2)
                yaw_err = self.angle_diff(goal_euler[2], current_euler[2])

                # --- ウェイポイントを通り過ぎたかどうかの判定 ---
                is_passed = False
                # 最初のウェイポイント以外で判定
                if self.current_waypoint_index > 0:
                    prev_pose = self.waypoints[self.current_waypoint_index - 1].pose
                    # ベクトルA: prev_wp -> current_wp
                    vec_a_x = goal_pose.position.x - prev_pose.position.x
                    vec_a_y = goal_pose.position.y - prev_pose.position.y
                    # ベクトルB: current_wp -> robot_pos
                    vec_b_x = pos.x - goal_pose.position.x
                    vec_b_y = pos.y - goal_pose.position.y
                    
                    # 内積を計算
                    dot_product = vec_a_x * vec_b_x + vec_a_y * vec_b_y
                    
                    # 内積が正の場合、ロボットはウェイポイントを通り過ぎたと判断
                    if dot_product > 0:
                        is_passed = True
                        self.get_logger().info(f"Waypoint {self.current_waypoint_index} has been passed due to position correction. Considering it as reached.")
                # -----------------------------------------

                # 到着判定: 距離と角度がそれぞれの許容誤差以内であること(AND条件)
                if dist_err <= xy_tolerance and yaw_err <= yaw_tolerance:
                    self.arrival_check_count += 1
                else:
                    self.arrival_check_count = 0
                
            except TransformException as ex:
                self.get_logger().warn(f'Could not transform "base_link" to "map": {ex}')
                self.arrival_check_count = 0
                return
                
            # 3. 5回連続で閾値内にいれば到達と見なす
            if self.arrival_check_count > 1 or is_passed:
                self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}.")
                
                # ウェイポイント到達後の属性処理 (last_waypoint_indexが更新された場合のみ)
                if self.current_waypoint_index != self.last_waypoint_index and self.current_waypoint_index < len(self.attributes):
                    attribute = self.attributes[self.current_waypoint_index] # 次のウェイポイントの属性
                    self.process_waypoint_attribute(attribute)
                    self.last_waypoint_index = self.current_waypoint_index
                    self.get_logger().info(f"Processing attribute for waypoint {self.current_waypoint_index}")
                
                # 次のウェイポイントへ
                self.is_navigating = False
                self.arrival_check_count = 0
                self.current_waypoint_index += 1

                # 進行中のゴールをキャンセル
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()

                # 全てのウェイポイントが完了したかチェック
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info('All waypoints finished!')
                    if self.is_looping:
                        self.get_logger().info('Looping waypoints...')
                        self.current_waypoint_index = 0
                    else:
                        self.get_logger().info('Run once mode. Shutting down node...')
                        self.destroy_timer(self.main_loop_timer)
                        self.shutdown_flag.set()

    def process_waypoint_attribute(self, attribute):
        self.reset_attribute_state()  # 前回のwaypointのattributeを解除する

        attr_type = attribute.get("type", "normal")
        attr_value = attribute.get("value", 0)
        xy_tolerance = attribute.get("xy_tolerance", self.get_parameter("xy_goal_tolerance").value)
        yaw_tolerance = attribute.get("yaw_tolerance", self.get_parameter("yaw_goal_tolerance").value)

        self.get_logger().info(f"Processing attribute: type={attr_type}, value={attr_value}, xy_tolerance={xy_tolerance}, yaw_tolerance={yaw_tolerance}")

        request = SetParameters.Request()
        request.parameters.append(rclpy.parameter.Parameter('xy_goal_tolerance', rclpy.Parameter.Type.DOUBLE, float(xy_tolerance)).to_parameter_msg())
        request.parameters.append(rclpy.parameter.Parameter('yaw_goal_tolerance', rclpy.Parameter.Type.DOUBLE, float(yaw_tolerance)).to_parameter_msg())

        if attr_type == "stop":
            self.get_logger().info(f"Stopping for {attr_value} seconds...")
            self.stop_command_pub.publish(Empty())
            self.last_attr_time = self.get_clock().now()
            self.current_attr_value = attr_value
            self.current_attr_type = "stop"

        elif attr_type == "slow":
            self.get_logger().info(f"Setting max_speed_xy to {attr_value} m/s.")
            slow_speed = float(attr_value)
            msg = Float32()
            msg.data = slow_speed
            self.slow_command_pub.publish(msg)
            self.last_attr_time = self.get_clock().now()
            self.current_attr_value = attr_value
            self.curent_attr_type = "slow"

        elif attr_type == "normal":
            param = rclpy.parameter.Parameter('max_speed_xy', rclpy.Parameter.Type.DOUBLE, self.original_speed)
            request.parameters.append(param.to_parameter_msg())
            self.get_logger().info(f"Setting max_speed_xy to original speed {self.original_speed} m/s.")
            self.set_parameters_client.call_async(request)
            self.get_logger().info(f"Resuming original speed ({self.original_speed} m/s).")
            self.start_command_pub.publish(Empty())
            self.current_attr_type = "normal"

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='A waypoint manager for Nav2.')
    parser.add_argument('filename', type=str, help='The name of the waypoint JSON file.')
    parser.add_argument('--once', action='store_true', help='Execute waypoints only once, do not loop.')

    parsed_args, _ = parser.parse_known_args()

    if not parsed_args.filename:
        parser.print_help()
        rclpy.shutdown()
        return

    node = None
    try:
        node = Nav2WaypointManager(parsed_args.filename, not parsed_args.once)
        # rclpy.spin(node)
        while rclpy.ok() and not node.shutdown_flag.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.')
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()