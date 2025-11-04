#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sys
import json
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration as DurationMsg
import time
import rclpy.parameter
from rcl_interfaces.srv import SetParameters
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus
from std_msgs.msg import Empty
from std_msgs.msg import Float32
import argparse

class Nav2WaypointManager(Node):
    def __init__(self, filename, is_looping=True):
        super().__init__('nav2_waypoint_manager_executor')

        self.odometry_switch_type = "LIO raw"
        bool_descriptor = ParameterDescriptor(
            name='use_gnss_switch',
            type=ParameterType.PARAMETER_BOOL,
            description='Enable or disable the feature',
            read_only=False
        )
        self.declare_parameter("use_gnss_switch", False, bool_descriptor)
        self.use_gnss_switch_flg = self.get_parameter("use_gnss_switch").value
        self.cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.initialize_cmd_vel_linear_x = self.declare_parameter("initialize_cmd_vel_linear_x", 0.1).value
        self.initialize_radius = self.declare_parameter("initialize_radius", 3.0).value
        self.waypoints = []
        self.attributes = []
        self.filename = filename
        self.is_looping = is_looping

        _ = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.waypoint_pub = self.create_publisher(PoseArray, 'waypoints', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'waypoint_marker_array', 10)

        self.original_speed = self.declare_parameter('original_speed', 1.12).value
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Execute mode logic
        self.load_waypoints_from_json()
        self.behavior_timer = self.create_timer(0.1, self.callback_behavior_timer) # stop等の状態を管理するタイマー.
        self.last_attr_time = self.get_clock().now()
        self.current_attr_value = 0
        self.current_attr_type = "normal"
        self.last_waypoint_index = -1
        self.odometry_switch_type_sub = self.create_subscription(String, '/odometry/switch/type', self.odometry_switch_type_callback, 10)
        self.initialize_cmdvel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.stop_command_pub = self.create_publisher(Empty, '/wizurg/stop_cmd_vel', 10)
        self.start_command_pub = self.create_publisher(Empty, '/wizurg/start_cmd_vel', 10)
        self.slow_command_pub = self.create_publisher(Float32, '/wizurg/slow_cmd_vel', 10)
        self.wait_for_stable_odometry_switch_type()
        self.send_waypoints_goal()
        self.get_logger().info("Execute mode enabled. Starting navigation...")

    def callback_behavior_timer(self):
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_attr_time
        if self.current_attr_type == "stop":
            if time_diff.nanoseconds / 1e9 > self.current_attr_value:
                self.reset_attribute_state()
                self.get_logger().info("Stop duration completed. Resuming navigation.")
    
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
                    pose_stamped.pose.position.x = item[0][0]
                    pose_stamped.pose.position.y = item[0][1]
                    pose_stamped.pose.position.z = 0.0
                    pose_stamped.pose.orientation.x = 0.0
                    pose_stamped.pose.orientation.y = 0.0
                    pose_stamped.pose.orientation.z = item[1][2]
                    pose_stamped.pose.orientation.w = item[1][3]
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
            position = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, 0.0]
            orientation = [0.0, 0.0, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
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
                marker.pose.position.x = pose_stamped.pose.position.x
                marker.pose.position.y = pose_stamped.pose.position.y
                marker.pose.position.z = 0.5
                
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

    def send_waypoints_goal(self):
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded. Exiting.")
            return

        self.get_logger().info("Waiting for Nav2 action server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available after waiting. Exiting.")
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self.get_logger().info("Sending goal to Nav2 action server...")
        self._action_client_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._action_client_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by action server')
            return

        self.get_logger().info('Goal accepted! Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        # 周回する場合、current_waypointが0になるので(N週目の始まり)、要確認. -> python の配列の要素番号 -1 は配列の末尾を指すので問題なし。
        completed_waypoint_index = feedback.current_waypoint - 1

        #ウェイポイント番号が更新されたら、attributeに基づいて動作を変更する.
        if completed_waypoint_index >= -1 \
            and completed_waypoint_index < len(self.attributes) \
            and completed_waypoint_index != self.last_waypoint_index:

            attribute = self.attributes[completed_waypoint_index]
            self.last_waypoint_index = completed_waypoint_index
            self.process_waypoint_attribute(attribute)
            self.get_logger().info(f'=============waypoint id debug===============')
            self.get_logger().info(f'current_waypoint_index: {feedback.current_waypoint}')
            self.get_logger().info(f'completed_waypoint_index {completed_waypoint_index}')
            self.get_logger().info(f'attribute: {attribute}')
            self.get_logger().info(f'=============================================')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! All waypoints reached.')
            if self.is_looping:
                self.get_logger().info('Looping back to the beginning...')
                self.send_waypoints_goal()
            else:
                self.get_logger().info('All waypoints processed once. Shutting down.')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')

    def process_waypoint_attribute(self, attribute):
        self.reset_attribute_state()  # 前回のwaypointのattributeを解除する.


        attr_type = attribute.get("type", "normal")
        attr_value = attribute.get("value", 0)

        xy_tolerance = attribute.get("xy_tolerance", 1.0)
        yaw_tolerance = attribute.get("yaw_tolerance", 3.14)

        self.get_logger().info(f"Processing attribute: type={attr_type}, value={attr_value}, xy_tolerance={xy_tolerance}, yaw_tolerance={yaw_tolerance}")
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
            self.current_attr_type = "slow"
            #param = rclpy.parameter.Parameter('max_speed_xy', rclpy.Parameter.Type.DOUBLE, float(attr_value))
            #request.parameters.append(param.to_parameter_msg())
            # self.set_parameters_client.call_async(request)

        elif attr_type == "normal":
            self.get_logger().info(f"Setting max_speed_xy to original speed {self.original_speed} m/s.")

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='A waypoint manager for Nav2.')
    parser.add_argument('filename', type=str, help='The name of the waypoint JSON file.')
    parser.add_argument('--once', action='store_true', help='Execute waypoints only once, do not loop.')

    parsed_args, ros_args = parser.parse_known_args()

    if parsed_args.filename:
        try:
            node = Nav2WaypointManager(parsed_args.filename, not parsed_args.once)
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            if 'node' in locals() and rclpy.ok():
                node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()