#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import json
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import qos_profile_sensor_data
import time
import math
from rclpy.duration import Duration
from builtin_interfaces.msg import Duration as DurationMsg

class Nav2WaypointMaker(Node):
    def __init__(self, mode, filename):
        super().__init__('nav2_waypoint_maker_' + mode) # ノード名にモードを含める

        self.waypoints = []  # ウェイポイントを PoseStamped のリストで管理
        self.mode = mode
        self.filename = filename

        self.waypoint_pub = self.create_publisher(PoseArray, 'waypoints', 10) # 可視化用
        self.marker_pub = self.create_publisher(Marker, 'waypoint_markers', 10)

        if self.mode == 'write':
            self.is_insert = -1
            self.joy_button = self.declare_parameter('waypoint_button', 1).value
            self.distance_threshold = self.declare_parameter('auto_waypoint_distance', 5.0).value
            self.topic_timeout = self.declare_parameter('estimated_pose_timeout', 20.0).value

            self.lio_loc_pose = PoseStamped()
            self.last_message_time = self.get_clock().now()
            self.previous_pose = None

            self.amcl_sub = self.create_subscription(
                PoseStamped, '/estimated_pose', self.amcl_callback, qos_profile_sensor_data)
            self.initialpose_sub = self.create_subscription(
                PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 10)
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
            self.goal_sub = self.create_subscription(
                PoseStamped, '/goal_pose', self.goal_callback, 10) # rviz の Goal Tool からの入力
            self.remove_sub = self.create_subscription(Int16, '/remove_waypoint', self.remove_callback, 10)
            self.insert_sub = self.create_subscription(Int16, '/insert_waypoint', self.insert_callback, 10)

            self.timer = self.create_timer(1.0, self.check_timeout)

            self.load_waypoints_from_json() # 起動時にウェイポイントを読み込むように変更

        elif self.mode == 'read':
            self.load_waypoints_from_json() # 起動時にウェイポイントを読み込む
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
        else:
            self.get_logger().error(f"Invalid mode: {self.mode}. Use 'write' or 'read'.")
            sys.exit()

    def load_waypoints_from_json(self):
        try:
            with open(self.filename, 'r') as f:
                data = json.load(f)
                self.waypoints = []
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
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.filename}")
        except FileNotFoundError:
            if self.mode == 'write':
                self.get_logger().warn(f"Waypoint file {self.filename} not found. Creating a new one.")
                self.save_waypoints_to_json()
            else:
                self.get_logger().error(f"Waypoint file {self.filename} not found in read mode.")
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON in {self.filename}. Please check the file format.")
        except IndexError:
            self.get_logger().error(f"Invalid JSON format in {self.filename}. Expected [[x, y, 0.0], [0.0, 0.0, z, w]].")

    def save_waypoints_to_json(self):
        if self.mode == 'write':
            data = []
            for pose_stamped in self.waypoints:
                position = [pose_stamped.pose.position.x, pose_stamped.pose.position.y, 0.0]
                orientation = [0.0, 0.0, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w]
                data.append([position, orientation])
            try:
                with open(self.filename, 'w') as f:
                    json.dump(data, f, indent=4)
                self.get_logger().info(f"Saved {len(self.waypoints)} waypoints to {self.filename}")
            except IOError as e:
                self.get_logger().error(f"Failed to write waypoint to JSON: {e}")
        else:
            self.get_logger().warn("Save function called in read-only mode.")

    def publish_waypoints_for_vis(self):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        for pose_stamped in self.waypoints:
            pose_array.poses.append(pose_stamped.pose)
        self.waypoint_pub.publish(pose_array)

    def rewrite_marker(self):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = self.get_clock().now().to_msg()
        marker_data.ns = "basic_shapes"
        marker_data.action = Marker.DELETEALL
        self.marker_pub.publish(marker_data)

        marker_data.action = Marker.ADD
        counter = 0
        marker_data.color.a = 1.0
        marker_data.scale.z = 0.5
        marker_data.lifetime = DurationMsg()
        marker_data.type = Marker.TEXT_VIEW_FACING
        for pose_stamped in self.waypoints:
            marker_data.id = counter
            marker_data.pose.position.x = pose_stamped.pose.position.x
            marker_data.pose.position.y = pose_stamped.pose.position.y
            marker_data.pose.orientation.z = pose_stamped.pose.orientation.z
            marker_data.pose.orientation.w = pose_stamped.pose.orientation.w
            marker_data.text = str(counter)
            self.marker_pub.publish(marker_data)
            counter += 1

    def goal_callback(self, msg):
        if self.mode == 'write':
            waypoint = PoseStamped()
            waypoint.header = msg.header
            waypoint.pose = msg.pose
            if waypoint.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                waypoint.header.frame_id = "map"
            self.waypoints.append(waypoint)
            self.save_waypoints_to_json()
            self.rewrite_marker()
            self.publish_waypoints_for_vis()
            self.get_logger().info("Waypoint added from /goal_pose")
        else:
            self.get_logger().warn("Goal callback active in read-only mode.")

    def amcl_callback(self, msg):
        if self.mode == 'write':
            self.lio_loc_pose = msg
            self.last_message_time = self.get_clock().now()

            if self.previous_pose is None:
                self.previous_pose = self.lio_loc_pose
            else:
                distance = math.sqrt(
                    (self.lio_loc_pose.pose.position.x - self.previous_pose.pose.position.x) ** 2 +
                    (self.lio_loc_pose.pose.position.y - self.previous_pose.pose.position.y) ** 2
                )
                if distance >= self.distance_threshold:
                    self.amcl_waypoint_append()
                    self.previous_pose = self.lio_loc_pose
        else:
            pass

    def amcl_waypoint_append(self):
        if self.mode == 'write':
            waypoint = PoseStamped()
            waypoint.header = self.lio_loc_pose.header
            waypoint.pose = self.lio_loc_pose.pose
            if waypoint.header.frame_id != "map":
                self.get_logger().warn("Appending waypoint in non-map frame. Assuming map frame.")
                waypoint.header.frame_id = "map"
            self.waypoints.append(waypoint)
            self.save_waypoints_to_json()
            self.rewrite_marker()
            self.publish_waypoints_for_vis()
            self.get_logger().info("Waypoint added from /estimated_pose")
        else:
            pass

    def joy_callback(self, msg):
        if self.mode == 'write':
            if msg.buttons[self.joy_button] == 1:
                self.get_logger().info("Adding waypoint via joystick button")
                waypoint = PoseStamped()
                waypoint.header = self.lio_loc_pose.header
                waypoint.pose = self.lio_loc_pose.pose
                if waypoint.header.frame_id != "map":
                    self.get_logger().warn("Appending joystick waypoint in non-map frame. Assuming map frame.")
                    waypoint.header.frame_id = "map"
                self.waypoints.append(waypoint)
                self.save_waypoints_to_json()
                self.rewrite_marker()
                self.publish_waypoints_for_vis()
            self.last_message_time = self.get_clock().now()
        else:
            pass

    def init_pose_callback(self, msg):
        if self.mode == 'write':
            init_pose_dict = {
                "frame_id": msg.header.frame_id,
                "Pos_x": msg.pose.pose.position.x,
                "Pos_y": msg.pose.pose.position.y,
                "Pos_z": msg.pose.pose.position.z,
                "Ori_x": msg.pose.pose.orientation.x,
                "Ori_y": msg.pose.pose.orientation.y,
                "Ori_z": msg.pose.pose.orientation.z,
                "Ori_w": msg.pose.pose.orientation.w,
                "cov": msg.pose.covariance,
            }
            try:
                with open(("initial_" + self.filename), 'w') as f:
                    json.dump(init_pose_dict, f, indent=4)
                self.get_logger().info("Initial pose saved")
            except IOError as e:
                self.get_logger().error(f"Failed to save initial pose: {e}")
            self.last_message_time = self.get_clock().now()
        else:
            pass

    def remove_callback(self, msg):
        if self.mode == 'write':
            try:
                index_to_remove = msg.data
                if 0 <= index_to_remove < len(self.waypoints):
                    removed_waypoint = self.waypoints.pop(index_to_remove)
                    self.get_logger().info(f"Removed waypoint at index {index_to_remove}: {removed_waypoint.pose.position}")
                    self.save_waypoints_to_json()
                    self.rewrite_marker()
                    self.publish_waypoints_for_vis()
                else:
                    self.get_logger().warn(f"Invalid index for removal: {index_to_remove}")
            except IndexError:
                self.get_logger().warn("Waypoint list is empty, cannot remove.")
        else:
            self.get_logger().warn("Remove callback active in read-only mode.")

    def insert_waypoint(self, msg):
        if self.mode == 'write':
            if 0 <= self.is_insert <= len(self.waypoints):
                waypoint = PoseStamped()
                waypoint.header = msg.header
                waypoint.pose = msg.pose
                if waypoint.header.frame_id != "map":
                    self.get_logger().warn("Inserting waypoint in non-map frame. Assuming map frame.")
                    waypoint.header.frame_id = "map"
                self.waypoints.insert(self.is_insert, waypoint)
                self.is_insert = -1
                self.save_waypoints_to_json()
                self.rewrite_marker()
                self.publish_waypoints_for_vis()
                self.get_logger().info(f"Inserted waypoint at index {self.is_insert}")
            else:
                self.get_logger().warn("Invalid insert index or no index set.")
        else:
            pass

    def insert_callback(self, msg):
        if self.mode == 'write':
            insert_index = msg.data
            if 0 <= insert_index <= len(self.waypoints):
                self.is_insert = insert_index
                self.get_logger().info(f"Ready to insert waypoint at index {self.is_insert}. Publish to /goal_pose.")
            else:
                self.get_logger().warn(f"Invalid index for insertion: {insert_index}")
                self.is_insert = -1
        else:
            self.get_logger().warn("Insert callback active in read-only mode.")

    def check_timeout(self):
        if self.mode == 'write':
            current_time = self.get_clock().now()
            time_diff = current_time - self.last_message_time
            if time_diff.nanoseconds / 1e9 > self.topic_timeout:
                self.get_logger().warn("Timeout on /estimated_pose. Last message older than specified limit.")

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r] <filename>.json")
        sys.exit()

    mode = None
    filename = None

    if sys.argv[1] == '-w':
        mode = 'write'
        if len(sys.argv) > 2:
            filename = sys.argv[2]
        else:
            print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r] <filename>.json")
            sys.exit()
    elif sys.argv[1] == '-r':
        mode = 'read'
        if len(sys.argv) > 2:
            filename = sys.argv[2]
        else:
            print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r] <filename>.json")
            sys.exit()
    else:
        print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r] <filename>.json")
        sys.exit()

    if mode and filename:
        node = Nav2WaypointMaker(mode, filename)
        rclpy.spin(node)
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()