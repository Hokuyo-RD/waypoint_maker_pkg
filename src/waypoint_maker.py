#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import json
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data
import math
from builtin_interfaces.msg import Duration as DurationMsg
import tkinter as tk
from tkinter import simpledialog, messagebox, Listbox, Scrollbar
import threading

class Nav2WaypointMakerGUI(tk.Toplevel):
    def __init__(self, parent, waypoint_maker_node):
        super().__init__(parent)
        self.title("Waypoint Editor")
        self.waypoint_maker_node = waypoint_maker_node
        self.waypoint_list = waypoint_maker_node.waypoints
        self.listbox = Listbox(self, width=50, height=15)
        self.scrollbar = Scrollbar(self)
        self.listbox.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.listbox.yview)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        button_frame = tk.Frame(self)
        tk.Button(button_frame, text="Add Waypoint (2D Goal)", command=self.add_waypoint).pack(fill=tk.X)
        tk.Button(button_frame, text="Remove Selected", command=self.remove_waypoint).pack(fill=tk.X)
        tk.Button(button_frame, text="Replace Selected (2D Goal)", command=self.replace_waypoint).pack(fill=tk.X)
        tk.Button(button_frame, text="Save Waypoints", command=self.save_waypoints).pack(fill=tk.X)
        self.update_listbox()
        button_frame.pack(side=tk.TOP, fill=tk.X)

    def update_listbox(self):
        self.listbox.delete(0, tk.END)
        for i, waypoint in enumerate(self.waypoint_list):
            pos = waypoint.pose.position
            ori = waypoint.pose.orientation
            self.listbox.insert(tk.END, f"[{i}] x:{pos.x:.2f}, y:{pos.y:.2f}, z:{ori.z:.2f}, w:{ori.w:.2f}")

    def add_waypoint(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            insert_index = selected_index[0] + 1 # 選択された項目の次に追加
            # messagebox.showinfo("Add Waypoint", f"Use the 2D Goal Pose tool in rviz to set the new waypoint to insert after index {insert_index - 1}.")
            self.waypoint_maker_node.is_adding = True
            self.waypoint_maker_node.insert_index = insert_index
        else:
            messagebox.showinfo("Add Waypoint", "Use the 2D Goal Pose tool in rviz to set the new waypoint at the end of the list.")
            self.waypoint_maker_node.is_adding = True
            self.waypoint_maker_node.insert_index = len(self.waypoint_list)

    def remove_waypoint(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            index_to_remove = selected_index[0]
            del self.waypoint_list[index_to_remove]
            self.waypoint_maker_node.save_waypoints_to_json()
            self.waypoint_maker_node.publish_waypoints_for_vis()
            self.waypoint_maker_node.rewrite_marker()
            self.update_listbox()
        else:
            messagebox.showerror("Error", "Please select a waypoint to remove.")

    def replace_waypoint(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            self.waypoint_maker_node.replace_index = selected_index[0]
            messagebox.showinfo("Replace Waypoint", "Use the 2D Goal Pose tool in rviz to set the new pose for the selected waypoint.")
        else:
            messagebox.showerror("Error", "Please select a waypoint to replace.")

    def save_waypoints(self):
        self.waypoint_maker_node.save_waypoints_to_json()
        messagebox.showinfo("Info", "Waypoints saved to file.")

class Nav2WaypointMaker(Node):
    def __init__(self, mode, filename):
        super().__init__('nav2_waypoint_maker_' + mode)
        self.waypoints = []
        self.mode = mode
        self.filename = filename
        self.replace_index = -1
        self.is_adding = False
        self.insert_index = -1 # 挿入位置を保持する変数
        self.previous_pose = None
        self.last_message_time = self.get_clock().now()
        self.joy_button = self.declare_parameter('waypoint_button', 1).value
        self.distance_threshold = self.declare_parameter('auto_waypoint_distance', 5.0).value
        self.topic_timeout = self.declare_parameter('estimated_pose_timeout', 20.0).value
        self.lio_loc_pose = PoseStamped()

        self.waypoint_pub = self.create_publisher(PoseArray, 'waypoints', 10)
        self.marker_pub = self.create_publisher(Marker, 'waypoint_markers', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        if self.mode == 'write':
            self.load_waypoints_from_json()
            self.amcl_sub = self.create_subscription(PoseStamped, '/estimated_pose', self.amcl_callback, qos_profile_sensor_data)
            self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 10)
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
            self.remove_sub = self.create_subscription(Int16, '/remove_waypoint', self.remove_callback, 10)
            self.insert_sub = self.create_subscription(Int16, '/insert_waypoint', self.insert_callback, 10)
            self.timer = self.create_timer(1.0, self.check_timeout)
        elif self.mode == 'read':
            self.load_waypoints_from_json()
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
        elif self.mode == 'edit':
            self.load_waypoints_from_json()
            self.gui = Nav2WaypointMakerGUI(None, self)
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
            self.get_logger().info("Edit mode enabled with separate GUI.")
        else:
            self.get_logger().error(f"Invalid mode: {self.mode}. Use 'write', 'read', or 'edit'.")
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
            self.get_logger().warn(f"Waypoint file {self.filename} not found. Creating a new one.")
            self.save_waypoints_to_json()
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode JSON in {self.filename}. Please check the file format.")
        except IndexError:
            self.get_logger().error(f"Invalid JSON format in {self.filename}. Expected [[x, y, 0.0], [0.0, 0.0, z, w]].")

    def save_waypoints_to_json(self):
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
        marker_data.ns = "waypoint_markers"
        marker_data.action = Marker.DELETEALL
        self.marker_pub.publish(marker_data)

        marker_data.action = Marker.ADD
        counter = 0
        marker_data.color.a = 1.0
        marker_data.scale.z = 0.5
        marker_data.lifetime = DurationMsg()
        marker_data.type = Marker.TEXT_VIEW_FACING

        for i, pose_stamped in enumerate(self.waypoints):
            marker_data.id = counter
            marker_data.pose.position.x = pose_stamped.pose.position.x
            marker_data.pose.position.y = pose_stamped.pose.position.y
            marker_data.pose.orientation.z = pose_stamped.pose.orientation.z
            marker_data.pose.orientation.w = pose_stamped.pose.orientation.w
            marker_data.text = str(i)
            marker_data.color.r = 0.0
            marker_data.color.g = 0.0
            marker_data.color.b = 1.0
            self.marker_pub.publish(marker_data)
            counter += 1

    def goal_callback(self, msg):
        if self.mode == 'edit' and self.replace_index != -1:
            if msg.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                msg.header.frame_id = "map"
            self.waypoints[self.replace_index] = msg
            self.replace_index = -1
            self.is_adding = False
            self.insert_index = -1 # 念のためリセット
            self.gui.update_listbox()
            self.save_waypoints_to_json()
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
            self.get_logger().info(f"Waypoint at index replaced via /goal_pose")
        elif self.mode == 'edit' and self.is_adding and self.insert_index != -1:
            if msg.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                msg.header.frame_id = "map"
            self.insert_waypoint_at(self.insert_index, msg)
            self.is_adding = False
            self.insert_index = -1
            self.gui.update_listbox()
            self.save_waypoints_to_json()
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
            self.get_logger().info(f"Waypoint inserted at index {self.insert_index} via /goal_pose")
        elif self.mode == 'edit' and self.is_adding: # 選択なしで追加する場合（末尾に追加）
            if msg.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                msg.header.frame_id = "map"
            self.waypoints.append(msg)
            self.is_adding = False
            self.insert_index = -1 # 念のためリセット
            self.gui.update_listbox()
            self.save_waypoints_to_json()
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
            self.get_logger().info("Waypoint added at the end via /goal_pose")
        elif self.mode == 'write':
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
            self.get_logger().info("Waypoint added from /goal_pose (write mode)")

    def insert_waypoint_at(self, index, pose_stamped):
        if 0 <= index <= len(self.waypoints):
            self.waypoints.insert(index, pose_stamped)
            self.get_logger().info(f"Waypoint inserted at index {index}")
        else:
            self.get_logger().warn(f"Invalid index for insertion: {index}")

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

    def amcl_waypoint_append(self):
        if self.mode == 'write':
            waypoint = PoseStamped()
            waypoint.header = self.lio_loc_pose.header
            waypoint.pose = self.lio_loc_pose
        if waypoint.header.frame_id != "map":
            self.get_logger().warn("Appending waypoint in non-map frame. Assuming map frame.")
            waypoint.header.frame_id = "map"
        self.waypoints.append(waypoint)
        self.save_waypoints_to_json()
        self.rewrite_marker()
        self.publish_waypoints_for_vis()
        self.get_logger().info("Waypoint added from /estimated_pose")

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

    def remove_callback(self, msg):
        if self.mode == 'write':
            index_to_remove = msg.data
            if 0 <= index_to_remove < len(self.waypoints):
                del self.waypoints[index_to_remove]
                self.save_waypoints_to_json()
                self.rewrite_marker()
                self.publish_waypoints_for_vis()
                self.get_logger().info(f"Removed waypoint at index {index_to_remove} via topic")
            else:
                self.get_logger().warn(f"Invalid index to remove: {index_to_remove}")
            self.last_message_time = self.get_clock().now()

    def insert_callback(self, msg):
        if self.mode == 'write':
            index_to_insert = msg.data
            if 0 <= index_to_insert <= len(self.waypoints):
                waypoint = PoseStamped()
                waypoint.header = self.lio_loc_pose.header
                waypoint.pose = self.lio_loc_pose.pose
                if waypoint.header.frame_id != "map":
                    self.get_logger().warn("Inserting waypoint in non-map frame. Assuming map frame.")
                    waypoint.header.frame_id = "map"
                self.waypoints.insert(index_to_insert, waypoint)
                self.save_waypoints_to_json()
                self.rewrite_marker()
                self.publish_waypoints_for_vis()
                self.get_logger().info(f"Inserted waypoint at index {index_to_insert} via topic")
            else:
                self.get_logger().warn(f"Invalid index to insert: {index_to_insert}")
            self.last_message_time = self.get_clock().now()

    def check_timeout(self):
        if self.mode == 'write':
            current_time = self.get_clock().now()
            if (current_time - self.last_message_time).to_sec() > self.topic_timeout:
                self.get_logger().warn(f"Timeout on /estimated_pose topic. Last message received {self.topic_timeout} seconds ago.")
                self.previous_pose = None

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r|-e] <filename>.json")
        sys.exit()

    mode = None
    filename = None

    if sys.argv[1] == '-w':
        mode = 'write'
    elif sys.argv[1] == '-r':
        mode = 'read'
    elif sys.argv[1] == '-e':
        mode = 'edit'
    else:
        print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r|-e] <filename>.json")
        sys.exit()

    if len(sys.argv) > 2:
        filename = sys.argv[2]
    else:
        print("Usage: ros2 run waypoint_maker_pkg waypoint_maker [-w|-r|-e] <filename>.json")
        sys.exit()

    if mode and filename:
        node = Nav2WaypointMaker(mode, filename)
        if mode == 'edit':
            thread = threading.Thread(target=ros_spin, args=(node,))
            thread.start()
            tk.mainloop()
        else:
            rclpy.spin(node)
            node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()