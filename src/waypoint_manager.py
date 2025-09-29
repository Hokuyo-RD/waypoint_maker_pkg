#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import sys
import json
import os
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16, String
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data
import math
from builtin_interfaces.msg import Duration as DurationMsg
import tkinter as tk
from tkinter import messagebox, Listbox, Scrollbar, ttk
import threading
import time
import rclpy.parameter
from rcl_interfaces.srv import SetParameters
from nav2_msgs.action import FollowWaypoints
from action_msgs.msg import GoalStatus
import argparse

class Nav2WaypointManagerGUI(tk.Toplevel):
    def __init__(self, parent, waypoint_manager_node):
        super().__init__(parent)
        self.title("Waypoint Editor")
        self.waypoint_manager_node = waypoint_manager_node
        self.waypoint_list = waypoint_manager_node.waypoints

        main_frame = tk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        save_button_frame = tk.Frame(main_frame)
        save_button_frame.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        tk.Button(save_button_frame, text="Save", command=self.save_waypoints).pack(side=tk.LEFT, padx=2)
        tk.Button(save_button_frame, text="Save and Exit", command=self.save_and_exit).pack(side=tk.LEFT, padx=2)

        list_frame = tk.LabelFrame(main_frame, text="Waypoints")
        list_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.listbox = Listbox(list_frame, width=50, height=15)
        self.scrollbar = Scrollbar(list_frame)
        self.listbox.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.listbox.yview)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox.bind('<<ListboxSelect>>', self.on_listbox_select)

        right_panel_frame = tk.Frame(main_frame)
        right_panel_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        button_frame = tk.LabelFrame(right_panel_frame, text="Actions")
        button_frame.pack(fill=tk.X, pady=2)

        tk.Button(button_frame, text="Add Waypoint (2D Goal)", command=self.add_waypoint).pack(fill=tk.X, pady=2)
        tk.Button(button_frame, text="Remove Selected", command=self.remove_waypoint).pack(fill=tk.X, pady=2)
        tk.Button(button_frame, text="Replace Selected (2D Goal)", command=self.replace_waypoint).pack(fill=tk.X, pady=2)

        attribute_frame = tk.LabelFrame(right_panel_frame, text="Edit Attributes")
        attribute_frame.pack(fill=tk.X, pady=5)

        # 修正: XY Tolerance と Yaw Tolerance の入力フィールドを上部に移動
        tk.Label(attribute_frame, text="XY Tolerance (m):", anchor="w").grid(row=0, column=0, padx=5, pady=2, sticky="W")
        self.xy_tolerance_var = tk.StringVar()
        tk.Entry(attribute_frame, textvariable=self.xy_tolerance_var).grid(row=0, column=1, columnspan=3, padx=5, pady=2, sticky="EW")

        tk.Label(attribute_frame, text="Yaw Tolerance (rad):", anchor="w").grid(row=1, column=0, padx=5, pady=2, sticky="W")
        self.yaw_tolerance_var = tk.StringVar()
        tk.Entry(attribute_frame, textvariable=self.yaw_tolerance_var).grid(row=1, column=1, columnspan=3, padx=5, pady=2, sticky="EW")

        # 既存の attribute value の入力フィールド
        tk.Label(attribute_frame, text="Value:", anchor="w").grid(row=2, column=0, padx=5, pady=2, sticky="W")
        self.attr_value_var = tk.StringVar()
        self.attr_value_entry = tk.Entry(attribute_frame, textvariable=self.attr_value_var)
        self.attr_value_entry.grid(row=2, column=1, columnspan=3, padx=5, pady=2, sticky="EW")

        tk.Label(attribute_frame, text="Type:", anchor="w").grid(row=3, column=0, padx=5, pady=2, sticky="W")

        tk.Button(attribute_frame, text="Normal", command=lambda: self.set_attribute_type('normal')).grid(row=3, column=1, padx=2, pady=2, sticky="EW")
        tk.Label(attribute_frame, text="Normal travel speed. Please set 0.0").grid(row=4, column=1, columnspan=1, padx=2, pady=2, sticky="EW")

        tk.Button(attribute_frame, text="Stop", command=lambda: self.set_attribute_type('stop')).grid(row=3, column=2, padx=2, pady=2, sticky="EW")
        tk.Label(attribute_frame, text="Stop and wait. Unit: seconds").grid(row=4, column=2, columnspan=1, padx=2, pady=2, sticky="EW")

        tk.Button(attribute_frame, text="Slow", command=lambda: self.set_attribute_type('slow')).grid(row=3, column=3, padx=2, pady=2, sticky="EW")
        tk.Label(attribute_frame, text="Travel at a specified speed. Unit: m/s").grid(row=4, column=3, columnspan=1, padx=2, pady=2, sticky="EW")

        tk.Label(attribute_frame, text="Please input the value, then you push the button you want to set for parameters.", fg="gray", font=("Arial", 8)).grid(row=5, column=1, columnspan=3, padx=5, pady=2, sticky="W")

        attribute_frame.grid_columnconfigure(1, weight=1)
        attribute_frame.grid_columnconfigure(2, weight=1)
        attribute_frame.grid_columnconfigure(3, weight=1)

        self.update_listbox()

    def on_listbox_select(self, event):
        selected_index = self.listbox.curselection()
        if not selected_index:
            return

        index = selected_index[0]
        if index < len(self.waypoint_manager_node.attributes):
            attr = self.waypoint_manager_node.attributes[index]
            self.attr_value_var.set(attr.get('value', ''))
            self.xy_tolerance_var.set(attr.get('xy_tolerance', ''))
            self.yaw_tolerance_var.set(attr.get('yaw_tolerance', ''))

    def set_attribute_type(self, attr_type):
        selected_index = self.listbox.curselection()
        if not selected_index:
            messagebox.showerror("Error", "Please select a waypoint to apply attributes.")
            return

        index = selected_index[0]
        try:
            attr_value = float(self.attr_value_var.get()) if self.attr_value_var.get() else 0.0
            xy_tolerance = float(self.xy_tolerance_var.get()) if self.xy_tolerance_var.get() else self.waypoint_manager_node.get_parameter("xy_goal_tolerance").value
            yaw_tolerance = float(self.yaw_tolerance_var.get()) if self.yaw_tolerance_var.get() else self.waypoint_manager_node.get_parameter("yaw_goal_tolerance").value

            if index < len(self.waypoint_manager_node.attributes):
                self.waypoint_manager_node.attributes[index]["type"] = attr_type
                self.waypoint_manager_node.attributes[index]["value"] = attr_value
                self.waypoint_manager_node.attributes[index]["xy_tolerance"] = xy_tolerance
                self.waypoint_manager_node.attributes[index]["yaw_tolerance"] = yaw_tolerance
            else:
                messagebox.showerror("Error", "Waypoint attributes not found for this index.")
                return

            self.waypoint_manager_node.save_waypoints_to_json()
            self.update_listbox_and_keep_selection(index)
            messagebox.showinfo("Success", f"Attributes '{attr_type}' applied successfully.")
        except ValueError:
            messagebox.showerror("Error", "Invalid value. Please enter a number.")

    def update_listbox(self):
        self.listbox.delete(0, tk.END)
        for i, waypoint in enumerate(self.waypoint_list):
            pos = waypoint.pose.position
            ori = waypoint.pose.orientation
            attr = self.waypoint_manager_node.attributes[i] if i < len(self.waypoint_manager_node.attributes) else {"type": "normal", "value": 0}
            self.listbox.insert(tk.END, f"[{i}] x:{pos.x:.2f}, y:{pos.y:.2f}, z:{ori.z:.2f}, w:{ori.w:.2f} | Type: {attr.get('type')}, Value: {attr.get('value')}, XY_tol: {attr.get('xy_tolerance')}, Yaw_tol: {attr.get('yaw_tolerance')}")

    def update_listbox_and_keep_selection(self, selected_index):
        self.listbox.delete(0, tk.END)
        for i, waypoint in enumerate(self.waypoint_list):
            pos = waypoint.pose.position
            ori = waypoint.pose.orientation
            attr = self.waypoint_manager_node.attributes[i] if i < len(self.waypoint_manager_node.attributes) else {"type": "normal", "value": 0}
            self.listbox.insert(tk.END, f"[{i}] x:{pos.x:.2f}, y:{pos.y:.2f}, z:{ori.z:.2f}, w:{ori.w:.2f} | Type: {attr.get('type')}, Value: {attr.get('value')}, XY_tol: {attr.get('xy_tolerance')}, Yaw_tol: {attr.get('yaw_tolerance')}")

        self.listbox.selection_set(selected_index)
        self.listbox.activate(selected_index)
        self.listbox.see(selected_index)

    def add_waypoint(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            insert_index = selected_index[0] + 1
            self.waypoint_manager_node.is_adding = True
            self.waypoint_manager_node.insert_index = insert_index
        else:
            messagebox.showinfo("Add Waypoint", "Use the 2D Goal Pose tool in rviz to set the new waypoint at the end of the list.")
            self.waypoint_manager_node.is_adding = True
            self.waypoint_manager_node.insert_index = len(self.waypoint_list)

    def remove_waypoint(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            index_to_remove = selected_index[0]
            del self.waypoint_list[index_to_remove]
            if index_to_remove < len(self.waypoint_manager_node.attributes):
                del self.waypoint_manager_node.attributes[index_to_remove]
            self.waypoint_manager_node.save_waypoints_to_json()
            self.waypoint_manager_node.update_waypoint_visualization()
            self.update_listbox()
        else:
            messagebox.showerror("Error", "Please select a waypoint to remove.")

    def replace_waypoint(self):
        selected_index = self.listbox.curselection()
        if selected_index:
            self.waypoint_manager_node.replace_index = selected_index[0]
        else:
            messagebox.showerror("Error", "Please select a waypoint to replace.")

    def save_waypoints(self):
        self.waypoint_manager_node.save_waypoints_to_json()
        messagebox.showinfo("Info", "Waypoints saved to file.")

    def save_and_exit(self):
        self.waypoint_manager_node.save_waypoints_to_json()
        messagebox.showinfo("Info", "Waypoints saved to file. Exiting GUI.")
        self.destroy()

class Nav2WaypointManager(Node):
    def __init__(self, mode, filename, is_looping=True):
        super().__init__('nav2_waypoint_manager_' + mode)

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
        self.mode = mode
        self.filename = filename
        self.replace_index = -1
        self.is_adding = False
        self.insert_index = -1
        self.previous_pose = None
        self.last_message_time = self.get_clock().now()
        self.joy_button = self.declare_parameter('waypoint_button', 1).value
        self.distance_threshold = self.declare_parameter('auto_waypoint_distance', 5.0).value
        self.topic_timeout = self.declare_parameter('estimated_pose_timeout', 20.0).value
        self.lio_loc_pose = PoseStamped()
        self.is_looping = is_looping

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.waypoint_pub = self.create_publisher(PoseArray, 'waypoints', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'waypoint_marker_array', 10)

        self.set_parameters_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_parameters service not available, waiting again...')
        self.original_speed = self.declare_parameter('original_speed', 0.5).value
        self.declare_parameter("xy_goal_tolerance", 0.2)
        self.declare_parameter("yaw_goal_tolerance", 0.1)

        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.change_params = []

        if self.mode == 'write':
            self.load_waypoints_from_json()
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
            self.lio_loc_sub = self.create_subscription(PoseStamped, '/estimated_pose', self.lio_loc_callback, qos_profile_sensor_data)
            self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 10)
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
            self.remove_sub = self.create_subscription(Int16, '/remove_waypoint', self.remove_callback, 10)
            self.insert_sub = self.create_subscription(Int16, '/insert_waypoint', self.insert_callback, 10)
            self.timer = self.create_timer(1.0, self.check_timeout)
        elif self.mode == 'execute':
            self.load_waypoints_from_json()
            self.update_waypoint_visualization()
            self.odometry_switch_type_sub = self.create_subscription(String, '/odometry/switch/type', self.odometry_switch_type_callback, 10)
            self.initialize_cmdvel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self.wait_for_stable_odometry_switch_type()
            self.send_waypoints_goal()
            self.get_logger().info("Execute mode enabled. Starting navigation...")
        elif self.mode == 'edit':
            self.load_waypoints_from_json()
            self.update_waypoint_visualization()
            self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
            self.get_logger().info("Edit mode enabled. GUI will be initialized from main.")
        elif self.mode == 'read':
            self.load_waypoints_from_json()
            self.update_waypoint_visualization()
            self.get_logger().info("Read mode enabled. Waypoints are loaded and visualized in Rviz.")
            self.timer = self.create_timer(5.0, self.republish_waypoints)
        else:
            self.get_logger().error(f"Invalid mode: {self.mode}. Use 'write', 'execute', 'edit', or 'read'.")
            sys.exit()

    def republish_waypoints(self):
        self.update_waypoint_visualization()
        self.get_logger().info("Re-publishing waypoints for visualization.")

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
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = "map"
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = "waypoint_markers"
        marker_array.markers.append(delete_marker)
        self.marker_array_pub.publish(marker_array)
        time.sleep(0.1)

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
            # ウェイポイントの向きを設定して、テキストが正しく表示されるようにする
            marker.pose.orientation.x = pose_stamped.pose.orientation.x
            marker.pose.orientation.y = pose_stamped.pose.orientation.y
            marker.pose.orientation.z = pose_stamped.pose.orientation.z
            marker.pose.orientation.w = pose_stamped.pose.orientation.w
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            # テキスト文字列をカスタマイズして、x, y, z, qx, qy, qz, qwの座標と向きを表示
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z
            qx = pose_stamped.pose.orientation.x
            qy = pose_stamped.pose.orientation.y
            qz = pose_stamped.pose.orientation.z
            qw = pose_stamped.pose.orientation.w

            marker.text = f"[{i}]\nx: {x:.2f}\ny: {y:.2f}\nz: {z:.2f}\nqx: {qx:.2f}\nqy: {qy:.2f}\nqz: {qz:.2f}\nqw: {qw:.2f}"

            marker.lifetime = DurationMsg()
            marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)


    def goal_callback(self, msg):
        if self.mode == 'edit' and self.replace_index != -1:
            if msg.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                msg.header.frame_id = "map"
            self.waypoints[self.replace_index] = msg
            self.replace_index = -1
            self.is_adding = False
            self.insert_index = -1
            self.gui.update_listbox()
            self.save_waypoints_to_json()
            self.update_waypoint_visualization()
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
            self.update_waypoint_visualization()
            self.get_logger().info(f"Waypoint inserted at index {self.insert_index} via /goal_pose")
        elif self.mode == 'edit' and self.is_adding:
            if msg.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                msg.header.frame_id = "map"
            self.waypoints.append(msg)
            # 修正: 新しいウェイポイントの属性にデフォルトの許容範囲を追加
            self.attributes.append({"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14})
            self.is_adding = False
            self.insert_index = -1
            self.gui.update_listbox()
            self.save_waypoints_to_json()
            self.update_waypoint_visualization()
            self.get_logger().info("Waypoint added at the end via /goal_pose")
        elif self.mode == 'write':
            waypoint = PoseStamped()
            waypoint.header = msg.header
            waypoint.pose = msg.pose
            if waypoint.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                waypoint.header.frame_id = "map"
            self.waypoints.append(waypoint)
            # 修正: writeモードでも新しいウェイポイントの属性にデフォルトの許容範囲を追加
            self.attributes.append({"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14})
            self.save_waypoints_to_json()
            self.update_waypoint_visualization()
            self.get_logger().info("Waypoint added from /goal_pose (write mode)")

    def insert_waypoint_at(self, index, pose_stamped):
        if 0 <= index <= len(self.waypoints):
            self.waypoints.insert(index, pose_stamped)
            # 修正: 挿入するウェイポイントの属性にデフォルトの許容範囲を追加
            self.attributes.insert(index, {"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14})
            self.get_logger().info(f"Waypoint inserted at index {index}")
        else:
            self.get_logger().warn(f"Invalid index for insertion: {index}")

    def lio_loc_callback(self, msg):
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
                    self.lio_loc_waypoint_append()
                    self.previous_pose = self.lio_loc_pose

    def lio_loc_waypoint_append(self):
        if self.mode == 'write':
            waypoint = PoseStamped()
            waypoint.header = self.lio_loc_pose.header
            waypoint.pose = self.lio_loc_pose.pose
        if waypoint.header.frame_id != "map":
            self.get_logger().warn("Appending waypoint in non-map frame. Assuming map frame.")
            waypoint.header.frame_id = "map"
        self.waypoints.append(waypoint)
        # 修正: 自動追加するウェイポイントの属性にデフォルトの許容範囲を追加
        self.attributes.append({"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14})
        self.save_waypoints_to_json()
        self.update_waypoint_visualization()
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
                # 修正: ジョイスティックで追加するウェイポイントの属性にデフォルトの許容範囲を追加
                self.attributes.append({"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14})
                self.save_waypoints_to_json()
                self.update_waypoint_visualization()
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
                del self.attributes[index_to_remove]
                self.save_waypoints_to_json()
                self.update_waypoint_visualization()
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
                # 修正: 挿入するウェイポイントの属性にデフォルトの許容範囲を追加
                self.attributes.insert(index_to_insert, {"type": "normal", "value": 0, "xy_tolerance": 1.0, "yaw_tolerance": 3.14})
                self.save_waypoints_to_json()
                self.update_waypoint_visualization()
                self.get_logger().info(f"Inserted waypoint at index {index_to_insert} via topic")
            else:
                self.get_logger().warn(f"Invalid index to insert: {index_to_insert}")
            self.last_message_time = self.get_clock().now()

    def check_timeout(self):
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_message_time
        if time_diff.nanoseconds / 1e9 > self.topic_timeout:
            self.get_logger().warn("Timeout on /estimated_pose. Last message older than specified limit.")

    def wait_for_stable_odometry_switch_type(self):
        if self.use_gnss_switch_flg:
            while self.odometry_switch_type == "LIO raw":
                msg = Twist()
                msg.linear.x = self.initialize_cmd_vel_linear_x
                # msg.angular.z = msg.linear.x / self.initialize_radius
                msg.angular.z = 0
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
        completed_waypoint_index = feedback.current_waypoint - 1

        if completed_waypoint_index >= 0 and completed_waypoint_index < len(self.attributes):
            attribute = self.attributes[completed_waypoint_index]
            self.process_waypoint_attribute(attribute)

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
            time.sleep(attr_value)
            self.get_logger().info("Resuming navigation.")

        elif attr_type == "slow":
            param = rclpy.parameter.Parameter('max_vel_x', rclpy.Parameter.Type.DOUBLE, float(attr_value))
            request.parameters.append(param.to_parameter_msg())

            self.get_logger().info(f"Setting max_vel_x to {attr_value} m/s.")
            self.set_parameters_client.call_async(request)

        elif attr_type == "normal":
            param = rclpy.parameter.Parameter('max_vel_x', rclpy.Parameter.Type.DOUBLE, self.original_speed)
            request.parameters.append(param.to_parameter_msg())
            self.get_logger().info(f"Setting max_vel_x to original speed {self.original_speed} m/s.")
            self.set_parameters_client.call_async(request)

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='A waypoint manager for Nav2.')
    parser.add_argument('-w', '--write', action='store_true', help='Set mode to write.')
    parser.add_argument('-x', '--execute', action='store_true', help='Set mode to execute.')
    parser.add_argument('-e', '--edit', action='store_true', help='Set mode to edit.')
    parser.add_argument('-r', '--read', action='store_true', help='Set mode to read.')
    parser.add_argument('filename', type=str, help='The name of the waypoint JSON file.')
    parser.add_argument('--once', action='store_true', help='Execute waypoints only once, do not loop.')

    parsed_args, ros_args = parser.parse_known_args()

    mode = None
    if parsed_args.write:
        mode = 'write'
    elif parsed_args.execute:
        mode = 'execute'
    elif parsed_args.edit:
        mode = 'edit'
    elif parsed_args.read:
        mode = 'read'

    if mode and parsed_args.filename:
        node = Nav2WaypointManager(mode, parsed_args.filename, not parsed_args.once)
        if mode == 'edit':
            root = tk.Tk()
            root.withdraw()

            node.gui = Nav2WaypointManagerGUI(root, node)

            thread = threading.Thread(target=ros_spin, args=(node,))
            thread.start()

            root.mainloop()
        else:
            rclpy.spin(node)
            node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()