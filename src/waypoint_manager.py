#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, QoSDurabilityPolicy
import sys
import json
import os
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
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

class Nav2WaypointManagerGUI(tk.Toplevel):
    def __init__(self, parent, waypoint_manager_node):
        super().__init__(parent)
        self.title("Waypoint Editor")
        self.waypoint_manager_node = waypoint_manager_node
        self.waypoint_list = waypoint_manager_node.waypoints
        
        main_frame = tk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 新しい保存ボタンフレームを左上に配置
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
        
        # 属性タイプと説明のグリッドを定義
        tk.Label(attribute_frame, text="Type:", anchor="w").grid(row=0, column=0, padx=5, pady=2, sticky="W")

        # Normal
        tk.Button(attribute_frame, text="Normal", command=lambda: self.set_attribute_type('normal')).grid(row=0, column=1, padx=2, pady=2, sticky="EW")
        tk.Label(attribute_frame, text="Normal travel speed. Please set 0.0").grid(row=1, column=1, columnspan=1, padx=2, pady=2, sticky="EW")

        # Stop
        tk.Button(attribute_frame, text="Stop", command=lambda: self.set_attribute_type('stop')).grid(row=0, column=2, padx=2, pady=2, sticky="EW")
        tk.Label(attribute_frame, text="Stop and wait. Unit: seconds").grid(row=1, column=2, columnspan=1, padx=2, pady=2, sticky="EW")

        # Slow
        tk.Button(attribute_frame, text="Slow", command=lambda: self.set_attribute_type('slow')).grid(row=0, column=3, padx=2, pady=2, sticky="EW")
        tk.Label(attribute_frame, text="Travel at a specified speed. Unit: m/s").grid(row=1, column=3, columnspan=1, padx=2, pady=2, sticky="EW")

        tk.Label(attribute_frame, text="Value:").grid(row=2, column=0, padx=5, pady=2, sticky="W")
        self.attr_value_var = tk.StringVar()
        self.attr_value_entry = tk.Entry(attribute_frame, textvariable=self.attr_value_var)
        self.attr_value_entry.grid(row=2, column=1, columnspan=3, padx=5, pady=2, sticky="EW")

        # 単位のラベルを追加
        tk.Label(attribute_frame, text="Please input the value, then you push the button you want to set for parameters.", fg="gray", font=("Arial", 8)).grid(row=3, column=1, columnspan=3, padx=5, pady=2, sticky="W")
        
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
            
    def set_attribute_type(self, attr_type):
        selected_index = self.listbox.curselection()
        if not selected_index:
            messagebox.showerror("Error", "Please select a waypoint to apply attributes.")
            return

        index = selected_index[0]
        try:
            attr_value = float(self.attr_value_var.get()) if self.attr_value_var.get() else 0.0
            
            if index < len(self.waypoint_manager_node.attributes):
                self.waypoint_manager_node.attributes[index]["type"] = attr_type
                self.waypoint_manager_node.attributes[index]["value"] = attr_value
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
            self.listbox.insert(tk.END, f"[{i}] x:{pos.x:.2f}, y:{pos.y:.2f}, z:{ori.z:.2f}, w:{ori.w:.2f} | Type: {attr.get('type')}, Value: {attr.get('value')}")

    def update_listbox_and_keep_selection(self, selected_index):
        """リストボックスを更新し、指定されたインデックスの選択状態を維持する。"""
        self.listbox.delete(0, tk.END)
        for i, waypoint in enumerate(self.waypoint_list):
            pos = waypoint.pose.position
            ori = waypoint.pose.orientation
            attr = self.waypoint_manager_node.attributes[i] if i < len(self.waypoint_manager_node.attributes) else {"type": "normal", "value": 0}
            self.listbox.insert(tk.END, f"[{i}] x:{pos.x:.2f}, y:{pos.y:.2f}, z:{ori.z:.2f}, w:{ori.w:.2f} | Type: {attr.get('type')}, Value: {attr.get('value')}")
        
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
            self.waypoint_manager_node.publish_waypoints_for_vis()
            self.waypoint_manager_node.rewrite_marker()
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
        self.marker_pub = self.create_publisher(Marker, 'waypoint_markers', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.set_parameters_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_parameters service not available, waiting again...')
        self.original_speed = self.declare_parameter('original_speed', 0.5).value

        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.change_params = []

        if self.mode == 'write':
            self.load_waypoints_from_json()
            self.lio_loc_sub = self.create_subscription(PoseStamped, '/estimated_pose', self.lio_loc_callback, qos_profile_sensor_data)
            self.initialpose_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 10)
            self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, qos_profile_sensor_data)
            self.remove_sub = self.create_subscription(Int16, '/remove_waypoint', self.remove_callback, 10)
            self.insert_sub = self.create_subscription(Int16, '/insert_waypoint', self.insert_callback, 10)
            self.timer = self.create_timer(1.0, self.check_timeout)
        elif self.mode == 'execute':
            self.load_waypoints_from_json()
            self.publish_waypoints_for_vis()
            self.rewrite_marker()
            self.send_waypoints_goal()
            self.get_logger().info("Execute mode enabled. Starting navigation...")
        elif self.mode == 'edit':
            self.load_waypoints_from_json()
            # GUIはmain関数で初期化するため、ここでは何もしない
            self.get_logger().info("Edit mode enabled. GUI will be initialized from main.")
        else:
            self.get_logger().error(f"Invalid mode: {self.mode}. Use 'write', 'execute', or 'edit'.")
            sys.exit()

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
                    
                    attribute = item[2] if len(item) > 2 else {"type": "normal", "value": 0}
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
            attribute = self.attributes[i] if i < len(self.attributes) else {"type": "normal", "value": 0}
            data.append([position, orientation, attribute])
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
            self.insert_index = -1
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
        elif self.mode == 'edit' and self.is_adding:
            if msg.header.frame_id != "map":
                self.get_logger().warn("Received goal in non-map frame. Assuming map frame.")
                msg.header.frame_id = "map"
            self.waypoints.append(msg)
            self.attributes.append({"type": "normal", "value": 0})
            self.is_adding = False
            self.insert_index = -1
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
            self.attributes.append({"type": "normal", "value": 0})
            self.save_waypoints_to_json()
            self.rewrite_marker()
            self.publish_waypoints_for_vis()
            self.get_logger().info("Waypoint added from /goal_pose (write mode)")

    def insert_waypoint_at(self, index, pose_stamped):
        if 0 <= index <= len(self.waypoints):
            self.waypoints.insert(index, pose_stamped)
            self.attributes.insert(index, {"type": "normal", "value": 0})
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
        self.attributes.append({"type": "normal", "value": 0})
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
                self.attributes.append({"type": "normal", "value": 0})
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
                del self.attributes[index_to_remove]
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
                self.attributes.insert(index_to_insert, {"type": "normal", "value": 0})
                self.save_waypoints_to_json()
                self.rewrite_marker()
                self.publish_waypoints_for_vis()
                self.get_logger().info(f"Inserted waypoint at index {index_to_insert} via topic")
            else:
                self.get_logger().warn(f"Invalid index to insert: {index_to_insert}")
            self.last_message_time = self.get_clock().now()

    def check_timeout(self):
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_message_time
        if time_diff.nanoseconds / 1e9 > self.topic_timeout:
            self.get_logger().warn("Timeout on /estimated_pose. Last message older than specified limit.")

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
        self._action_client_future = self.action_client.send_goal_async(goal_msg)
        self._action_client_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by action server')
            return

        self.get_logger().info('Goal accepted! Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        # Fix: Use 'missed_waypoints' instead of 'completed_waypoints'
        last_completed_waypoint_index = len(self.waypoints) - len(result.missed_waypoints) - 1
        
        if last_completed_waypoint_index >= 0 and last_completed_waypoint_index < len(self.attributes):
            attribute = self.attributes[last_completed_waypoint_index]
            self.process_waypoint_attribute(attribute)
        
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

        self.get_logger().info(f"Processing attribute: type={attr_type}, value={attr_value}")

        if attr_type == "stop":
            self.get_logger().info(f"Stopping for {attr_value} seconds...")
            time.sleep(attr_value)
            self.get_logger().info("Resuming navigation.")
            
        elif attr_type == "slow":
            request = SetParameters.Request()
            param = rclpy.parameter.Parameter('max_vel_x', rclpy.Parameter.Type.DOUBLE, float(attr_value))
            request.parameters.append(param.to_parameter_msg())
            
            self.get_logger().info(f"Setting max_vel_x to {attr_value} m/s.")
            self.set_parameters_client.call_async(request)
            
        elif attr_type == "normal":
            request = SetParameters.Request()
            param = rclpy.parameter.Parameter('max_vel_x', rclpy.Parameter.Type.DOUBLE, self.original_speed)
            request.parameters.append(param.to_parameter_msg())
            self.get_logger().info(f"Setting max_vel_x to original speed {self.original_speed} m/s.")
            self.set_parameters_client.call_async(request)

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print("Usage: ros2 run waypoint_manager waypoint_manager [-w|-x|-e] <filename>.json [once]")
        sys.exit()

    mode = None
    filename = None
    is_once = False

    try:
        mode_arg_index = -1
        if '-w' in sys.argv:
            mode = 'write'
            mode_arg_index = sys.argv.index('-w')
        elif '-x' in sys.argv:
            mode = 'execute'
            mode_arg_index = sys.argv.index('-x')
        elif '-e' in sys.argv:
            mode = 'edit'
            mode_arg_index = sys.argv.index('-e')
        else:
            print("Usage: ros2 run waypoint_manager waypoint_manager [-w|-x|-e] <filename>.json [once]")
            sys.exit()

        filename = sys.argv[mode_arg_index + 1]
    except IndexError:
        print("Usage: ros2 run waypoint_manager waypoint_manager [-w|-x|-e] <filename>.json [once]")
        sys.exit()

    if 'once' in sys.argv:
        is_once = True

    if mode and filename:
        node = Nav2WaypointManager(mode, filename, not is_once)
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