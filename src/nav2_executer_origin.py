#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import json
import os
import sys

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints

class Nav2Executer(Node):
    def __init__(self):
        super().__init__('nav2_executer')

        # QoSプロファイルの設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ウェイポイントパブリッシャー（可視化用）
        self.waypoint_pub = self.create_publisher(PoseStamped, 'waypoints', qos_profile)

        # 初期位置パブリッシャー
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos_profile)

        # Nav2のFollowWaypointsアクションクライアント
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.waypoints = []
        self.change_params = []

    def load_waypoints(self, file_name):
        """JSONファイルからウェイポイントを読み込む"""
        try:
            with open(file_name, 'r') as f:
                waypoint_data = json.load(f)
                for point in waypoint_data:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = point[0][0]
                    pose.pose.position.y = point[0][1]
                    pose.pose.position.z = point[0][2]
                    pose.pose.orientation.x = point[1][0]
                    pose.pose.orientation.y = point[1][1]
                    pose.pose.orientation.z = point[1][2]
                    pose.pose.orientation.w = point[1][3]
                    self.waypoints.append(pose)
                    if len(point) >= 3:
                        self.change_params.append(point[2])
                    else:
                        self.change_params.append(None)
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {file_name}")
        except FileNotFoundError:
            self.get_logger().error(f"Waypoint file not found: {file_name}")
            return False
        return True

    def load_initial_pose(self, file_name):
        """初期位置をJSONファイルから読み込みパブリッシュする"""
        try:
            with open(file_name, 'r') as f:
                init_pose_dict = json.load(f)
                initial_pose_msg = PoseWithCovarianceStamped()
                initial_pose_msg.header.frame_id = init_pose_dict["frame_id"]
                initial_pose_msg.pose.pose.position.x = init_pose_dict["Pos_x"]
                initial_pose_msg.pose.pose.position.y = init_pose_dict["Pos_y"]
                initial_pose_msg.pose.pose.position.z = init_pose_dict["Pos_z"]
                initial_pose_msg.pose.pose.orientation.x = init_pose_dict["Ori_x"]
                initial_pose_msg.pose.pose.orientation.y = init_pose_dict["Ori_y"]
                initial_pose_msg.pose.pose.orientation.z = init_pose_dict["Ori_z"]
                initial_pose_msg.pose.pose.orientation.w = init_pose_dict["Ori_w"]
                initial_pose_msg.pose.covariance = [float(c) for c in init_pose_dict["cov"]]
                
                # 初期位置を複数回パブリッシュ
                self.get_logger().info("Publishing initial pose...")
                for _ in range(10):
                    self.initial_pose_pub.publish(initial_pose_msg)
                    self.get_logger().info(f"Published initial pose from {file_name}")
                    # 小さな遅延を入れてAMCLが確実に受け取るようにする
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        except FileNotFoundError:
            self.get_logger().warning(f"Initial pose file not found: {file_name}")
            
    def send_waypoints_goal(self):
        """ウェイポイントをNav2アクションサーバーに送信する"""
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
        """ゴールレスポンスを処理する"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by action server')
            return

        self.get_logger().info('Goal accepted! Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """アクションの結果を処理する"""
        result = future.result().result
        status = future.result().status
        
        if status == 2: # succeeded
            self.get_logger().info('Goal succeeded! All waypoints reached.')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')

        # ループ実行の場合は再度ウェイポイントを送信
        if 'once' not in sys.argv:
            self.get_logger().info('Looping back to the beginning...')
            self.send_waypoints_goal()


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> nav2_executer.py <waypoint_file_name.json> [once]")
        rclpy.shutdown()
        return

    node = Nav2Executer()
    
    # 初期位置の読み込みとパブリッシュ
    init_pose_file = "initial_" + sys.argv[1]
    if os.path.isfile(init_pose_file):
        node.load_initial_pose(init_pose_file)

    # ウェイポイントの読み込み
    if node.load_waypoints(sys.argv[1]):
        # ウェイポイントをアクションサーバーに送信
        node.send_waypoints_goal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()