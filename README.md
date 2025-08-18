# waypoint_manager

## 概要
このプログラムは、ROS 2ノードとして動作し、主に以下の3つのモードでウェイポイントを扱います。
write モード: ロボットの現在位置 (/estimated_pose) やジョイスティックの入力、Rviz2の2D Goal Poseツール (/goal_pose) を使ってウェイポイントを記録し、JSONファイルに保存します。
read モード: 既存のJSONファイルからウェイポイントを読み込み、Rviz2で可視化するためにPublishします。
edit モード: GUI (tkinter) を使ってウェイポイントを視覚的に管理・編集します。Rviz2の2D Goal Poseツールを使って、ウェイポイントの追加、削除、置換ができます。
保存されるウェイポイントはJSON形式で、各ウェイポイントは 
``` [[x, y, z], [qx, qy, qz, qw]] ```
の形式で格納されます。ただし、このコードでは位置の z と姿勢の x, y は0.0に固定されており、2Dナビゲーションに特化しています。

## クラスの解説
### Nav2WaypointManagerGUI(tk.Toplevel)
このクラスは、edit モードでウェイポイントを管理するためのTkinterベースのGUIウィンドウを構築します。

```
__init__(self, parent, waypoint_maker_node):
```

親ウィンドウと Nav2WaypointMaker ノードのインスタンスを受け取ります。
ウェイポイントを表示するための Listbox と Scrollbar を設定します。
ウェイポイントの追加、削除、置換、保存のためのボタンを配置します。
初期状態でウェイポイントリストを更新します。
```
update_listbox(self):
```
waypoint_list の現在のウェイポイントをクリアし、最新の情報を表示するように Listbox を更新します。
```
add_waypoint(self):
```
新しいウェイポイントを追加するための処理を開始します。

Rviz2の2D Goal Poseツールを使用して新しいウェイポイントを設定するようにユーザーに促します。

waypoint_maker_node.is_adding と waypoint_maker_node.insert_index を設定し、goal_callback がウェイポイントを正しく追加できるようにします。
```
remove_waypoint(self):
```
Listbox で選択されたウェイポイントをリストから削除します。
変更をJSONファイルに保存し、Rviz2での可視化を更新します。
```
replace_waypoint(self):
```
Listbox で選択されたウェイポイントをRviz2の2D Goal Poseツールで設定される新しいポーズに置き換えるための準備をします。

waypoint_maker_node.replace_index を設定し、goal_callback がウェイポイントを正しく置換できるようにします。
```
save_waypoints(self):
```
現在のウェイポイントリストをJSONファイルに保存します。

### Nav2WaypointManager(Node)
このクラスは、ROS 2ノードの主要なロジックを実装しています。
```
__init__(self, mode, filename):
```
ROS 2ノードとして初期化されます（例: nav2_waypoint_maker_write）。

ノードのモード (write, read, edit) とウェイポイントファイルのパスを保存します。

ROSパラメータを宣言し、デフォルト値と説明を設定します。

- map_waypoint_list_csv: ウェイポイントリストを記述するCSVファイル名 (現状は使われていない)

- map_base_path: 地図ファイルが置かれている基本パス (現状は使われていない模様)

- waypoint_base_path: ウェイポイントファイルが置かれている基本パス (現状は使われていない)

- waypoint_tolerance_position: ウェイポイント間の自動追加距離閾値

- waypoint_tolerance_orientation: ウェイポイント間の自動追加距離閾値 (方向は現状使われていない模様)

- waypoint_button: ジョイスティックのウェイポイント追加ボタンのインデックス

- auto_waypoint_distance: 自動ウェイポイント追加のための距離閾値

- estimated_pose_timeout: /estimated_pose トピックのタイムアウト時間

### パブリッシャー (Publisher) の設定:

- waypoints (geometry_msgs.msg.PoseArray): Rviz2でウェイポイントを表示するために使用。
- waypoint_markers (visualization_msgs.msg.Marker): Rviz2でウェイポイントの番号を表示するために使用。
- サブスクライバー (Subscriber) の設定:
- /goal_pose (geometry_msgs.msg.PoseStamped): Rviz2の2D Goal Poseツールからの入力。
- /estimated_pose (geometry_msgs.msg.PoseStamped): ロボットの推定位置（write モードのみ）。
- /initialpose (geometry_msgs.msg.PoseWithCovarianceStamped): Rviz2の2D Pose Estimateツールからの初期位置（write モードのみ）。
- /joy (sensor_msgs.msg.Joy): ジョイスティックの入力（write モードのみ）。
- /remove_waypoint (std_msgs.msg.Int16): 指定されたインデックスのウェイポイントを削除（write モードのみ）。
- /insert_waypoint (std_msgs.msg.Int16): 指定されたインデックスにウェイポイントを挿入（write モードのみ）。
- タイマーを設定し、/estimated_pose トピックのタイムアウトを監視します。
- 指定されたモードに応じて、ウェイポイントのロード、可視化、GUIの起動などを行います。

### load_waypoints_from_json(self):

指定されたJSONファイルからウェイポイントを読み込み、waypoints リストに PoseStamped メッセージの形式で格納します。

ファイルが見つからない場合は新しいファイルを作成し、JSONのデコードエラーやフォーマットエラーをログに出力します。

### save_waypoints_to_json(self):

現在の waypoints リストの内容を、指定されたJSONファイルに保存します。
JSON形式は
``` [[position.x, position.y, 0.0], [0.0, 0.0, orientation.z, orientation.w]] ``` です。

### publish_waypoints_for_vis(self):
Rviz2でウェイポイントの集合を表示するために、waypoints リストを PoseArray メッセージとして /waypoints トピックにPublishします。

### rewrite_marker(self):
Rviz2で各ウェイポイントに番号付きのマーカー（Marker.TEXT_VIEW_FACING タイプ）を表示するために、/waypoint_markers トピックにPublishします。
既存のマーカーを一度削除し、新しいマーカーを再描画します。

### goal_callback(self, msg):

Rviz2の2D Goal Poseツールから /goal_pose トピックに新しい目標姿勢がPublishされたときに呼び出されます。

edit モードの場合、選択したウェイポイントの置換、または指定位置への追加を行います。
write モードの場合、新しいウェイポイントをリストの末尾に追加します。
insert_waypoint_at(self, index, pose_stamped):
指定されたインデックスに新しいウェイポイントを挿入します。

### lio_loc_callback(self, msg):

/estimated_pose トピックからロボットの現在位置を受信します（write モードのみ）。
設定された距離閾値 (distance_threshold) に基づいて、自動的にウェイポイントを追加します。

### lio_loc_waypoint_append(self):

ロボットの現在の推定位置をウェイポイントリストに追加します。

### joy_callback(self, msg):
ジョイスティックからの入力 /joy を受信します（write モードのみ）。

設定されたボタン (joy_button) が押された場合、現在のロボット位置をウェイポイントとして追加します。

### init_pose_callback(self, msg):

Rviz2の2D Pose Estimateツールから /initialpose トピックに初期姿勢がPublishされたときに呼び出されます（write モードのみ）。

受信した初期姿勢を別のJSONファイル (initial_<filename>.json) に保存します。

### remove_callback(self, msg):

/remove_waypoint トピックに整数メッセージを受信したときに呼び出されます（write モードのみ）。

メッセージのデータとして指定されたインデックスのウェイポイントを削除します。

### insert_callback(self, msg):

/insert_waypoint トピックに整数メッセージを受信したときに呼び出されます（write モードのみ）。

メッセージのデータとして指定されたインデックスに、現在のロボット位置をウェイポイントとして挿入します。

### check_timeout(self):

タイマーによって定期的に呼び出されます。

/estimated_pose トピックからの最後のメッセージが設定されたタイムアウト時間 (topic_timeout) を超えている場合に警告をログに出力します。

main 関数の解説
ros_spin(node):

Tkinter GUIとROS 2ノードを同時に実行するために、ROS 2ノードを別のスレッドで rclpy.spin() させるためのヘルパー関数です。

main(args=None):

ROS 2のクライアントライブラリを初期化します (rclpy.init)。

コマンドライン引数を解析して、実行モード (-w, -r, -e) とウェイポイントファイル名を取得します。

### Nav2WaypointMaker ノードのインスタンスを作成します。

edit モードの場合、ros_spin 関数を新しいスレッドで実行し、メインスレッドではTkinterのGUI (tk.mainloop()) を開始します。これにより、ROS 2の処理とGUIが同時に動作できます。

その他のモードでは、メインスレッドで直接 rclpy.spin(node) を実行します。

ノードを破棄し、ROS 2のクライアントライブラリをシャットダウンします。

実行方法
このスクリプトは、以下のコマンドライン引数で実行します。

Bash
```
ros2 run waypoint_manager waypoint_manager [-w|-r|-e] filename.json
```

-w: write モード (ウェイポイント記録)
-r: read モード (ウェイポイント読み込みと可視化)
-e: edit モード (GUIによるウェイポイント編集)
filename.json: ウェイポイントを保存/読み込みするJSONファイルの名前

例:
ウェイポイントを記録する場合:
Bash
```
ros2 run your_package_name waypoint_maker -w my_waypoints.json
(Rviz2で2D Goal Poseツールを使用するか、ジョイスティックのボタンを押すか、ロボットを動かして自動で追加します)
```
ウェイポイントを読み込んで可視化する場合:

Bash
```
ros2 run your_package_name waypoint_maker -r my_waypoints.json
(Rviz2で /waypoints と /waypoint_markers トピックを購読して確認します)
```
GUIでウェイポイントを編集する場合:

Bash
```
ros2 run your_package_name waypoint_maker -e my_waypoints.json
(ウェイポイントエディタのGUIウィンドウが表示されます。必要に応じてRviz2の2D Goal Poseツールを使用します)
```