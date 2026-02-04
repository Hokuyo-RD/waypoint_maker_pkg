# waypoint_manager

## 概要
このパッケージは、Nav2を使用してウェイポイント追従を行うためのROS 2ノード `waypoint_manager` を提供します。
JSONファイルからウェイポイントを読み込み、Nav2の `navigate_to_pose` アクションを使用してロボットを順番に移動させます。
各ウェイポイントには属性（停止、減速など）や許容誤差を設定でき、柔軟なナビゲーションが可能です。

---

## 目次
- [waypoint\_manager](#waypoint_manager)
  - [概要](#概要)
  - [目次](#目次)
  - [インストール](#インストール)
  - [ウェイポイントファイル (JSON) の形式](#ウェイポイントファイル-json-の形式)
  - [ノード: waypoint\_manager](#ノード-waypoint_manager)
    - [クラス: Nav2WaypointManager](#クラス-nav2waypointmanager)
      - [パラメータ](#パラメータ)
      - [トピック (Publishers)](#トピック-publishers)
      - [トピック (Subscribers)](#トピック-subscribers)
      - [アクションクライアント](#アクションクライアント)
    - [主な機能](#主な機能)
  - [実行方法](#実行方法)
    - [実行例](#実行例)

---

## インストール

```bash
cd <YOUR-ROS2-WORKSPACE>/src
git clone https://github.com/hokuyo-rd/waypoint_manager.git
cd ../
colcon build --packages-select waypoint_manager
```


## ウェイポイントファイル (JSON) の形式
ウェイポイントはJSON形式で保存されます。各ウェイポイントは以下のリスト構造を持ちます。

```json
[
  [position_x, position_y, position_z],
  [orientation_x, orientation_y, orientation_z, orientation_w],
  {
    "type": "normal",      // 属性タイプ ("normal", "stop", "slow")
    "value": 0,            // 属性の値 (停止時間[s] や 制限速度[m/s])
    "xy_tolerance": 0.25,  // 到着判定の距離許容誤差 [m]
    "yaw_tolerance": 0.25  // 到着判定の角度許容誤差 [rad]
  }
]
```

## ノード: waypoint_manager

### クラス: Nav2WaypointManager

Nav2のアクションクライアントとして動作し、ウェイポイントの管理と実行を行います。

#### パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
| --- | --- | --- | --- |
| `use_gnss_switch` | bool | `False` | GNSS/LIOの切り替え安定待ち機能を使用するかどうか |
| `yaw_goal_tolerance` | double | `0.25` | デフォルトの角度許容誤差 [rad] |
| `xy_goal_tolerance` | double | `0.25` | デフォルトの距離許容誤差 [m] |
| `cmd_vel_topic` | string | `"/cmd_vel"` | 初期化動作時に使用する速度トピック名 |
| `initialize_cmd_vel_linear_x` | double | `0.1` | 初期化動作時の並進速度 [m/s] |
| `initialize_radius` | double | `3.0` | 初期化動作時の旋回半径 (現在は直進のみで使用) |
| `original_speed` | double | `0.56` | `normal` 属性時に復帰する速度 [m/s] |

#### トピック (Publishers)

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `waypoints` | `geometry_msgs/PoseArray` | 読み込んだウェイポイントの可視化用。 |
| `waypoint_marker_array` | `visualization_msgs/MarkerArray` | ウェイポイントの詳細情報（矢印、属性テキスト）の可視化用。 |
| `current_goal_marker` | `visualization_msgs/Marker` | 現在目指しているウェイポイントの可視化用。 |
| `cmd_vel_topic` | `geometry_msgs/Twist` | `use_gnss_switch` 有効時の初期化動作指令。 |
| `/wizurg/stop_cmd_vel` | `std_msgs/Empty` | 停止指令（属性 `stop` 時など）。 |
| `/wizurg/start_cmd_vel` | `std_msgs/Empty` | 再開指令（属性 `normal` 時など）。 |
| `/wizurg/slow_cmd_vel` | `std_msgs/Float32` | 減速指令（属性 `slow` 時）。 |

#### トピック (Subscribers)

| トピック名 | 型 | 説明 |
| --- | --- | --- |
| `/rsf/rsf_odom_type` | `std_msgs/String` | オドメトリのスイッチタイプ監視用 (`use_gnss_switch` が True の場合)。 |

#### アクションクライアント

- `navigate_to_pose` (`nav2_msgs/NavigateToPose`): Nav2への移動目標送信。

### 主な機能

1.  **ウェイポイントの読み込み**: 指定されたJSONファイルからウェイポイントと属性を読み込みます。
2.  **初期化動作 (`wait_for_stable_odometry_switch_type`)**: `use_gnss_switch` が有効な場合、オドメトリタイプが "LIO raw" 以外になるまで待機し、その間微速前進指令を出します。
3.  **ナビゲーション実行 (`main_loop`)**:
    - 順番にNav2へゴールを送信します。
    - 独自の到着判定ロジックを持ち、Nav2の判定とは別に、設定された `xy_tolerance` と `yaw_tolerance` に基づいて到着を確認します。
    - ウェイポイント通過判定（スキップ機能）を実装しており、経路上でウェイポイントを通り過ぎた場合、次のウェイポイントへ自動的に切り替えます。
4.  **属性処理 (`process_waypoint_attribute`)**:
    - `stop`: 指定時間停止します。
    - `slow`: 指定速度に減速します（`/wizurg/slow_cmd_vel` をPublish）。
    - `normal`: 通常速度に戻ります。

## 実行方法

```bash
ros2 run waypoint_manager waypoint_manager <filename.json> [--once]
```

- `<filename.json>`: ウェイポイントファイルのパス。
- `--once`: オプション。指定するとループ実行せず、最後のウェイポイントで終了します。指定しない場合はループします。

### 実行例

```bash
# ループ実行
ros2 run waypoint_manager waypoint_manager /home/user/waypoints/path.json

# 1回のみ実行
ros2 run waypoint_manager waypoint_manager /home/user/waypoints/path.json --once
```