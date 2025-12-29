# minicar_simulation

ROS2用のミニカーシミュレーションパッケージです。Gazeboシミュレーション環境で差動駆動ロボットの動作をテストできます。

## 概要

このパッケージは以下の機能を提供します：

- Gazeboでのミニカーシミュレーション
- 差動駆動制御（diff_drive_controller）
- カスタマイズ可能な道路環境
- ロボットの姿勢・位置の初期化

## 使用方法

### 基本的な起動

```bash
ros2 launch minicar_simulation road_env_minicar.launch.py
```

### launchファイル

#### `road_env_minicar.launch.py`

Gazeboシミュレーション環境でミニカーを起動します。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `world` | `road_env.world` | 使用するGazeboワールドファイル |
| `entity` | `minicar` | ロボットのエンティティ名 |
| `x` | `0.0` | 初期X座標 (m) |
| `y` | `1.5` | 初期Y座標 (m) |
| `z` | `0.05` | 初期Z座標 (m) |
| `yaw` | `0.0` | 初期ヨー角 (rad) |
| `use_sim_time` | `true` | シミュレーション時間の使用 |
| `robot_ns` | `sim_robot` | ロボットの名前空間 |
| `joint_state_broadcaster` | `joint_state_broadcaster` | ジョイント状態ブロードキャスター名 |
| `diff_controller` | `diff_drive_controller` | 差動駆動コントローラー名 |

**使用例：**

```bash
# 初期位置を変更
ros2 launch minicar_simulation road_env_minicar.launch.py x:=2.0 y:=0.0 yaw:=1.57

# 名前空間を変更
ros2 launch minicar_simulation road_env_minicar.launch.py robot_ns:=my_robot

# カスタムワールドファイルを使用
ros2 launch minicar_simulation road_env_minicar.launch.py world:=/path/to/custom.world
```

#### `robot_state_publisher.launch.py`

ロボットの状態パブリッシャーのみを起動します。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `robot_ns` | `sim_robot` | ロボットの名前空間 |
| `use_sim_time` | `true` | シミュレーション時間の使用 |

## ロボット制御

シミュレーション起動後、以下のトピックでロボットを制御できます：

```bash
# 速度指令の送信
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'

# オドメトリ情報の確認
ros2 topic echo /sim_robot/odom
```

## 依存関係

- ROS2 Humble以上
- Gazebo
- ros2_control
- diff_drive_controller
- robot_state_publisher
- gazebo_ros

## トラブルシューティング

### コントローラーが起動しない場合

コントローラーの起動は遅延設定されています（6秒後）。Gazeboとロボットの完全な起動を待ってから実行されます。

### ロボットが表示されない場合

GAZEBO_MODEL_PATHが正しく設定されているか確認してください。このパッケージは自動的にモデルパスを設定しますが、カスタムモデルを使用する場合は手動設定が必要です。