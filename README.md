# minicar_simulation

ROS2用のミニカーシミュレーションパッケージです。Gazeboシミュレーション環境で差動駆動ロボットの動作をテストできます。

## 概要

このパッケージは以下の機能を提供します：

- Gazeboでのミニカーシミュレーション
- 差動駆動制御（diff_drive_controller）
- ランダムなコースの動的生成（seed指定で再現可能）
- コース上の任意の位置からのスタート
- ヘッドレスモード（GUI無し）でのシミュレーション

## 使用方法

### 基本的な起動

```bash
ros2 launch minicar_simulation road_env_minicar.launch.py
```

### ランダムコースで起動（毎回異なるコース）

```bash
ros2 launch minicar_simulation road_env_minicar.launch.py
```

### 再現可能なコースで起動（seed指定）

```bash
ros2 launch minicar_simulation road_env_minicar.launch.py seed:=42
```

### ヘッドレスモード（GUIなし）

```bash
ros2 launch minicar_simulation road_env_minicar.launch.py gui:=false
```

### launchファイル

#### `road_env_minicar.launch.py`

Gazeboシミュレーション環境でミニカーを起動します。

**使用可能なオプション：**

| パラメータ | デフォルト値 | 説明 |
|-----------|-------------|------|
| `seed` | (空) | コース生成のシード値。空の場合はランダム |
| `start_position` | `0.0` | コース上の開始位置（0.0〜1.0の比率） |
| `generate_course` | `true` | コースを動的に生成するか |
| `gui` | `true` | GazeboのGUIを表示するか |
| `world` | `road_env.world` | 使用するGazeboワールドファイル |
| `entity` | `minicar` | ロボットのエンティティ名 |
| `x` | `0.0` | 初期X座標 (m) ※generate_course=falseの場合のみ |
| `y` | `1.5` | 初期Y座標 (m) ※generate_course=falseの場合のみ |
| `z` | `0.05` | 初期Z座標 (m) ※generate_course=falseの場合のみ |
| `yaw` | `0.0` | 初期ヨー角 (rad) ※generate_course=falseの場合のみ |
| `use_sim_time` | `true` | シミュレーション時間の使用 |
| `robot_ns` | `sim_robot` | ロボットの名前空間 |

**使用例：**

```bash
# seed=42で再現可能なコース、GUIなし
ros2 launch minicar_simulation road_env_minicar.launch.py seed:=42 gui:=false

# コースの中間地点（50%）からスタート
ros2 launch minicar_simulation road_env_minicar.launch.py seed:=42 start_position:=0.5

# コース生成なし（既存コースを使用）
ros2 launch minicar_simulation road_env_minicar.launch.py generate_course:=false x:=2.0 y:=0.0
```

## コース生成スクリプト

`scripts/generate_course.py` でコースを手動生成できます。

```bash
# ランダムなコースを生成
python3 scripts/generate_course.py

# seed指定で再現可能なコースを生成
python3 scripts/generate_course.py --seed 42

# 出力先を指定
python3 scripts/generate_course.py --output-dir /tmp/my_course --models-dir /tmp/my_models
```

**生成されるファイル：**

| ファイル | 説明 |
|---------|------|
| `circuit.json` | コースのウェイポイント・曲線データ |
| `circuit.png` | コースのプレビュー画像 |
| `spawn_pose.json` | ロボットの初期位置・姿勢 |
| `models/road_env/` | Gazebo用の壁モデル（STL + SDF） |

## ロボット制御

シミュレーション起動後、以下のトピックでロボットを制御できます：

```bash
# 速度指令の送信
ros2 topic pub /sim_robot/diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}'

# オドメトリ情報の確認
ros2 topic echo /sim_robot/diff_drive_controller/odom
```

## 依存関係

- ROS2 Humble以上
- Gazebo
- ros2_control
- diff_drive_controller
- robot_state_publisher
- gazebo_ros
- Python: numpy, opencv-python, scipy

## トラブルシューティング

### コントローラーが起動しない場合

コントローラーの起動は遅延設定されています（6秒後）。Gazeboとロボットの完全な起動を待ってから実行されます。

### ロボットが表示されない場合

GAZEBO_MODEL_PATHが正しく設定されているか確認してください。このパッケージは自動的にモデルパスを設定します。

### ヘッドレスモードで動作確認

```bash
# トピック一覧を確認
ros2 topic list

# オドメトリが出力されていることを確認
ros2 topic echo /sim_robot/diff_drive_controller/odom --once
```
