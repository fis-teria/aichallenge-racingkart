# 01 実装仕様書：simple_delay_aware_control

## 目的

`simple_delay_aware_control` は、教育向けに小さく読める delay-aware 制御教材です。

本教材では、既存の `simple_pure_pursuit` と同じように trajectory と odometry を読み、次の2つを追加します。

- ステアリング遅延を考えた odometry の先読み
- trajectory の曲率から決める速度上限

実走時は `/control/command/control_cmd` を出せます。さらに `delay_aware_mpc_ros` と同じ考え方で、補償済み状態を `nav_msgs/msg/Odometry` として出します。

```text
/localization/kinematic_state
  -> simple_delay_aware_control_node
  -> /simple_delay_aware_control/localization/kinematic_state
```

## 対象者

- C++ の関数・構造体・配列処理を読める
- ROS 2 topic / launch / package の基本を学び始めている
- MPC の内部最適化まではまだ扱わない
- Pure Pursuit と自転車モデルを、式を追いながら理解したい

## 生成物

```text
simple_delay_aware_control/
  CMakeLists.txt
  package.xml
  README.md

  config/
    simple_delay_aware_control.param.yaml

  docs/
    01_implementation_spec.md
    02_logic_explanation.md
    03_launch_integration_guide.md

  include/simple_delay_aware_control/
    control_core.hpp
    simple_delay_aware_control_node.hpp
    exercise/control_core_exercise.hpp

  src/
    control_core.cpp
    control_core_exercise.cpp
    simple_delay_aware_control_node.cpp

  launch/
    simple_delay_aware_control.launch.xml

  test/
    test_control_core.cpp
```

## 編集してよいファイル

学習者が穴埋めするファイルは次です。

```text
src/control_core_exercise.cpp
```

`include/simple_delay_aware_control/exercise/control_core_exercise.hpp` は宣言だけです。関数の引数や戻り値の形を確認するために読んでよいですが、基本的には変更しません。

正解実装は次です。

```text
src/control_core.cpp
```

最初から正解を見ると学習効果が薄くなるので、詰まったときの確認用として扱います。

## 入出力topic

ノード本体は `simple_delay_aware_control_node` です。

| 種類 | topic | 型 | 役割 |
| --- | --- | --- | --- |
| input | `input/kinematics` | `nav_msgs/msg/Odometry` | 現在の車両状態 |
| input | `input/trajectory` | `autoware_auto_planning_msgs/msg/Trajectory` | 追従する経路 |
| input | `/vehicle/status/steering_status` | `autoware_auto_vehicle_msgs/msg/SteeringReport` | 実舵角 |
| input | `input/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | 目標ステア |
| output | `output/kinematics` | `nav_msgs/msg/Odometry` | 遅延補償後の車両状態 |
| output | `output/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | 制御指令 |
| output | `output/raw_control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | gain前の確認用指令 |
| output | `/simple_delay_aware_control/debug` | `std_msgs/msg/String` | JSON形式のデバッグ |

AI Challenge launch では次のように remap します。

```text
input/kinematics
  <- /localization/kinematic_state

input/trajectory
  <- /planning/scenario_planning/trajectory

input/control_cmd
  <- /control/command/control_cmd_raw

output/kinematics
  -> /simple_delay_aware_control/localization/kinematic_state

output/control_cmd
  -> /control/command/control_cmd
```

## パラメータ

主なパラメータは `config/simple_delay_aware_control.param.yaml` にあります。

### 遅延補償

| パラメータ | 代表値 | 意味 |
| --- | ---: | --- |
| `delay_enabled` | `true` | odometry の未来予測を有効にする |
| `steering_delay_sec` | `0.20` | 何秒先の姿勢を予測するか |
| `prediction_dt` | `0.02` | 予測積分の刻み幅 |
| `use_steering_lag` | `true` | ステア一次遅れを近似する |
| `steering_time_constant_sec` | `0.30` | ステア一次遅れの時定数 |
| `wheel_base` | `1.087` | 自転車モデルのホイールベース |
| `use_control_cmd_input` | `true` | 目標ステアを `input/control_cmd` から受け取る |
| `control_cmd_timeout_sec` | `0.50` | control_cmd を新鮮とみなす時間 |

### 曲率速度制御

| パラメータ | 代表値 | 意味 |
| --- | ---: | --- |
| `curvature_speed_enabled` | `true` | 曲率速度上限を使う |
| `max_speed_mps` | `8.333333333333334` | 最大速度 |
| `min_speed_mps` | `1.50` | 最低速度 |
| `lateral_accel_limit_mps2` | `3.0` | 横加速度上限 |
| `curvature_lookahead_points` | `8` | 曲率推定に使う先読み点数 |

### Odometry出力

| パラメータ | 代表値 | 意味 |
| --- | ---: | --- |
| `publish_delay_aware_odometry` | `true` | 補償済みodometryをpublishする |
| `write_target_speed_to_output_twist` | `false` | `true` のとき target speed を output odometry の twist に書く |

`write_target_speed_to_output_twist` は通常 `false` のままにします。下流ノードが `twist.twist.linear.x` を現在速度として読む場合、target speed に書き換えると意味が混ざるためです。

## 穴埋めTODO

`src/control_core_exercise.cpp` に次のTODOがあります。

| TODO | 関数 | 学ぶこと |
| --- | --- | --- |
| TODO-01 | `normalizeAngle` | 角度の正規化 |
| TODO-02 | `nearestTrajectoryIndex` | 最近傍点探索 |
| TODO-03 | `curvatureFromThreePoints` | 3点曲率 |
| TODO-04 | `estimateTrajectoryCurvature` | trajectory上の曲率推定 |
| TODO-05 | `speedLimitForCurvature` | 横加速度から速度上限を作る |
| TODO-06 | `predictDelay` | 自転車モデルで未来姿勢を予測する |
| TODO-07 | `planCurvatureSpeed` | 最近傍、曲率、速度上限を統合する |

## 合格条件

### コアロジック

```text
[ ] 直進状態で delay=0.20 sec, v=10 m/s のとき x が約 2.0 m 進む
[ ] 正のステア角で yaw が正方向へ変化する
[ ] 半径1mの円弧3点から曲率が約 1.0 になる
[ ] 曲率が大きいほど速度上限が下がる
[ ] trajectory速度と曲率速度上限の小さいほうが target speed になる
```

### ROS統合

```text
[ ] colcon build で simple_delay_aware_control が通る
[ ] /simple_delay_aware_control/localization/kinematic_state がpublishされる
[ ] /simple_delay_aware_control/debug に delay と curvature の情報が出る
[ ] control_method:=simple_delay_aware_control で起動できる
[ ] make dev で1周以上走行できる
```

## 推奨テスト

```bash
cd /aichallenge/workspace
colcon build --packages-select simple_delay_aware_control aichallenge_submit_launch
colcon test --packages-select simple_delay_aware_control --event-handlers console_direct+
```

Docker外のホスト環境では Autoware message package が見つからないことがあります。その場合は、AI Challenge の Docker 内でビルドします。

```bash
cd aichallenge-racingkart
CMD='source /opt/ros/humble/setup.bash && source /autoware/install/setup.bash && cd /aichallenge/workspace && colcon build --packages-select simple_delay_aware_control aichallenge_submit_launch' \
  docker compose run -T --rm --no-deps autoware-command
```

## 実装で扱わないこと

- MPC solver の行列や制約の変更
- CBF や最適化ベースの障害物回避
- V2Xを使った追い抜き判断
- trajectory 自体の生成やライン切替

この教材は、制御入力の前段で状態を少し未来へずらし、速度目標を曲率で抑えるところに集中します。
