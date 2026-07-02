# 02 ロジック説明書：遅延制御と曲率速度制御

## 全体像

`simple_delay_aware_control` は、次の順番で処理します。

```text
1. 現在odometryを読む
2. 現在ステア角を推定する
3. delay秒ぶん未来の姿勢を予測する
4. 予測姿勢をOdometryとしてpublishする
5. 予測姿勢からtrajectory最近傍点を探す
6. 先のtrajectory曲率を推定する
7. 曲率から速度上限を決める
8. Pure Pursuitでステア角を決める
9. control_cmdとdebugをpublishする
```

重要なのは、ステア計算や速度計算に使う状態を「本当の現在姿勢」ではなく「遅れを考えた未来姿勢」にすることです。

```text
本当の現在姿勢: x(t)
制御に使う姿勢: x(t + delay)
```

これは `delay_aware_mpc_ros` の考え方と同じです。MPC solver の中身を直接作り替えるのではなく、制御器へ渡す手前で odometry を未来側へずらします。

## 1. 現在ステア角を推定する

未来姿勢を予測するには、車両がいま曲がっているかを知る必要があります。

この教材では、現在ステアと目標ステアを分けて扱います。

現在ステアは、今すでに車両へ効いているステア角です。次の順で推定します。

```text
1. /vehicle/status/steering_status が新しければ実舵角を使う
2. 速度が十分あれば yaw_rate から逆算する
3. どちらも使えなければ最後に出した steering command を使う
```

目標ステアは、これからステア機構が追従していく指令値です。`delay_aware_mpc_ros` と同じように、`AckermannControlCommand` を購読して受け取ります。

```text
input/control_cmd
  <- /control/command/control_cmd_raw
```

`control_cmd_timeout_sec` より古い指令は使わず、最後に自分が出した steering command へ戻します。

yaw rate からステア角を逆算するときは、自転車モデルの関係を使います。

```text
yaw_rate = v / wheel_base * tan(steer)
steer = atan2(wheel_base * yaw_rate, v)
```

低速では `v` が小さくなり不安定なので、`min_velocity_for_yaw_prediction` 未満ではこの逆算を使いません。

## 2. 遅延ぶん未来姿勢を予測する

未来姿勢の予測には、軽量な kinematic bicycle model を使います。

```text
x_next   = x + v * cos(yaw) * dt
y_next   = y + v * sin(yaw) * dt
yaw_next = yaw + v / wheel_base * tan(steer) * dt
```

たとえば `v = 10 m/s`、`steering_delay_sec = 0.20 sec` の直進なら、車両は約2m進みます。

```text
10 m/s * 0.20 sec = 2.0 m
```

この `2.0 m` 先の姿勢を制御計算に使うことで、ステアが効き始めるころの車両位置に合わせて指令を出します。

## 3. ステア一次遅れ

実際のステアは、目標角へ瞬間移動しません。

そこで `use_steering_lag = true` のときは、次のような一次遅れでステア角を目標へ近づけます。

```text
alpha = 1 - exp(-dt / steering_time_constant_sec)
steer_next = steer_now + (steer_target - steer_now) * alpha
```

`steering_time_constant_sec` が大きいほど、ステアの変化はゆっくりになります。

## 4. 補償済みOdometryをpublishする

予測した姿勢は、元の `nav_msgs/msg/Odometry` をコピーし、poseだけを書き換えてpublishします。

```text
input:
  /localization/kinematic_state

output:
  /simple_delay_aware_control/localization/kinematic_state
```

twist はデフォルトでは書き換えません。

理由は、`twist.twist.linear.x` は通常「現在速度」を意味するからです。曲率速度制御で得た target speed は「目標速度」であり、現在速度とは意味が違います。

必要な教材実験では、次を `true` にすると target speed を output odometry の `twist.twist.linear.x` へ書けます。

```yaml
write_target_speed_to_output_twist: true
```

ただし実走時に下流ノードへ渡す場合は、意味の混同に注意します。

## 5. trajectory最近傍点を探す

曲率速度制御とPure Pursuitのために、現在の制御姿勢に一番近い trajectory point を探します。

処理は単純です。

```text
best_index = 0
best_distance = very_large

for each trajectory point:
  d2 = (point.x - x)^2 + (point.y - y)^2
  if d2 < best_distance:
    best_index = index
```

平方根は不要です。距離の大小だけ比べるなら、二乗距離で十分です。

## 6. 3点から曲率を推定する

曲率は、簡単に言うと「どれくらい急に曲がっているか」です。

この教材では、trajectory上の3点を使って曲率を近似します。

```text
p0 = nearest point
p1 = middle lookahead point
p2 = lookahead point
```

3点で作る三角形の外積と辺の長さから、符号付き曲率を求めます。

```text
cross = (p1 - p0) x (p2 - p0)
a = |p0 - p1|
b = |p1 - p2|
c = |p2 - p0|
kappa = 2 * cross / (a * b * c)
```

符号は左曲がり・右曲がりを表します。速度制御では急カーブかどうかが重要なので、主に `abs(kappa)` を使います。

## 7. 曲率から速度上限を作る

横加速度は、概念的に次で近似できます。

```text
a_lat = v^2 * |kappa|
```

横加速度の上限を `a_lat_max` とすると、速度上限は次です。

```text
v_limit = sqrt(a_lat_max / |kappa|)
```

曲率が小さい直線では、速度上限は `max_speed_mps` になります。曲率が大きいカーブでは、速度上限が下がります。

最終的な target speed は、trajectory に入っている速度と曲率速度上限の小さいほうです。

```text
target_speed = min(trajectory_speed, curvature_speed_limit)
```

## 8. Pure Pursuitでステア角を決める

ステア角は、予測済み姿勢から lookahead point を見て決めます。

```text
lookahead_distance = max(
  lookahead_min_distance,
  lookahead_gain * max(target_speed, current_speed)
)
```

車両後輪中心から lookahead point への角度差を `alpha` とすると、Pure Pursuitのステア角は次です。

```text
steer = atan2(2 * wheel_base * sin(alpha), lookahead_distance)
```

最後に `steering_tire_angle_gain` をかけ、`max_steering_tire_angle_rad` で制限して publish します。

## debugで見るもの

`/simple_delay_aware_control/debug` は JSON 文字列です。

代表的な項目は次です。

| 項目 | 意味 |
| --- | --- |
| `delay_shifted` | delay補償が有効か |
| `prediction_steps` | 予測積分のステップ数 |
| `current_steering_rad` | 推定した現在ステア角 |
| `target_steering_rad` | control_cmdから受け取った目標ステア角 |
| `applied_steering_rad` | 予測中に使った最後のステア角 |
| `nearest_index` | trajectory最近傍index |
| `curvature` | 推定曲率 |
| `curvature_speed_limit_mps` | 曲率から決めた速度上限 |
| `trajectory_speed_mps` | trajectory点の速度 |
| `target_speed_mps` | 最終目標速度 |
| `input_pose` | 元のodometry姿勢 |
| `control_pose` | delay補償後の制御用姿勢 |

確認コマンド例です。

```bash
ros2 topic echo /simple_delay_aware_control/debug
ros2 topic echo /simple_delay_aware_control/localization/kinematic_state
```

## よくある失敗

### カーブで減速しない

確認するもの:

```text
curvature_speed_enabled が true か
curvature_lookahead_points が小さすぎないか
trajectory が直線に近すぎないか
lateral_accel_limit_mps2 が大きすぎないか
```

### 走行ラインに対して切り遅れる

確認するもの:

```text
steering_delay_sec が小さすぎないか
use_steering_lag が false になっていないか
steering_status が古くなっていないか
control_cmd_raw が入力されているか
debug の control_pose が input_pose より前へ進んでいるか
```

### Odometryを下流へ渡すと挙動が変

確認するもの:

```text
write_target_speed_to_output_twist が true になっていないか
下流ノードが twist を現在速度として読んでいないか
delay補償済みOdometryと通常Odometryを二重に補償していないか
```

## この教材の設計意図

この教材は、本格的な solver-level delay-augmented MPC ではありません。

狙いは、次の3点を小さいC++コードで学べるようにすることです。

```text
1. 現在状態を未来へずらすと、遅延を見込んだ制御になる
2. 曲率から速度上限を作ると、カーブで自然に減速できる
3. ROS topicの前段ノードに分けると、既存制御器を壊さず比較できる
```
