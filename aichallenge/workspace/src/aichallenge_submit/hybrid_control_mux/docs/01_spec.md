# Hybrid Control Mux 仕様

## 目的

`hybrid_control_mux` は、通常時は delay aware MPC の制御指令を使い、MPC が infeasible になったり制御指令が途切れたりした場合に、低速の Pure Pursuit へ一時的に切り替えるための制御コマンド切替ノードです。

オンライン提出環境や負荷が高い環境では、MPC が重くなって解けない、または制御周期に間に合わない可能性があります。このノードはそのときに車両を完全に無制御へ落とさず、最低限コース追従を続けるためのフォールバックとして動作します。

## 対象範囲

このノードが行うことは、制御コマンドの選択とフォールバック時の速度制限です。

- MPC が正常なときは MPC の `AckermannControlCommand` をそのまま最終出力へ流す
- MPC が連続して infeasible になったときは Pure Pursuit の制御指令へ切り替える
- MPC の制御指令が途切れたときも、設定により Pure Pursuit へ切り替える
- hybrid 起動時の Pure Pursuit は `/overtake/reference_override` を読み、追い越し・追従用の横オフセットと速度 cap を反映した trajectory を追う
- Pure Pursuit の指令も使えないときは停止指令を出す
- 切り替え状態を `/hybrid_control_mux/debug` に JSON 形式で出す

一方で、次の処理は行いません。

- MPC の最適化問題そのものを軽くする、または解きやすくする
- Pure Pursuit 内で独自に障害物回避や追い越し判断を行う
- 相手車両の未来位置を予測する
- 走行経路や速度プロファイルを生成する
- 複数の最終制御コマンドを同時に `/control/command/control_cmd` へ出す

## 起動方法

`aichallenge_submit_launch/launch/reference.launch.xml` の `control_method` に `hybrid_delay_aware_mpc` を指定すると起動します。

```bash
control_method:=hybrid_delay_aware_mpc
```

この制御方式では、以下の3つを同時に起動します。

- delay aware MPC
- Pure Pursuit
- hybrid control mux

MPC と Pure Pursuit は最終制御トピックへ直接出力せず、いったん hybrid control mux 用の内部トピックへ出力します。最終的に `/control/command/control_cmd` を publish するのは `hybrid_control_mux` だけです。

## トピック構成

### 入力

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/hybrid_control/mpc/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | delay aware MPC の制御指令 |
| `/hybrid_control/pure_pursuit/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | Pure Pursuit の制御指令 |
| `/mpc/speed_profile_debug` | `std_msgs/msg/String` | MPC の状態監視用 JSON |
| `/overtake/reference_override` | `std_msgs/msg/Float32MultiArray` | MPC と Pure Pursuit が読む追い越し・追従用の横オフセットと速度 cap |

ノード内部では、それぞれ次の名前へ remap されます。

- `input/mpc_control_cmd`
- `input/pure_pursuit_control_cmd`
- `input/mpc_health`

### 出力

| トピック | 型 | 内容 |
| --- | --- | --- |
| `/control/command/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | 車両へ渡す最終制御指令 |
| `/hybrid_control_mux/debug` | `std_msgs/msg/String` | 切り替え状態の JSON |

ノード内部では、それぞれ次の名前から remap されます。

- `output/control_cmd`
- `output/debug`

## MPC health の入力仕様

`/mpc/speed_profile_debug` は JSON 文字列として扱います。`hybrid_control_mux` が参照するフィールドは以下です。

| フィールド | 型 | 内容 |
| --- | --- | --- |
| `mpc_status` | string | MPC の状態。復帰判定では `solved` を正常扱いする |
| `mpc_infeasible_count` | int | MPC が連続して infeasible になった回数 |

JSON が壊れている場合、health は `parse_error` として無効扱いになります。一定時間 health が届かない場合は `stale` として無効扱いになります。

## 基本動作

### 通常時

MPC の制御指令が新しく、MPC health が `solved` で、`mpc_infeasible_count` が 0 の場合は、MPC を正常とみなします。この状態では MPC の制御指令をそのまま `/control/command/control_cmd` へ出力します。

### フォールバック開始

以下の条件で Pure Pursuit へのフォールバックを開始します。

- `mpc_infeasible_count` が `fallback_trigger_infeasible_count` 以上になった
- MPC の制御指令が `mpc_cmd_timeout_sec` より長く届かず、`use_pure_pursuit_on_mpc_cmd_timeout` が true
- MPC health がタイムアウトし、`use_pure_pursuit_on_mpc_health_timeout` が true

デフォルトでは health のタイムアウトだけではフォールバックしません。これは、debug トピックの一時的な欠落だけで Pure Pursuit に落ちる誤判定を避けるためです。

### フォールバック中

Pure Pursuit の制御指令が新しければ、その指令を最終出力に使います。ただし、速度と加速度はフォールバック用の範囲に制限します。

- speed は `0.0` から `fallback_speed_mps` の範囲に制限
- acceleration は `fallback_decel_min_mps2` から `fallback_accel_max_mps2` の範囲に制限

hybrid 起動時の Pure Pursuit は `use_overtake_reference_override=true` で起動します。`/overtake/reference_override` が新しければ、Pure Pursuit は現在位置から先の trajectory 点を overtake planner の横オフセット分だけずらし、速度 cap も目標速度の上限として使います。これにより、MPC から Pure Pursuit に落ちている間も、追い越し・追従・譲りの速度抑制が制御指令へ反映されます。

Pure Pursuit の制御指令も新しくない場合は、停止指令を出します。

### MPC への復帰

フォールバック中に以下を満たすと MPC に復帰します。

- MPC の制御指令が新しい
- MPC health が有効
- `mpc_status` が `solved`
- `mpc_infeasible_count` が 0
- `fallback_min_hold_sec` 以上フォールバックを保持した
- 正常判定が `fallback_release_solved_cycles` 回続いた

フォールバックへ入った直後に即座に MPC へ戻ると切り替えが振動しやすいため、最小保持時間と連続正常回数の両方を使っています。

### 停止指令

以下の場合は停止指令を出します。

- MPC の指令が使えない
- Pure Pursuit の指令も使えない
- `enabled` が false で MPC 指令も使えない

停止指令では speed を `0.0`、acceleration を `stop_decel_mps2`、steering を `0.0` にします。

## debug 出力

`/hybrid_control_mux/debug` には JSON 形式で現在の選択状態が出ます。主なフィールドは以下です。

| フィールド | 内容 |
| --- | --- |
| `controller` | `hybrid_control_mux` 固定 |
| `source` | 最終出力に使った入力。`mpc`、`pure_pursuit`、`stop` のいずれか |
| `fallback_active` | フォールバック状態かどうか |
| `reason` | 現在の切り替え理由 |
| `solved_cycles` | 復帰判定で数えている連続正常回数 |
| `mpc_cmd_fresh` | MPC 指令が timeout していないか |
| `pure_pursuit_cmd_fresh` | Pure Pursuit 指令が timeout していないか |
| `mpc_health_valid` | MPC health が有効か |
| `mpc_status` | MPC health の `mpc_status` |
| `mpc_infeasible_count` | MPC health の `mpc_infeasible_count` |
| `mpc_health_age_sec` | 最後に health を受信してからの秒数 |
| `output_speed_mps` | 実際に出した速度指令 |
| `output_accel_mps2` | 実際に出した加速度指令 |
| `output_steer_rad` | 実際に出した操舵指令 |

## 注意点

Pure Pursuit は低速のコース追従用フォールバックです。hybrid 起動では overtake planner の横オフセットと速度 cap を反映しますが、MPC のような最適化や制約解決は行いません。

そのため、フォールバック速度を高くしすぎると、MPC が苦しい場面で速度だけ維持してしまい、かえって危険になる可能性があります。特にサイドバイサイドでコーナーに入る場面では、Pure Pursuit への切り替えは「制約を解いた安全回避」ではなく「overtake planner の判断を使いながら低速でコースを追う退避動作」として扱ってください。
