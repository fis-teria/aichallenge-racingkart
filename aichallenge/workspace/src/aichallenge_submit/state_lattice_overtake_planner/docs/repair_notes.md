# State Lattice Overtake Planner 修正内容

## 対象

この修正は `state_lattice_overtake_planner` の未接続・未完成だった安全ロジックを、既存の MPC / Pure Pursuit override 契約を維持したまま補完するものです。

## 主な修正

### 1. 軌道と速度制約の統合

従来は各候補をほぼ `normal_speed_mps` で走行する前提で操舵速度を検査していたため、低速なら実行できる軌道も `steering_rate` で棄却されていました。

現在は候補の各点について以下の速度上限を算出し、減速可能性を後方伝播します。

- 操舵角速度上限
- 曲率による横加速度上限
- 基準経路 CSV の速度上限
- 加減速度上限

現在速度が候補の進入速度上限を超える場合は、横軌道を即時送信せず `PREPARE_OVERTAKE_LEFT/RIGHT` の速度のみの指令へ移ります。進入速度まで落ちた後に、同じ安全監査を通った横軌道を送信します。

### 2. 追越し候補の実コース適合

- 横移動量に応じて終端距離を延長
- 前走車の手前で横移動を完了する終端距離を算出
- 車体寸法、相手車寸法、V2X 不確かさ、安全距離から必要な追越し横オフセットを算出
- 基準曲率の異常値を物理上限内にサニタイズ
- Frenet 投影探索と壁判定の重複処理を削減

`final_ver3` の全 350 基準点、相手車なし、5 m/s の条件で、速度を落とせば成立する候補も含めた候補ゼロ地点は修正前の 275/350 から 65/350 へ減少しました。

### 3. V2X 対象消失時の復旧

- 追越し中に観測した全車 ID を永久必須化する処理を撤廃
- 対象車の短時間消失は等速予測と増加する不確かさで補完
- grace 期間を超えた場合は `YIELD_BEHIND` または `ABORT_RECOVERY` へ遷移
- 安全に中央へ戻った後、対象 ID、候補履歴、速度状態、検出器を明示的にリセット
- V2X timestamp の逆行と過大な位置ジャンプを fail-closed で拒否
- 同一 timestamp は直前速度を再利用

### 4. 後方安全確認

後方安全経路の各点に、復帰完了までの時刻を付与しました。後続車はその時刻まで等速予測されるため、現在位置だけでなく復帰時刻の接近も判定されます。

後方経路は、現在の横位置から中央へ戻る側へ扇状に広げます。

### 5. MPC health 連携

`/mpc/speed_profile_debug` の JSON から以下を読み取り、異常時は `SPEED_GUARD` を出力します。

- `mpc_infeasible_count`
- `mpc_solve_time_ms`
- health message age

正常復帰には連続した healthy sample を要求し、チャタリングを防止します。

### 6. 状態機械の補完

以下の状態を実動作へ接続しました。

- `FOLLOW_BLOCKED`
- `PREPARE_OVERTAKE_LEFT`
- `PREPARE_OVERTAKE_RIGHT`
- `OVERTAKE_LEFT`
- `OVERTAKE_RIGHT`
- `SIDE_BY_SIDE_KEEP`
- `MERGE_BACK`
- `ABORT_RECOVERY`
- `YIELD_BEHIND`
- `SAFE_STOP`
- `SPEED_GUARD`

### 7. 追越し禁止区間

`config/overtake_permission.csv` を追加しました。新しい追越しの開始時だけ現在区間と先読み区間を検査します。すでに横移動中の場合は、区間境界で指令を突然切らず、安全な継続または中央復帰を優先します。

プロファイルを有効にしているのに CSV が読めない、列数が違う、Waypoint が範囲外などの場合は、黙って追越しを許可せず Node を degraded / SAFE_STOP にします。

### 8. deadline 処理の整合性

Planner をコピーして試算し、50 ms deadline 内に完了した周期だけ内部状態を commit します。超過時に前周期 V3 を再利用する場合も、現在の自車・相手車状態で横軌道を再監査します。

通常周期が速度のみの `FOLLOW/PREPARE/YIELD/STOP` に移った時点で前周期 V3 キャッシュを破棄し、後続の deadline 超過で古い横軌道が復活しないようにしました。

### 9. 入力・状態リセット

- `allow_reverse=false` 時の後退入力を明示的に SAFE_STOP
- `hard_max_steer_rad` を最終物理上限へ接続
- `frontmost_s_tolerance_m` を前方判定へ接続
- CSV の `speed_mps` を候補速度上限へ接続
- 非有限曲率、非有限速度、負の基準速度を拒否
- 追越し完了時に候補、選択軌道、速度・加速度状態、対象履歴、検出器をリセット

### 10. Pure Pursuit + MPC horizon 専用launch

`aichallenge_submit_launch` に、State Lattice backendを固定した次のlaunchを追加しました。

- `launch/state_lattice_pure_pursuit_mpc_horizon.launch.xml`: AI Challenge全体起動用
- `launch/state_lattice_pure_pursuit_mpc_horizon_control.launch.xml`: control-only用

`pure_pursuit_mpc_horizon` の40点overrideと200 ms MPC solve-time guardをState Lattice側へも明示的に転送します。Planner YAMLは `state_lattice_param_path` で差し替えできます。

## 追加・変更した主なパラメータ

詳細な既定値は `config/state_lattice_overtake_planner.param.yaml` を参照してください。

- `minimum_lateral_transition_distance_m`
- `lateral_transition_distance_gain`
- `adaptive_pass_offset_enabled`
- `pass_lateral_extra_margin_m`
- `max_adaptive_lateral_offset_m`
- `minimum_obstacle_transition_distance_m`
- `rear_prediction_horizon_sec`
- `target_missing_prediction_grace_sec`
- `target_missing_forget_sec`
- `target_missing_uncertainty_growth_mps`
- `target_missing_recovery_speed_mps`
- `reference_speed_limit_enabled`
- `reference_curvature_sanity_limit_radpm`
- `opponent_lateral_tracking_margin_m`
- `opponent_longitudinal_tracking_margin_m`
- `candidate_entry_speed_tolerance_mps`
- `mpc_health_*`
- `overtake_permission_*`

## 検証

### ROS 非依存回帰テスト

```bash
CXX=g++ state_lattice_overtake_planner/test/run_standalone_regression.sh
CXX=clang++ state_lattice_overtake_planner/test/run_standalone_regression.sh
```

確認項目は、設定値・基準経路検証、速度と操舵制約の統合、PREPARE から追越しへの遷移、Pure Pursuit 契約、禁止区間、対象消失復旧、MPC health、後方時刻予測、後退 fail-closed、実マップ全周評価です。

修正時の結果:

- GCC: 46 passed, 0 failed
- Clang: 46 passed, 0 failed
- `final_ver3` 候補ゼロ: 65/350
- 平均候補生成時間: GCC 約 20 ms、Clang 約 13 ms
- 最大候補生成時間: 実行環境により GCC 約 50 ms、Clang 約 37 ms

### 下流契約テスト

```bash
PYTHONPATH=multi_purpose_mpc_ros pytest -q \
  multi_purpose_mpc_ros/test/test_overtake_contract.py \
  multi_purpose_mpc_ros/test/test_mpc_lateral_target.py \
  multi_purpose_mpc_ros/test/test_predicted_horizon_contract.py
```

修正時の結果: 35 passed

### Sanitizer

ROS 非依存テストを AddressSanitizer / UndefinedBehaviorSanitizer 付きで実行し、合成コースの 40 checks が成功しました。

## 残る制約と実機前確認

### 全安全評価OFF（デバッグ専用）

`safety_evaluation_enabled=false` は、stale/V2X必須判定、壁・相手車衝突、
後方監査、追越し許可、cost safe-stop、MPC health guard、planning deadline
fail-safe を無効化します。NaN防止、Frenet投影、軌道生成成立性、操舵・
加減速の物理的成立性はプロセス破損防止のため維持します。

通常の既定値は `true` です。通常設定は
`config/state_lattice_overtake_planner.param.yaml` の
`safety_evaluation_enabled` を `true` / `false` に変更します。

一時的にYAMLを上書きするデバッグ起動には次を使用します。

```bash
STATE_LATTICE_SAFETY_EVALUATION_ENABLED=false \
  make dev CONTROL_METHOD=state_lattice_pure_pursuit_mpc_horizon
```

OFF時はノードが `UNSAFE DEBUG MODE` をERRORログへ出し、
`/debug/overtake/metrics` の `safety_evaluation_enabled` が `false` になります。
環境変数が未指定または空の場合はYAML値を使用し、明示的な `true` / `false`
だけがYAMLを上書きします。

1. ROS 2 Humble で Node と control launch をビルドし、planner/launch 契約テストと `make` 経由の実パラメータ伝播を確認済みです。AWSIM の走行開始を伴う動的シナリオ評価は別途必要です。
2. 現行 V3 契約では、Pure Pursuit と MPC が同じ 20 点配列を異なる基準 index から使用します。Planner は両方の解釈と nearest-index 誤差を監査し、どちらかで危険なら速度のみへ落とします。このため安全側ですが、通行可能な軌道を保守的に拒否する場合があります。完全解決には下流も含む契約更新が必要です。
3. `final_ver3` の基準経路上で、静的 map の level-9 領域と車体が重なる地点は、wall layer の二重 inflation を除去した後で nominal 0/350、hard-margin 8/350、tracking-margin 21/350 です。残る地点は地図、基準経路、車体寸法、wall margin の実測校正が必要です。
4. GCC の全周評価では一部周期が 50 ms 付近になります。Node には transactional deadline fallback を実装していますが、実機 CPU 上で profiling し、必要なら候補数・投影窓・map 解像度を調整してください。
5. shared V3 契約の保守的監査と狭いコースのため、全地点で即時追越しを保証するものではありません。成立しない場合は `PREPARE`、`FOLLOW_BLOCKED`、または `SAFE_STOP` を明示的に出力します。
