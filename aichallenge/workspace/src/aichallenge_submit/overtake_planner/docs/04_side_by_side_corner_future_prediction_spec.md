# サイドバイサイド・コーナー予測仕様メモ

この文書は、`overtake_planner` がサイドバイサイド状態でコーナーへ入るときの現状仕様、起きている問題、見落としやすい点、実装済みの未来予測仕様をまとめたものです。
未来予測仕様は、コーナー手前で横並び継続が危険かを判断し、早めに `YIELD_BEHIND` へ倒すためのものです。

対象パッケージ:

```text
aichallenge/workspace/src/aichallenge_submit/overtake_planner/
```

関連するMPC側:

```text
aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/
```

## 背景

サイドバイサイド状態のままコーナーへ入ると、外側車両が曲がりきれず壁側へ膨らむことがあります。
ログ上は、その前後でMPCの横方向制約が厳しくなり、`!!!! Infeasible path detected !!!!` や `opponent_collision` が出ることがあります。

重要なのは、MPCだけが悪いわけではないことです。
`overtake_planner` がコーナー手前で横並び継続を許し、MPCへ横方向overrideを出した結果、MPC側では次を同時に満たす必要があります。

- 他車との安全距離を守る
- 壁マージンを守る
- コーナーの曲率に沿って曲がる
- ステアレート制限内で横位置を戻す
- 短いhorizon内で解を作る

これらが同時に厳しくなると、MPCの実行可能領域がなくなります。

## 現状仕様

### 入力

`overtake_planner_node` は次を使います。

- `/localization/kinematic_state`
  - 自車位置、yaw、速度
- `/v2x/vehicle_positions`
  - 他車IDと位置
- 参照CSV
  - Frenet座標の基準線

V2Xにはyawがないため、他車速度は前回位置との差分から推定します。

```text
vx = (x_now - x_prev) / dt
vy = (y_now - y_prev) / dt
v = hypot(vx, vy)
```

位置ジャンプが大きい場合は速度推定を0へ戻します。

### 自車ID除外

`own_vehicle_id` が `auto` の場合、`ROS_DOMAIN_ID=N` から `dN` を推定します。
このIDと一致するV2X車両は他車リストから除外します。

### 他車検出

`collectOpponents()` で他車をFrenet座標へ変換します。

`detectBlocked()` では現在位置ベースで次を判定します。

- 前方車両
  - `delta_s > 0`
  - `delta_s < lookahead_s_m`
- 同一コリドー
  - `abs(delta_d) < same_corridor_width_m`
- 横並び
  - `abs(signed_delta_s) < side_by_side_s_m`
  - `abs(delta_d) < side_margin_m`
- 並走候補
  - `parallel_side_detection_enabled == true`
  - `abs(signed_delta_s) < parallel_side_s_m`
  - `abs(delta_d) < parallel_side_margin_m`

ここでの横並び判定は現在の `s/d` 関係を見ます。
並走候補は通常の `side_by_side` にはしません。
スタート直後の別ライン並走や、斜め後方の相手を未来コーナー予測へ渡すための入口として使います。
同方向判定と未来の横並びコーナー判定は、この後の処理で追加評価します。

### blocked判定

同一コリドー内の前方車両がある場合、次で `blocked` を決めます。

- 自車が相手より `dv_block_threshold_mps` 以上速い
- または `front_delta_s < follow_trigger_s_m`

横並びだけの場合は `side_by_side=true` でも、`blocked=false` になり得ます。
この状態では追従や追い越し開始の判断とは別に、横並び専用の処理が必要になります。

### コーナー横並び判定

`corner_side_by_side` は次で決まります。

```text
side_by_side == true
and maxAbsCurvatureAhead(ego_s, corner_side_yield_lookahead_m) >= corner_side_yield_curvature_m_inv
```

つまり、現在横並びで、近い先の曲率が大きいと「コーナー横並び」と見ます。
ただし、未来の相手位置はこの判定には直接使っていません。

### 未来予測の使われ方

`predictOpponents()` は他車を短いhorizonで予測します。

```text
x(t) = x + vx * t
y(t) = y + vy * t
```

その後、予測点をFrenet座標へ戻して安全評価に使います。

現状の使い道:

- 左右のpass gap評価
- 候補軌道と他車予測の安全楕円評価

現在は、通常の短期予測を「候補が安全か」の評価に使い、別途Frenetベースの未来予測を「横並びコーナーに入る前に譲るべきか」の判断に使います。

### 候補生成

主な候補は次です。

- `FASTEST`
  - overrideなし相当
- `FOLLOW`
  - 前走車より少し低い速度上限
- `PASS_LEFT`
  - 左オフセット
- `PASS_RIGHT`
  - 右オフセット
- `SIDE_BY_SIDE_KEEP`
  - 横並び相手から距離を取る
- `YIELD_BEHIND`
  - 相手の後ろへ入るために減速
- `RECOVERY`
  - 中心線側へ戻る
- `SAFE_STOP`
  - 通常候補が安全に成立しないときの最後のfallback

### SIDE_BY_SIDE_KEEP

横並び時は、相手から離れる方向へ `target_d` を作ります。

```text
away_sign = opponent_delta_d > 0 ? -1 : +1
gap_target = opponent_d + away_sign * side_by_side_target_gap_m
target_d = clamp(gap_target, lower_d, upper_d)
```

この仕様は直線では自然ですが、コーナー外側の車両では壁側へ逃げる方向になることがあります。
その結果、外側車両がMPCの通行可能範囲から外れやすくなります。

### YIELD_BEHIND

横並びで相手が前へ出た場合や、コーナー横並びで壁余裕が小さい場合、`YIELD_BEHIND` に倒します。

コーナー横並び中は、速度上限が `corner_yield_v_max_mps` で抑えられます。
ただし、譲りへ入るタイミングが遅いと、すでに横方向制約が潰れており、MPCが解けないことがあります。
未来予測で `YIELD_BEHIND` に入った場合は、横並びが一瞬解けても、近いコーナーが残っている間は `future_yield_hold` として譲りを保持します。

### SAFE_STOP

`SAFE_STOP` は、通常候補がすべてunsafeで、左右追い越しもできない場合に使う最後のfallbackです。
停止候補自体も安全評価に通る必要があります。
停止候補が他車や壁でunsafeな場合は、unsafeな横方向候補を有効化しません。
その代わり、現在横位置を保持するd列と低い速度capだけを出す速度only fallbackを残します。

注意点:

- YAMLでは `safe_stop_v_mps: 0.20` として、小さい正値を明示します。
- コード側ではMPCへ0速度を渡さないよう、実際の速度capは `1.0e-3` 以上へ丸めます。
- 仕様としては、YAMLも小さい正値に揃えた方が読みやすいです。

## MPCが解けなくなる流れ

ログで見えている典型的な流れは次です。

1. 横並び、または前走車に接近する。
2. コーナーへ入る。
3. `overtake_planner` が横方向overrideを出す。
4. 自車の `d` が大きく外側へ出る。
5. MPC側のpath constraintが狭くなる。
6. 安全マージンや壁マージンを引くと `ub_sm < lb_sm` になる。
7. `!!!! Infeasible path detected !!!!` が出る。
8. 候補が `opponent_collision` や `wall_margin` でrejectされる。
9. overrideが無効化される、またはMPCが古い制御を使い続ける。

`!!!! Infeasible path detected !!!!` はOSQPの直接ログではありません。
`multi_purpose_mpc_ros/core/reference_path.py` の path constraint生成で、横方向の通行可能幅が消えたときに出ます。

## 速度only fallback

横方向候補が `opponent_collision` や `wall_margin` でrejectされた場合、plannerはその候補をMPCへ復活させません。
代わりに `BehaviorMode::SPEED_GUARD` または現在の非zero modeで、現在横位置を保持するd列と低い `v_ref` だけを `/overtake/reference_override` へ出します。
MPCは `mode_id==0` または `n<=0` のoverrideを消すため、速度onlyでも非zero modeを使います。

速度only fallbackの発動理由は `speed_cap_reason` で確認します。

- `speed_only_fallback_opponent_collision`
- `speed_only_fallback_safe_stop_infeasible`
- `wall_risk_speed_guard`
- `mpc_health_infeasible_guard`
- `mpc_health_solve_time_guard`
- `mpc_health_stale_guard`

MPC healthは `/mpc/speed_profile_debug` を読みます。
debugが `mpc_health_stale_time_sec` より古い場合も、solver停止やpublisher停止の可能性があるため保守的に速度を落とします。

## 見落としやすい点

### 同方向判定

実装では、他車が自車と同じ向きに進んでいるかをFrenet参照線方向速度で評価します。
低速停止車両は方向不明として障害物扱いを残します。

実装済みの観点:

- V2X速度を参照線接線方向へ投影した `s_dot`
- `s_dot > 0` なら同方向
- `s_dot < 0` なら逆方向または誤検出候補
- 低速停止車両は方向不明として障害物扱いを残す

### 未来のサイドバイサイド

実装では「今横並びか」に加えて、未来でも横並びが継続し、コーナーと壁余裕不足が重なるかを見ます。
しかし問題は「今はまだぎりぎり成立しているが、1秒後にコーナーで横幅がなくなる」ことです。

実装済みの観点:

- 0.5秒後、1.0秒後、1.5秒後の `delta_s`
- 未来の `delta_d`
- 未来の曲率
- 未来の壁余裕

### コーナーでの未来予測

安全評価用の短期予測は従来どおり `x + vx*t`, `y + vy*t` です。
一方、横並びコーナー判定では、相手車両も参照線に沿って曲がる前提でFrenet予測を使います。

実装済みの観点:

- Cartesian等速予測だけでなくFrenet予測を使う
- `s += s_dot * t`
- `d` は現在値を維持、または緩やかに中心線へ戻すモデルにする

### 外側車両の壁余裕先読み

`SIDE_BY_SIDE_KEEP` は相手から離れる方向を選びます。
外側車両では、その方向が壁側になることがあります。

実装済みの観点:

- 自車が外側かどうか
- 相手から離れたときの未来 `target_d`
- `target_d` に対する未来壁余裕
- 壁余裕が不足するなら、横へ逃げずに `YIELD_BEHIND` へ倒す

### horizonが短い

基本YAMLでは、MPCへ渡すoverride列は `horizon_points * horizon_dt_sec = 20 * 0.025 = 0.50 sec` です。
これはbaseline MPCの `N: 20` と契約を合わせるためです。
delay-aware MPC経路では `delay_aware_mpc.launch.xml` から `horizon_points=50` に上書きし、delay-aware MPCの `N: 50` と揃えます。
コーナー進入前に譲る判断は、override列とは別に `future_side_prediction_horizon_sec = 1.2 sec` で見ます。

実装済みの観点:

- 判定用の予測horizonをoverride列とは別に持つ
- MPCへ渡すoverride点数とは別に、判断用horizonを持つ

### debug強化

debugでは、なぜ未来で詰むと判断したかを見られるようにします。

実装済みdebug:

- `future_side_by_side`
- `future_corner_side_by_side`
- `future_delta_s`
- `future_delta_d`
- `future_wall_clearance_m`
- `future_outer_wall_risk`
- `predicted_opponent_s`
- `predicted_opponent_d`
- `parallel_side_candidate`
- `parallel_side_vehicle_id`
- `parallel_side_delta_s`
- `parallel_side_delta_d`
- `parallel_side_lateral_gap_m`
- `same_direction`
- `opponent_s_dot_mps`
- `yield_reason`
- `speed_only_fallback_active`
- `wall_risk_speed_guard_active`
- `mpc_health_speed_guard_active`
- `speed_cap_reason`
- `applied_speed_cap_mps`
- `active_section_name`
- `active_section_profile`
- `mpc_health_age_sec`

## 実装済みの未来予測仕様

### 目的

コーナーに入ってから回避するのではなく、コーナー手前で横並び継続が危険かを判断し、早めに `YIELD_BEHIND` または減速へ倒す。

### 方針

次の条件を満たす場合、横並び維持より譲りを優先します。

```text
future_side_by_side == true
and future_corner_curvature >= corner_side_yield_curvature_m_inv
and future_wall_clearance_m < required_wall_clearance
```

parallel観測からの譲りでは、狭義の横並びへの接近も必要です。

```text
parallel_side_candidate == true
and future_parallel_interaction == true
and future_side_by_side == true
and future_corner_side_by_side == true
and future_outer_wall_risk == true
```

### 予測モデル

相手車両について、現在の `vx/vy` から参照線方向速度を求めます。

```text
ref_yaw = reference_yaw(opponent_s)
s_dot = vx * cos(ref_yaw) + vy * sin(ref_yaw)
```

方向不明条件:

```text
opponent_v < same_direction_min_speed_mps
```

同方向条件:

```text
s_dot >= same_direction_min_s_dot_mps
```

未来位置:

```text
pred_s(t) = opponent_s + s_dot * t
pred_d(t) = opponent_d
```

最初は `pred_d(t) = opponent_d` でよいです。
相手の横移動まで推定すると誤差が増えやすいためです。

### 判定ステップ

1. 現在の `side_by_side` または `parallel_side_candidate` を見る。
2. `s_dot` から相手の進行方向を推定する。
3. 方向不明または同方向なら、未来予測対象に残す。
4. 予測時刻 `t = 0.5, 1.0, 1.5` 秒を評価する。
5. 各時刻で `future_delta_s`, `future_delta_d` を計算する。
6. 狭義の横並びしきい値へ入り、parallel候補なら現在より接近している時だけ `future_side_by_side=true`。
7. その時刻の自車予定 `d` と相手 `d` から、外側壁余裕を見積もる。
8. 曲率が高い、または未来の外壁余裕が不足する場合、`YIELD_BEHIND` 候補を優先する。

### YIELD_BEHINDへ倒す条件

```text
future_side_by_side
and future_corner_side_by_side
and future_outer_wall_risk
```

真の現在横並びは従来どおり壁リスクを保持します。parallel候補からの譲りは、横並びへの接近・コーナー・壁リスクを同時に満たす時だけ `YIELD_BEHIND` へ倒します。

推奨初期値:

```yaml
future_side_prediction_enabled: true
future_side_prediction_horizon_sec: 1.2
future_side_prediction_dt_sec: 0.3
future_side_yield_wall_clearance_m: 0.35
parallel_side_detection_enabled: true
parallel_side_s_m: 12.0
parallel_side_margin_m: 4.0
same_direction_min_speed_mps: 0.30
same_direction_min_s_dot_mps: 0.05
```

### 追加・整合したパラメータ

未来横並びコーナー予測のため、以下を新規追加しました。
`PlannerConfig`、nodeの `declare_parameter()`、`overtake_planner.param.yaml` の初期値は同じです。

| パラメータ | 初期値 | 使い方 |
|---|---:|---|
| `same_direction_filter_enabled` | `true` | 同方向ではない相手をfront/side対象から外す。 |
| `same_direction_min_speed_mps` | `0.30` | これ未満の低速車は方向不明として扱い、障害物から外さない。 |
| `same_direction_min_s_dot_mps` | `0.05` | 参照線方向速度 `s_dot` がこの値以上なら同方向。 |
| `future_side_prediction_enabled` | `true` | 未来横並びコーナー予測を使う。 |
| `future_side_prediction_horizon_sec` | `1.2` | 未来横並びを判定する最大時間。 |
| `future_side_prediction_dt_sec` | `0.3` | 未来判定のサンプル間隔。 |
| `future_side_yield_wall_clearance_m` | `0.35` | 未来の壁余裕がこの値未満なら `future_yield_required=true` にする。 |
| `parallel_side_detection_enabled` | `true` | 通常横並びより広い並走候補を未来予測へ渡す。 |
| `parallel_side_s_m` | `12.0` | 並走候補として見る前後方向の範囲。 |
| `parallel_side_margin_m` | `4.0` | 並走候補として見る横方向の範囲。 |

今回の挙動を安全寄りに揃えるため、以下の既存パラメータもYAML、node default、`PlannerConfig` defaultで同じ値にしました。

| パラメータ | 初期値 | 目的 |
|---|---:|---|
| `lookahead_s_m` | `10.0` | 遠すぎる相手より近い相手の閉塞判断を優先する。 |
| `horizon_points` | `20` | baseline MPCの `N: 20` とoverride列を合わせる。delay-aware経路ではlaunchで50へ上書きする。 |
| `side_yield_s_m` | `0.30` | 相手が前に出た時点で譲る。微小なs差だけで譲り続けないよう余裕を残す。 |
| `side_by_side_target_gap_m` | `0.75` | 横並び維持で壁側へ逃げすぎる力を弱める。 |
| `corner_yield_v_max_mps` | `3.0` | コーナー横並びの譲り速度を抑える。 |
| `speed_only_fallback_v_max_mps` | `1.0` | unsafe候補を横方向に復活させず速度だけ落とす。 |
| `opponent_collision_fallback_v_max_mps` | `0.5` | `opponent_collision` で横候補がunsafeな場合の安全ゲート向け低速cap。 |
| `wall_soft_margin_m` | `0.25` | 壁リスク速度ガードを始めるソフト余裕。 |
| `mpc_health_v_max_mps` | `3.0` | MPC health悪化またはstale時の速度上限。 |
| `large_lateral_error_threshold_m` | `0.60` | 横ずれが大きい状態で壁/横並び/閉塞/未来横並びリスクがある場合、追い越し判断を凍結して `RECOVERY` を優先する。 |
| `large_lateral_error_v_max_mps` | `2.5` | 大きい横ずれ中の復帰・譲り速度を抑える。 |
| `recovery_release_lateral_error_m` | `0.60` | `ABORT_RECOVERY` を抜けるために必要な中心線からの横ずれ上限。負値で無効。 |
| `yield_release_lateral_error_m` | `0.60` | `YIELD_BEHIND` と `future_yield_hold` を抜けるために必要な中心線からの横ずれ上限。負値で無効。 |
| `min_pass_gap_m` | `1.80` | 狭い隙間の追い越しを抑える。 |
| `min_wall_margin_m` | `0.50` | 壁余裕を広めに取る。 |
| `safety_ellipse_b_m` | `1.8` | 横方向の他車接近に厳しくする。 |
| `safe_stop_v_mps` | `0.20` | 0速度ではなく小さい正値で停止意図を表す。 |
| `safe_stop_trigger_cycles` | `1` | 回避不能時に早く安全停止へ倒す。 |

未来譲り保持は `min_mode_hold_time_sec` の最低保持時間を尊重します。
保持解除は `yield_rejoin_wall_clearance_m`、`yield_release_lateral_error_m`、`corner_side_yield_curvature_m_inv`、`yield_rejoin_gap_m`、`corner_yield_rejoin_gap_m` を使います。

### 未来譲り保持

`future_yield_required=true` で `YIELD_BEHIND` に入った後は、次を満たすまで譲りを保持します。

- 自車の壁余裕が `yield_rejoin_wall_clearance_m` 以上
- 中心線からの横ずれが `yield_release_lateral_error_m` 以下
- `min_mode_hold_time_sec` 以上保持済み
- 近い先の曲率が `corner_side_yield_curvature_m_inv` 未満
- 前方ターゲットがいる場合は、`max(yield_rejoin_gap_m, corner_yield_rejoin_gap_m)` 以上のギャップがある

保持中は `yield_reason="future_yield_hold"` をdebugへ出します。

### SIDE_BY_SIDE_KEEPの変更

未来壁余裕が不足する場合は、相手から離れる横移動を弱めます。

候補:

- `target_d` を現在値より外側へ出さない
- `target_d` を中心線側へ寄せる
- 速度capを `corner_yield_v_max_mps` に近づける
- `SIDE_BY_SIDE_KEEP` より `YIELD_BEHIND` を優先する

最初の実装では、複雑な横位置最適化より `YIELD_BEHIND` 優先が安全です。

### 区間安全プロファイル

区間ごとに安全ポリシーを強めたい場合は、section safety profileを使います。
`side_by_side_corner_strict` は壁soft marginを1.35倍、速度capを0.65倍にし、`outer_yields` と組み合わせると外側車両の譲りを強めます。
YAMLに空配列は置かず、使うときだけ同じ長さの `section_safety_names`、`section_safety_profiles`、`section_safety_role_policies`、`section_safety_start_wp`、`section_safety_end_wp` を追加します。

## 実装対応

### 対応1: 未来横並びコーナー判定

効果が大きく、既存構造へ足しやすいです。

変更箇所:

- `types.hpp`
  - 未来リスク情報を `BlockedInfo` に追加
- `overtake_planner_core.cpp`
  - 予測判定関数を追加
  - `shouldYieldBehindSideBySide()` に未来リスクを反映
- `overtake_planner_node.cpp`
  - debug JSONへ未来リスクを出す
- `overtake_planner.param.yaml`
  - 新パラメータを追加

### 対応2: 同方向判定

誤検出や逆方向車両をovertake対象にしにくくします。
停止車両は無視せず、方向不明の障害物として扱います。

### 対応3: debug強化

なぜ譲ったか、なぜ横並び維持したかをログから読めるようにします。
チューニング効率がかなり上がります。

### 対応4: Frenetベース予測

横並びコーナー判定用の相手予測をFrenetベースにしています。
通常の安全評価用 `predictOpponents()` は、既存挙動を崩さないようCartesian等速予測のままです。

## テスト観点

実装済みテスト:

- 横並びで直線なら `SIDE_BY_SIDE_KEEP` を維持する。
- 横並びでコーナーが近く、外側壁余裕が不足するなら `YIELD_BEHIND` へ入る。
- 相手が少し前にいる場合は、現在より早く `YIELD_BEHIND` へ入る。
- 逆方向に動く相手は、前走車/横並び対象にしない。
- 停止車両は方向不明として無視しない。
- 未来予測で譲る場合、debugに `future_corner_side_by_side=true` と理由が出る。

## チューニング方針

まずは安全寄りにします。

```yaml
future_side_prediction_horizon_sec: 1.2
future_side_yield_wall_clearance_m: 0.35
corner_yield_v_max_mps: 3.0
side_yield_s_m: 0.30
mpc_health_v_max_mps: 3.0
```

挙動が保守的すぎる場合は、次の順で戻します。

1. `future_side_yield_wall_clearance_m` を下げる
2. `future_side_prediction_horizon_sec` を短くする
3. `corner_yield_v_max_mps` を上げる
4. `side_yield_s_m` を上げる

## まとめ

今の問題は、サイドバイサイドでコーナーへ入ったあとに回避しようとして、MPC側の横方向実行可能領域が消えることです。
そのため、改善の主眼は「コーナー中にうまく避ける」ではなく、「コーナーに入る前に横並び継続が危ないと判断して譲る」ことです。

最初に実装すべきなのは、未来横並びコーナー判定と、それに基づく早期 `YIELD_BEHIND` です。
