# evalwrap レポート出力ガイド ROS 2 初心者向け

この資料は、`tools/evalwrap` が作るレポートと出力ファイルを、ROS 2 初心者でも
追えるように分けて説明したものです。

`evalwrap` は、AI Challenge の評価結果を集めて、あとから比較しやすい形に
変換するローカル専用ツールです。公式評価のJSON、ログ、rosbag、MPC参照経路を
まとめて読み、HTMLレポートとCSV/JSONを生成します。

## まず全体像

評価または取り込みを実行すると、結果は次の場所に保存されます。

```text
analysis/runs/<run_id>/
```

`<run_id>` は、日時とラベルを組み合わせた名前です。例:

```text
20260624_123456_mpc-headless-eval
```

中身は大きく4種類に分かれます。

| 種類 | パス | 役割 |
| --- | --- | --- |
| 実行メタ情報 | `manifest.yaml` | いつ、どの設定で、どのGit差分で走らせたかを記録します。 |
| 生データ | `raw/d1/` など | 公式評価の出力、ログ、rosbagをコピーしたものです。 |
| 処理済みデータ | `processed/` | グラフや比較に使いやすいCSV/JSONです。 |
| HTMLレポート | `report/index.html` | ブラウザで見るまとめレポートです。 |

## 実行コマンド別の違い

| コマンド | 何をするか |
| --- | --- |
| `tools/evalwrap run --label ...` | 提出アーカイブ作成、evalイメージビルド、`make eval`、成果物収集、レポート生成を行います。 |
| `tools/evalwrap run --skip-build --label ...` | 既存ビルドを使って `make eval` と収集を行います。 |
| `tools/evalwrap ingest --label ... --path output/latest` | 既にある評価出力を収集してレポート化します。 |
| `tools/evalwrap list` | 収集済みrunの一覧を出します。 |
| `tools/evalwrap leaderboard --metric total_time_sec` | 指定メトリクスで順位表を出します。 |
| `tools/evalwrap compare --base A --target B` | 2つのrunを比較するHTMLを作ります。 |

GUIの `evalwrap` ボタンは `run`、`log化` ボタンは `ingest` に近い役割です。

## ROS 2 初心者向けの見方

| データ | 何から来るか | 見る目的 |
| --- | --- | --- |
| 公式JSON | `result-summary.json`、`d1-result-details.json` | 完走したか、ラップタイム、ペナルティなどを確認します。 |
| Autowareログ | `autoware.log` | エラー、警告、落ちたノードのヒントを探します。 |
| rosbag | `rosbag2_autoware/` | 車両位置、速度、制御指令などのtopicをあとから解析します。 |
| 参照経路CSV | MPC設定やrosbag内のtrajectory | コーナー番号、経路誤差、速度プロファイルの土台にします。 |

`rosbag_available` は「rosbagを読むことができたか」です。launchで
`rosbag:=true` を指定したかどうかの単純なコピーではありません。`metadata.yaml`
がまだできていない、ROS Python依存が足りない、必要topicが入っていない、などでも
`false` になります。

## manifest.yaml

```text
analysis/runs/<run_id>/manifest.yaml
```

実行の説明書です。主に次の内容が入ります。

| 項目 | 内容 |
| --- | --- |
| `run_id` | このrunのIDです。 |
| `label` | GUIやコマンドで付けたラベルです。 |
| `created_at` | 実行開始時刻です。 |
| `status` | `success`、`partial`、`failed` のどれかです。 |
| `repo.root` | 評価したリポジトリの場所です。 |
| `repo.branch` | 実行時のGitブランチです。 |
| `repo.commit` | 実行時のGitコミットです。 |
| `repo.dirty` | 未コミット差分があったかどうかです。 |
| `repo.diff_hash` | 差分を識別するためのハッシュです。 |
| `repo.diff_patch` | 実行時差分を保存した `diff.patch` への参照です。 |
| `submission.tar_path` | 提出アーカイブのパスです。 |
| `submission.tar_sha256` | 提出アーカイブのハッシュです。 |
| `eval.mode` | `single`、`parallel`、`ingest` などの実行モードです。 |
| `eval.command` | 実行したコマンド列です。 |
| `eval.domains` | `d1`〜`d4` のうち収集できた領域です。 |
| `reference_trajectory` | 参照経路の読み込み元、点数、警告です。 |
| `notes` | ラベルやメモです。 |
| `warnings` | 収集や解析で出た警告です。 |

`partial` は失敗と同じではありません。「公式JSONは読めたがrosbagが読めない」
「一部CSVが作れない」など、使える情報だけでレポートを作った状態です。

## raw ディレクトリ

```text
analysis/runs/<run_id>/raw/d1/
```

公式評価の出力をコピーした場所です。`d1` は評価ドメイン名で、複数車両や
複数提出の評価では `d2`、`d3`、`d4` も出ることがあります。

| ファイル/ディレクトリ | 内容 |
| --- | --- |
| `result-summary.json` | 公式の概要結果です。完走、ラップ数、最速ラップなどの元になります。 |
| `d1-result-details.json` | 公式の詳細結果です。ラップタイムや詳細メトリクスの元になります。 |
| `autoware.log` | AutowareやROS 2ノードの標準出力/エラーです。 |
| `rosbag2_autoware/` | ROS 2 topic の録画です。速度、位置、制御指令などの解析元です。 |
| `capture/` | 評価側が出したキャプチャ類がある場合に入ります。 |
| `motion_analytics-*.html` | 公式側または別ツールが出したモーション解析HTMLがある場合に入ります。 |

## processed ディレクトリ

```text
analysis/runs/<run_id>/processed/
```

HTMLレポートやGUIの `Motion Log` が使う処理済みファイルです。

### metrics.json

```text
processed/metrics.json
```

run全体の代表値です。HTMLの `Result Summary` や `leaderboard` の元になります。

主な項目:

| 項目 | 意味 |
| --- | --- |
| `finish` | 完走したかどうかです。 |
| `total_time_sec` | 総走行時間です。 |
| `lap_count` | ラップ数です。 |
| `best_lap_sec` | 最速ラップです。 |
| `penalty_count` | ペナルティ数です。 |
| `collision_count` | 衝突数です。 |
| `stuck_count` | スタック検出回数です。 |
| `low_speed_time_sec` | 低速状態だった時間です。 |
| `avg_speed_mps` / `max_speed_mps` | 平均速度と最大速度です。 |
| `max_abs_steer_rad` | 実操舵角の最大絶対値です。 |
| `steer_oscillation_score` | 操舵の振れやすさを見る指標です。 |
| `max_accel_mps2` / `max_decel_mps2` | 実加速/実減速の最大値です。 |
| `max_command_accel_mps2` / `max_command_decel_mps2` | 制御指令として出した加速/減速の最大値です。 |
| `avg_path_error_m` / `max_path_error_m` | 参照経路からのズレです。 |
| `trajectory_source` | 参照軌道をどこから取ったかです。 |
| `rosbag_available` | rosbagを解析できたかどうかです。 |
| `judgement` | 成績や警告をまとめた判定です。 |

### lap_summary.csv

```text
processed/lap_summary.csv
```

ラップ単位の結果です。

| 列 | 内容 |
| --- | --- |
| `run_id` | run IDです。 |
| `domain_id` | `d1` などのドメインです。 |
| `lap` | ラップ番号です。 |
| `lap_time_sec` | ラップタイムです。 |
| `avg_speed_mps` / `max_speed_mps` | ラップ平均/最大速度です。 |
| `penalty_count` / `collision_count` | そのrunのペナルティ/衝突数です。 |

### section_summary.csv

```text
processed/section_summary.csv
```

rosbagから推定した区間ごとの概要です。AWSIMの公式セクションが取れない場合の
補助データとして使います。

| 列 | 内容 |
| --- | --- |
| `lap` | ラップ番号です。 |
| `section` | 区間番号です。 |
| `entry_time_sec` / `exit_time_sec` | 区間へ入った時刻/出た時刻です。 |
| `duration_sec` | 区間にかかった時間です。 |
| `avg_speed_mps` / `max_speed_mps` / `min_speed_mps` | 区間内の速度です。 |
| `event_count` | 区間内のイベント数です。 |
| `avg_path_error_m` / `max_path_error_m` | 区間内の経路誤差です。 |
| `distance_m` | 区間距離です。 |

### awsim_section_summary.csv

```text
processed/awsim_section_summary.csv
```

AWSIM側のセクション情報が記録されている場合に使う、より公式寄りの区間表です。
HTMLの `Section Splits` は、これがある場合こちらを優先します。

| 列 | 内容 |
| --- | --- |
| `lap` | ラップ番号です。 |
| `section` | AWSIMセクション番号です。 |
| `entry_time_sec` / `exit_time_sec` | run全体での入退出時刻です。 |
| `entry_lap_time_sec` / `exit_lap_time_sec` | ラップ内での入退出時刻です。 |
| `duration_sec` | セクション通過時間です。 |
| `avg_speed_mps` / `max_speed_mps` / `min_speed_mps` | セクション内の速度です。 |
| `avg_path_error_m` / `max_path_error_m` | セクション内の経路誤差です。 |
| `sample_count` | 集計に使ったサンプル数です。 |

### corner_summary.csv

```text
processed/corner_summary.csv
```

参照経路の曲率からコーナーを検出し、コーナーごとの走りをまとめたCSVです。
`awsim_section_summary.csv` がない場合、HTMLの `Section Splits` はこれを
フォールバックとして使います。

| 列 | 内容 |
| --- | --- |
| `corner_id` | `corner_01` などのコーナーIDです。 |
| `pass` | そのコーナーを何回目に通ったかです。周回すると2回目、3回目が出ます。 |
| `entry_time_sec` / `exit_time_sec` | コーナーに入った/出た時刻です。 |
| `duration_sec` | コーナー通過時間です。 |
| `entry_speed_mps` / `exit_speed_mps` | 入口/出口速度です。 |
| `min_speed_mps` / `avg_speed_mps` / `max_speed_mps` | コーナー中の速度です。 |
| `avg_path_error_m` / `max_path_error_m` | コーナー中の経路誤差です。 |
| `entry_distance_m` / `exit_distance_m` | 走行距離ベースの入口/出口位置です。 |
| `start_track_s_m` / `end_track_s_m` | 参照経路上の入口/出口位置です。 |
| `corner_length_m` | コーナー長です。 |
| `peak_curvature_1pm` | 最大曲率です。曲がりの強さを見る値です。 |
| `event_count` | コーナー中に発生したイベント数です。 |

### trajectory_reference.csv

```text
processed/trajectory_reference.csv
```

レポートの地図表示やコーナー検出の基準になる参照経路です。

| 列 | 内容 |
| --- | --- |
| `point_index` | 参照経路上の点番号です。 |
| `x_m` / `y_m` / `z_m` | 地図座標です。 |
| `track_s_m` | 経路に沿った距離です。 |
| `grade_percent` / `grade_rad` | 勾配です。 |
| `trajectory_curvature_1pm` | 曲率です。コーナー検出に使います。 |
| `corner_id` | その点が属するコーナーIDです。 |
| `trajectory_source` | rosbag内trajectoryか、MPC CSVフォールバックかを示します。 |

### vehicle_timeseries.csv

```text
processed/vehicle_timeseries.csv
```

車両状態の時系列です。rosbag内の車両位置や速度から作られます。

| 列 | 内容 |
| --- | --- |
| `time_sec` | 時刻です。 |
| `x_m` / `y_m` / `z_m` | 車両位置です。 |
| `distance_m` | 走行距離です。 |
| `section` | 区間番号です。 |
| `corner_id` | 近い参照経路上のコーナーIDです。 |
| `track_s_m` | 参照経路上の位置です。 |
| `trajectory_curvature_1pm` | その位置の曲率です。 |
| `trajectory_z_m` | 参照経路側の高さです。 |
| `trajectory_grade_percent` | 参照経路から見た勾配です。 |
| `grade_percent` / `grade_rad` | 実走行から見た勾配です。 |
| `grade_source` | 勾配の算出元です。 |
| `speed_mps` | 実速度です。 |
| `acceleration_mps2` | 実加速度です。 |
| `steering_rad` | 実操舵角です。 |
| `yaw_rate_rps` | ヨーレートです。 |
| `path_error_m` | 参照経路からの横ズレです。 |

### control_timeseries.csv

```text
processed/control_timeseries.csv
```

制御指令の時系列です。車両が「何をしようとしたか」を見るデータです。

| 列 | 内容 |
| --- | --- |
| `time_sec` | 時刻です。 |
| `target_speed_mps` | 目標速度です。 |
| `accel_mps2` | 指令加速度です。 |
| `steer_rad` | 指令操舵角です。 |
| `throttle` | スロットル指令です。 |
| `brake` | ブレーキ指令です。 |

### delay_aware_debug.csv

```text
processed/delay_aware_debug.csv
```

delay-aware系の制御で、遅延補償がどう働いたかを見るデバッグCSVです。

| 列 | 内容 |
| --- | --- |
| `mode` | 遅延補償モードです。 |
| `shifted` | 入力poseをずらしたかどうかです。 |
| `delay_sec` | 想定遅延秒数です。 |
| `prediction_steps` | 予測ステップ数です。 |
| `steering_source` | 操舵推定の元データです。 |
| `estimated_current_steering_rad` | 推定した現在操舵角です。 |
| `applied_steering_rad` | 補償後に使った操舵角です。 |
| `input_pose_*` | 補償前poseです。 |
| `delayed_pose_*` | 補償後poseです。 |

実装側で項目が増えた場合は、追加列もそのままCSVに出ます。

### speed_profile_debug.csv

```text
processed/speed_profile_debug.csv
```

目標速度がなぜその値になったかを見るデバッグCSVです。速度が急に落ちた理由を
探すときに使います。

| 列 | 内容 |
| --- | --- |
| `wp_id` | 参照経路上の近いウェイポイントIDです。 |
| `source` | 速度制限の理由です。例: `global`、`curvature`、`section`、`wall`。 |
| `target_speed_mps` | 最終的な目標速度です。 |
| `curvature_speed_mps` | 曲率から決まる速度上限です。 |
| `section_cap_mps` | 区間ごとの速度上限です。 |
| `global_cap_mps` | 全体の速度上限です。 |
| `actual_speed_mps` | 実速度です。 |
| `command_speed_mps` | 制御側が使った速度です。 |
| `use_curvature_speed_profile` | 曲率ベース速度を使ったかどうかです。 |
| `use_ref_vel_as_speed_cap` | 参照速度を上限として使ったかどうかです。 |
| `lateral_target_mode` | 横方向目標のモードです。 |
| `wall_margin_m` | 壁までの余裕距離です。 |
| `use_grade_accel_feedforward` | 勾配加速度フィードフォワードを使ったかどうかです。 |
| `grade_percent` | 勾配です。 |
| `grade_accel_base_mps2` | 勾配から計算した基本加速度補正です。 |
| `grade_accel_ff_mps2` | 実際に加えた勾配フィードフォワードです。 |

### grade_profile.csv

```text
processed/grade_profile.csv
```

勾配と加減速の関係を見るためのCSVです。

| 列 | 内容 |
| --- | --- |
| `distance_m` / `track_s_m` | 走行距離と参照経路上の距離です。 |
| `x_m` / `y_m` / `z_m` | 車両位置です。 |
| `trajectory_z_m` | 参照経路側の高さです。 |
| `grade_percent` / `grade_rad` | 勾配です。 |
| `grade_source` | 勾配の算出元です。 |
| `speed_mps` / `target_speed_mps` | 実速度と目標速度です。 |
| `acceleration_mps2` / `command_accel_mps2` | 実加速度と指令加速度です。 |
| `grade_accel_base_mps2` / `grade_accel_ff_mps2` | 勾配補正に関する加速度です。 |
| `command_steer_rad` | 指令操舵角です。 |

### motion_log.csv

```text
processed/motion_log.csv
```

GUIの `Motion Log` が最初に読むCSVです。速度、加速度、操舵、指令値を1つに
まとめた軽量な時系列です。

| 列 | 内容 |
| --- | --- |
| `time_sec` | 時刻です。 |
| `speed_mps` | 実速度です。 |
| `acceleration_mps2` | 実加速度です。 |
| `grade_percent` / `grade_source` | 勾配と算出元です。 |
| `steering_rad` | 実操舵角です。 |
| `target_speed_mps` | 目標速度です。 |
| `command_accel_mps2` | 指令加速度です。 |
| `grade_accel_base_mps2` / `grade_accel_ff_mps2` | 勾配補正です。 |
| `command_steer_rad` | 指令操舵角です。 |
| `throttle` / `brake` | スロットル/ブレーキ指令です。 |

### events.csv

```text
processed/events.csv
```

ペナルティ、衝突、低速、経路誤差、急加減速などのイベント一覧です。

| 列 | 内容 |
| --- | --- |
| `time_sec` | イベント時刻です。 |
| `lap` | ラップ番号です。 |
| `section` | 区間番号です。 |
| `event_type` | イベント種類です。 |
| `severity` | 重要度です。 |
| `description` | 説明文です。 |

## HTML レポート

```text
analysis/runs/<run_id>/report/index.html
```

ブラウザで見るメイン資料です。章ごとに見るポイントが違います。

### Run Overview

実行条件のまとめです。

| 表示項目 | 見ること |
| --- | --- |
| `label` | どの試行かを見分けます。 |
| `status` | 成功、部分成功、失敗を見ます。 |
| `created_at` | いつ走らせたかを見ます。 |
| `git_branch` / `git_commit` | どのコードで走らせたかを見ます。 |
| `dirty` / `diff_hash` | 未コミット変更込みかを見ます。 |
| `submit_sha256` | 提出アーカイブが同じかを見ます。 |
| `eval_mode` | `run` か `ingest` かなどを見ます。 |

### Result Summary

評価の代表値を表で見ます。最初に見るならここです。

見る順番のおすすめ:

1. `finish` が `true` か。
2. `total_time_sec` と `best_lap_sec` が改善しているか。
3. `penalty_count`、`collision_count`、`stuck_count` が増えていないか。
4. `avg_path_error_m`、`max_path_error_m` が大きくなっていないか。
5. `rosbag_available` が `true` か。

### Section Splits

区間またはコーナーごとの通過時間です。

優先順位:

1. `awsim_section_summary.csv` がある場合はAWSIMセクション表を表示します。
2. なければ `corner_summary.csv` を使ってコーナー別表を表示します。

AWSIMセクション表では、各セルにセクション通過時間、ラップ内出口時刻、最低/最高速度が
出ます。コーナー表では、各セルにコーナー出口時刻と通過時間が出ます。

コーナー表には `Corner Map` も付きます。表のコーナーと地図上のコーナーが
対応するので、「遅いのはどの曲がりか」を見つけやすくなっています。

### Speed Profile

実速度と目標速度の関係を見る章です。

| 図/表 | 内容 |
| --- | --- |
| actual speed map | コース上の実速度を色で表示します。遅い場所、速い場所が見えます。 |
| speed chart | 距離に対して、実速度、目標速度、指令速度を線で表示します。 |
| target speed drops | 目標速度が大きく下がった場所を表で表示します。 |

「コーナー前で減速できているか」「目標速度だけ下がって実速度が追いついていないか」
を見るのに使います。

### Track Diagnostics

コース上の問題箇所を地図で見る章です。

| 図 | 内容 |
| --- | --- |
| Speed Error Map | 目標速度と実速度の差です。 |
| Path Error Map | 参照経路からの横ズレです。 |
| Speed Limit Source Map | 速度制限の理由です。`global`、`curvature`、`section`、`wall` などで色が変わります。 |
| Event Marker Map | ペナルティ、警告、経路誤差などのイベント位置です。 |

ROS 2初心者向けに言うと、これは「どのtopic由来の数値が、コース上のどこで
悪くなったか」を見る地図です。

### Control Response

制御指令と車両の反応を比べる章です。

| 図 | 内容 |
| --- | --- |
| Acceleration Response | 指令加速度と実加速度、速度の関係を見ます。 |
| Steering Response | 指令操舵角と実操舵角、ヨーレートの関係を見ます。 |

指令は出ているのに車両が反応していない、操舵が遅れている、急な指令が多い、
といった問題を探します。

### Corner Performance

コーナーごとの良し悪しを地図と表で見る章です。

| 図/表 | 内容 |
| --- | --- |
| Corner Duration | コーナー通過時間を色で表示します。 |
| Corner Minimum Speed | コーナー中の最低速度を色で表示します。 |
| Corner Path Error | コーナー中の平均経路誤差を色で表示します。 |
| Corner Summary | コーナーごとの平均通過時間、最低速度、平均経路誤差などの表です。 |

どのコーナーが遅いか、どのコーナーで外しているかを見るのに使います。

### Grade & Acceleration Profile

坂や勾配と加減速の関係を見る章です。

| 図/表 | 内容 |
| --- | --- |
| grade map | コース上の勾配を色で表示します。 |
| grade chart | 距離に対して勾配、実加速度、指令加速度、勾配補正を表示します。 |
| extreme table | 勾配や加速度が大きい地点を表で表示します。 |

勾配フィードフォワードを入れたとき、上り坂や下り坂で加速度指令が自然に変わっているかを
確認できます。

### Artifacts

`raw/d1/` などにコピーされた生データへのリンクです。

| リンク | 内容 |
| --- | --- |
| `result-summary.json` | 公式概要JSONです。 |
| `d1-result-details.json` | 公式詳細JSONです。 |
| `autoware.log` | ログです。 |
| `rosbag2_autoware` | rosbagディレクトリです。 |
| `capture` | キャプチャ類です。 |
| `motion_analytics-*.html` | 追加のモーション解析HTMLです。 |

### Processed Files

`processed/` にあるCSV/JSONへのリンク集です。Excel、Python、pandasなどで
追加解析したいときはここから開きます。

### Log Excerpts

ログから拾った警告やエラーの抜粋です。

| 列 | 内容 |
| --- | --- |
| `domain` | `d1` などの対象です。 |
| `path` | どのログから拾ったかです。 |
| `line` | 行番号です。 |
| `text` | 実際の警告/エラー文です。 |

## よくある見方

### タイムが悪いとき

1. `Result Summary` の `total_time_sec`、`best_lap_sec` を見る。
2. `Section Splits` で遅い区間やコーナーを探す。
3. `Speed Profile` で目標速度が落ちているのか、実速度が追いついていないのかを見る。
4. `Track Diagnostics` で経路誤差やイベント位置を見る。

### コーナーで遅いとき

1. `Corner Performance` の `Corner Duration` を見る。
2. `Corner Minimum Speed` で落としすぎかを見る。
3. `Corner Path Error` でラインを外していないかを見る。
4. `corner_summary.csv` で数値を確認する。

### 制御が荒いとき

1. `Control Response` で指令加速度/指令操舵角を見る。
2. `motion_log.csv` またはGUIの `Motion Log` で時系列を見る。
3. `events.csv` で急加減速や経路誤差イベントを見る。
4. 必要なら `delay_aware_debug.csv` を見て遅延補償の状態を確認する。

### rosbag が読めないとき

1. `manifest.yaml` の `warnings` を見る。
2. `raw/d1/rosbag2_autoware/metadata.yaml` があるか見る。
3. `.mcap` または `.db3` があるか見る。
4. `Result Summary` の `rosbag_available` と `trajectory_source` を見る。
5. rosbagにtrajectoryがない場合は、MPC参照CSVフォールバックが効いているか見る。

## まず見るべき3ファイル

迷ったらこの順番で見ればOKです。

1. `report/index.html`
2. `manifest.yaml`
3. `processed/metrics.json`

細かく調べるときは、目的に合わせてCSVを開きます。

| 目的 | 開くCSV |
| --- | --- |
| ラップや区間 | `lap_summary.csv`、`awsim_section_summary.csv`、`section_summary.csv` |
| コーナー | `corner_summary.csv`、`trajectory_reference.csv` |
| 速度や操舵 | `motion_log.csv`、`vehicle_timeseries.csv`、`control_timeseries.csv` |
| 目標速度の理由 | `speed_profile_debug.csv` |
| 遅延補償 | `delay_aware_debug.csv` |
| 勾配と加減速 | `grade_profile.csv` |
| 警告や衝突 | `events.csv`、`autoware.log` |

レポートは「結果の点数表」だけではなく、「どのROS 2 topic由来のデータが、
コース上のどこで悪くなったか」を見るための資料です。HTMLで場所をつかみ、
CSVで数値を掘る、という使い方がいちばん迷いにくいです。
