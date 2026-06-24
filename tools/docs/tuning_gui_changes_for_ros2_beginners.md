# Tuning GUI 変更点ガイド ROS 2 初心者向け

この資料は、AI Challenge Tuning GUI に追加された機能と、これまでの変更点を
ROS 2 に慣れていない人でも追えるようにまとめたものです。

最初に押さえるポイントはこれです。GUI は「ROS 2 の launch ファイルや YAML
を直接探して編集する作業」と「評価コマンドをターミナルで組み立てる作業」を、
ブラウザからまとめて操作できるようにしたローカルツールです。

## 対象ファイル

- GUI 本体: `tools/tuning_gui/`
- 起動スクリプト: `tools/run_tuning_gui.bash`
- 評価ラッパー: `tools/evalwrap` と `tools/eval_wrapper/`
- ヘッドレス連携パッチ: `tools/scripts/setup.sh`

## ROS 2 初心者向けの用語

| 用語 | このGUIでの意味 |
| --- | --- |
| launch | 複数のROS 2ノードをまとめて起動する設定ファイルです。`*.launch.xml` や `*.launch.py` が該当します。 |
| YAML | ROS 2ノードへ渡すパラメータ設定です。速度上限、参照経路、トピック名などが入ります。 |
| topic | ROS 2ノード同士がデータを流す名前付きの通路です。例: 車両位置、速度、制御指令など。 |
| rosbag | topic の中身をあとで見返せるように録画したログです。GUIの `dev` は `ROSBAG=true` 付きで起動します。 |
| control_method | どの制御方式を使うかを選ぶlaunch引数です。例: `mpc`、`pure_pursuit` など。 |
| eval | 評価走行です。公式の `make eval` を走らせ、結果を `output/` や `analysis/runs/` に残します。 |

## 起動と管理

GUI はリポジトリルートから次のコマンドで起動します。

```bash
tools/run_tuning_gui.bash --background
```

ブラウザで `http://127.0.0.1:8765` を開きます。

管理用に次の操作も用意されています。

```bash
tools/run_tuning_gui.bash --status
tools/run_tuning_gui.bash --stop
tools/run_tuning_gui.bash --restart
```

## 画面全体の構成

| 画面 | できること |
| --- | --- |
| 左の `Control` | `control_method` を選び、XMLへ保存できます。保存後にビルドもできます。 |
| 左の `Files` | 制御方式ごとに関係するlaunch/YAML/CSVを選んで編集できます。 |
| 左の `Presets` | 現在の設定ファイル一式を名前付きで保存し、あとで戻せます。 |
| 中央のエディタ | 表編集、テキスト編集、Path Editor、検証、差分確認、保存ができます。 |
| 右の `Run` | build、dev、gate、evalwrap、quick eval、log化、down、stop、psを実行できます。 |
| 右の `Lap History` | `output/` 側の最近の走行結果を確認できます。 |
| 右の `Reports` | `analysis/runs/<run_id>/report/index.html` を開けます。 |
| 右の `Motion Log` | 速度、加速度、操舵角、目標速度、制御指令をグラフ表示できます。 |

## 追加された主な機能

### 1. Run Settings

ヘッダーの `Run Settings` から、走行時のAWSIMや評価の起動条件をまとめて
指定できます。

| タブ | 内容 |
| --- | --- |
| `Simulator` | AWSIMの起動方法、周回数、タイムアウト、センサ描画、NPC車両などを指定します。 |
| `Safety Gate` | 障害物停止、追い越し、車線維持などの安全確認シナリオを選びます。 |
| `Multiplay` | AWSIMのマルチプレイ用アドレス、ポート、名前、送信Hzを指定します。 |

GUIで入力した値は、ROS 2のlaunchへ直接書き込むのではなく、多くの場合は
環境変数や `AWSIM_EXTRA_ARGS` として `make dev` / `make eval` に渡されます。

### 2. AWSIMヘッドレス

`AWSIMヘッドレス` を有効にすると、AWSIMを起動したまま画面描画と重いセンサ
描画を抑えます。

内部的には次のようなAWSIM引数が渡されます。

```text
-batchmode -nographics --camera false --lidar false
```

ROS 2的には、完全にシミュレータを止める機能ではありません。`/clock` など
シミュレーションに必要なデータは出すけれど、画面や重い描画を減らすための
設定です。

使う前に、AI Challenge本体側へヘッドレス連携パッチを適用します。

```bash
tools/scripts/setup.sh --apply
```

### 3. 追加Autoware車両

`追加Autoware車両` は、自車以外に起動するAutoware車両の数です。

| GUI表示 | 実行されるターゲット |
| --- | --- |
| `0台` | `make dev` |
| `1台` | `make dev2` |
| `2台` | `make dev3` |
| `3台` | `make dev4` |

評価時は `AWSIM_VEHICLES=1〜4` として渡されます。ここでの数字は「合計車両数」
なので、追加0台なら1、自車+追加3台なら4です。

### 4. Safety Gate

右側の `gate` ボタンは、選択した `Safety Gate` シナリオに応じて
`make gate1`〜`make gate3` を実行します。

| gate | 目的 |
| --- | --- |
| `gate1` | 障害物の手前で止まれるかを見る確認です。 |
| `gate2` | 追い越し系のシナリオ確認です。 |
| `gate3` | 車線維持や安全な走行を確認します。 |

ROS 2初心者向けに言うと、これは「本番評価の前に、特定のシーンだけを切り出して
制御が危なくないか見るボタン」です。

### 5. Simulator 詳細設定

`Simulator` タブでは、AWSIMへ渡す細かい起動オプションをGUIから指定できます。

| 項目 | 何に効くか |
| --- | --- |
| `start` / `count sec` | AWSIM開始の同期やカウントダウンに使います。 |
| `laps` | 周回数を指定します。 |
| `timeout` | 最大実行時間を指定します。 |
| `NPC車両` | AWSIM内のNPC車両数です。 |
| `boosts` | AWSIM側のブースト数です。 |
| `camera` / `LiDAR` | センサ描画や重いセンサ処理の有無を切り替えます。 |
| `sound` | 音を出すかどうかです。 |
| `collision` | 衝突判定の扱いを変えます。 |
| `wall rec.` | 壁接触からの復帰機能を使うかどうかです。 |
| `ranking` | AWSIM側ランキング表示などの制御です。 |
| `steer` | AWSIMへ渡す操舵入力の種類を指定します。 |
| `manual` | 手動操作モードの有無です。 |
| `scenario path` | Safety Gateなどで使うシナリオファイルです。 |
| `vehicle poses` | 車両初期位置のファイルです。 |
| `replay0` | リプレイ入力に使うファイルです。 |
| `json path` | AWSIM追加設定JSONです。 |
| `raw args` | GUIにないAWSIM引数を直接追加します。 |

`raw args` は便利ですが、GUIが管理している `--camera`、`--laps`、
`--scenario` などと重複すると起動前にエラーになります。二重指定で挙動が
読めなくなるのを防ぐためです。

### 6. Multiplay

`Multiplay` は、AWSIMのマルチプレイ起動引数をGUIで組み立てる機能です。

| 項目 | 渡される引数 |
| --- | --- |
| `mode` | `--multiplay` |
| `address` | `--multiplay-address` |
| `port` | `--multiplay-port` |
| `name` | `--multiplay-name` |
| `send Hz` | `--multiplay-send-hz` |

### 7. evalwrap / quick eval / log化

| ボタン | 役割 |
| --- | --- |
| `evalwrap` | `tools/evalwrap run --label ...` を実行し、評価、成果物収集、HTMLレポート生成まで行います。 |
| `quick eval` | 軽く確認したいとき用に直接 `make eval` を実行します。 |
| `log化` | 既にある `output/latest` を `tools/evalwrap ingest` で `analysis/runs/` に取り込みます。 |

`evalwrap` は、評価の結果を `analysis/runs/<run_id>/` にまとめます。
ROS 2初心者向けには「rosbagや公式JSONをあとから見やすいレポートへ変換する係」
と考えるとわかりやすいです。

### 8. Motion Log

`evalwrap` または `log化` のあと、`Motion Log` パネルで走行ログを見られます。

表示元は主に次のファイルです。

```text
analysis/runs/<run_id>/processed/motion_log.csv
```

`motion_log.csv` がない場合でも、次のファイルから近い時刻のデータを組み合わせて
表示できます。

```text
analysis/runs/<run_id>/processed/vehicle_timeseries.csv
analysis/runs/<run_id>/processed/control_timeseries.csv
```

グラフでは、実速度、加速度、実操舵角、目標速度、指令加速度、指令操舵角を
確認できます。制御が「目標に対して遅れているか」「急に操舵しすぎていないか」
を見る入口になります。

## ファイル編集まわりの変更点

### 表編集

YAML/XMLは、直接テキストで読むのがしんどいので、表形式でも編集できます。

| 列 | 意味 |
| --- | --- |
| `line` | 元ファイルのおおよその行番号です。 |
| `path` | YAML内のキー階層、またはXML要素の位置です。 |
| `name` | XMLの `name` 属性など、よく使う名前です。 |
| `type` | 値の型やXML要素の種類です。 |
| `description` | GUI内だけに保存される説明です。元ファイルのコメントには書きません。 |
| `value` | 実際に編集する値です。 |

`表を本文に反映` を押すと、表で変えた値がファイル本文へ反映されます。
保存前にはYAML/XML/JSON/CSVの検証が走ります。

### テキスト編集

CSV、コメント、大きな構造変更、GUIが表にしきれない変更はテキスト編集を使います。
表編集より自由ですが、壊れた構文も書けるので、保存前の `検証` が大事です。

### バックアップ

保存時には、元ファイルが次の場所へバックアップされます。

```text
tools/tuning_gui/backups/
```

「保存したら動かなくなった」時に戻れるようにするための保険です。

## Path Editor の変更点

`Path Editor` は、MPCが追従する参照経路CSVをマップ上で編集する機能です。

### できること

- 現在有効なMPC参照経路を読み込みます。
- 占有グリッドマップ上に経路点を表示します。
- 経路点を移動、追加、削除できます。
- 経路を滑らかにできます。
- 選択範囲だけをまとめて移動、平滑化できます。
- 保存時に `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` を再計算します。
- `configへ反映` を有効にすると、MPCの `reference_path.csv_path` も更新します。
- `保存後ビルド` を有効にすると、保存後に `make autoware-build` を実行します。

### 操作

| 操作 | 意味 |
| --- | --- |
| 点をドラッグ | 1点を移動します。 |
| 空白をドラッグ | 矩形で範囲選択します。 |
| 選択済みの点や線分をドラッグ | 選択範囲全体を移動します。 |
| `add` | 点を追加するモードです。 |
| `delete` | 選択点を削除します。 |
| `smooth` | 選択範囲、または全体を平滑化します。 |
| `undo` | 直前の平滑化前に戻します。 |
| `clear` | 選択範囲を解除します。 |
| 中クリック / Altドラッグ / Shiftドラッグ | マップをパンします。 |
| `fit` | 経路全体が見えるように表示を合わせます。 |

保存先は `multi_purpose_mpc_ros/env` または `multi_purpose_mpc_ros/maps` 配下だけに
制限されています。最初に保存するときのデフォルト名は `<元名>_manual.csv` です。

## 実行履歴とスナップショット

`dev`、`evalwrap`、`quick eval`、`log化`、`gate`、`build` などを実行すると、
GUIは実行履歴を保存します。

```text
tools/tuning_gui/history/
tools/tuning_gui/history/snapshots/
tools/tuning_gui/runtime/commands/
```

履歴には、実行したコマンド、選んだ `control_method`、ログパス、実行時点の
設定ファイルスナップショットなどが入ります。

## これまでの変更で便利になったこと

| 以前 | 今 |
| --- | --- |
| XML/YAML/CSVを探して手で開く必要がありました。 | GUIの `Files` から関連ファイルを開けます。 |
| `control_method` のlaunch設定を手で追う必要がありました。 | GUIで選び、XMLへ保存できます。 |
| AWSIMヘッドレスやNPC台数の環境変数を手で組む必要がありました。 | `Run Settings` と `追加Autoware車両` から指定できます。 |
| Safety Gateを個別コマンドで覚える必要がありました。 | `gate` ボタンとシナリオ選択で実行できます。 |
| 評価後の `output/latest` を手で見に行く必要がありました。 | `evalwrap` / `log化` で `analysis/runs/` に整理できます。 |
| 速度や操舵の時系列を見るにはCSVを開く必要がありました。 | `Motion Log` でグラフ確認できます。 |
| 参照経路CSVの編集がつらい作業でした。 | `Path Editor` でマップ上から編集できます。 |

## 初心者向けのおすすめ順

1. `control_method` を選ぶ。
2. `Files` で関係するYAML/XMLを確認する。
3. 必要なら `Path Editor` で参照経路を調整する。
4. `dev` で短く走らせる。
5. `log化` で走行ログを `analysis/runs/` に取り込む。
6. `Motion Log` と `Reports` を見る。
7. 良さそうなら `evalwrap` で正式な評価レポートを作る。

最初から全部理解しなくて大丈夫です。ROS 2のlaunch、YAML、topic、rosbagが
それぞれ「起動」「設定」「通信」「録画」だと分かれば、このGUIの見え方はかなり
つかめます。
