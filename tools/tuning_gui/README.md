# AI Challenge Tuning GUI

`control_method` の選択、関連する launch/YAML/CSV ファイルの編集、再ビルド、
`make dev`、`tools/evalwrap run`、または軽量な `make eval` の起動を行うための
ローカルブラウザ UI です。

## 起動

```bash
cd /home/graneple/git/autononous_ai/aichallenge-racingkart
tools/run_tuning_gui.bash --background
```

`http://127.0.0.1:8765` を開きます。

ログをターミナルに表示したまま使いたい場合は、フォアグラウンドモードも使えます。

```bash
tools/run_tuning_gui.bash
```

便利な管理コマンド:

```bash
tools/run_tuning_gui.bash --status
tools/run_tuning_gui.bash --stop
tools/run_tuning_gui.bash --restart
```

## 動作

- control method の一覧は `aichallenge_submit_launch/launch/reference.launch.xml` から読み込みます。
- `XMLへ保存` は XML launch チェーン内の `control_method` デフォルト値を更新します。
- YAML/XML ファイルはデフォルトでテーブル編集モードで開きます。YAML のスカラー値は path/value の行として表示されます。
- テーブルには編集可能な `description` 列があります。説明文は `tools/tuning_gui/.state/parameter_descriptions.json` に保存される GUI メタデータで、YAML/XML のコメントには書き込まれません。
- XML ファイルは要素ごとに1行で表示され、`name`、`default`、`value`、`to`、`from`、`args`、`file` などのよく使う編集可能属性が列にまとめられます。
- テーブルセルを編集して保存すると、GUI は検証前にテーブルの変更をファイル本文へ反映します。
- CSV ファイル、コメント、YAML/XML の構造的な編集、大きな手動変更には、引き続きテキスト編集モードを使えます。
- `Path Editor` は現在有効な MPC 参照経路を占有グリッドマップ上で開きます。経路点の移動、追加、削除、平滑化を行い、再計算した `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` CSV として保存できます。
- 経路平滑化は点数を維持し、保存前に近傍平均のパスを適用します。`undo` は直前の平滑化操作前の点列に戻します。
- `move` モードでは、マップ上の空白部分をドラッグして矩形で点を選択します。選択済みの緑色の点または選択済みセグメントをドラッグすると、選択範囲全体をまとめて移動できます。マップのパンには中クリック、Alt ドラッグ、Shift ドラッグを使えます。
- 点が選択されている場合、`smooth` は選択矩形範囲だけに適用されます。経路全体の平滑化に戻すには `clear` を使います。
- Path Editor は `multi_purpose_mpc_ros/env` または `multi_purpose_mpc_ros/maps` 配下に保存します。元の経路から初めて保存する場合のデフォルト名は `<name>_manual.csv` で、それ以降の手動経路保存では同じ CSV をバックアップ作成後に上書きします。
- ファイル保存時には YAML/XML/JSON/CSV を検証し、タイムスタンプ付きバックアップを `tools/tuning_gui/backups/` に保存します。
- `保存してビルド` と control-method の `保存してビルド` は、編集成功後に `make autoware-build` を開始します。
- `dev` は `CONTROL_METHOD=<selected>` 付きで実行され、先に `make autoware-build` を走らせることもできます。
- `evalwrap` は `CONTROL_METHOD=<selected>` 付きで `tools/evalwrap run --label ...` を実行します。update-build チェックボックスを有効にすると、提出アーカイブの再生成、eval イメージの再ビルド、`make eval` の実行、レポート収集まで行います。
- `quick eval` は、短時間のローカル確認向けに軽量な直接 `make eval` 経路を残しています。こちらも先に `make autoware-build` を実行できます。
- 実行メモは evalwrap の label/note として渡されます。空の場合、GUI は `<control_method>-gui-eval` を使います。
- コマンド実行中はパラメータ編集がロックされます。
- 実行スナップショットとコマンド履歴は `tools/tuning_gui/history/` に保存されます。
