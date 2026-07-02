# 03 launch結合手順：AI Challengeで起動できるようにする

## ゴール

この資料では、`simple_delay_aware_control` を AI Challenge の `control_method` から選べるようにする手順を説明します。

最終的には次で起動できる状態を目指します。

```bash
ros2 launch aichallenge_submit_launch aichallenge_submit.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_delay_aware_control
```

互換エイリアスとして、プロンプトで使われていた綴りも通せます。

```bash
ros2 launch aichallenge_submit_launch aichallenge_submit.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_dealay_aware_contorol
```

## launch結合の全体像

AI Challengeの制御起動は、ざっくり次の流れです。

```text
aichallenge_submit.launch.xml
  -> reference.launch.xml
    -> launch/control/<control_method>.launch.xml
      -> 各制御パッケージのlaunch
```

`simple_delay_aware_control` では次のファイルを使います。

```text
aichallenge_submit_launch/
  launch/control/simple_delay_aware_control.launch.xml
  launch/reference.launch.xml
  package.xml

simple_delay_aware_control/
  launch/simple_delay_aware_control.launch.xml
  config/simple_delay_aware_control.param.yaml
```

このリポジトリでは、以下の手順はすでに反映済みです。学習者が別ブランチや白紙状態から結合する場合の再現手順として読んでください。

## Step 1: 制御パッケージ側のlaunchを作る

まず、パッケージ内に次を用意します。

```text
aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/launch/simple_delay_aware_control.launch.xml
```

役割は、ノード起動、パラメータ読み込み、topic remapです。

```xml
<node pkg="simple_delay_aware_control" exec="simple_delay_aware_control_node"
      name="simple_delay_aware_control_node" output="screen">
  <param name="use_sim_time" value="$(var use_sim_time)"/>
  <param from="$(var param_path)"/>

  <remap from="input/kinematics" to="/localization/kinematic_state"/>
  <remap from="input/trajectory" to="/planning/scenario_planning/trajectory"/>
  <remap from="input/control_cmd" to="/control/command/control_cmd_raw"/>
  <remap from="output/kinematics" to="/simple_delay_aware_control/localization/kinematic_state"/>
  <remap from="output/control_cmd" to="/control/command/control_cmd"/>
  <remap from="output/raw_control_cmd" to="/control/command/control_cmd_raw"/>
</node>
```

特に大事なのは `output/kinematics` です。

```text
output/kinematics
  -> /simple_delay_aware_control/localization/kinematic_state
```

これにより、delay-aware MPC と同じように「補償済みOdometry」を別topicで確認できます。

`input/control_cmd` も重要です。`delay_aware_mpc_ros` と同じように、目標ステアを `AckermannControlCommand` から受け取って遅延予測に使います。

## Step 2: aichallenge_submit_launch側のwrapperを作る

次に、AI Challenge共通launchから呼ぶwrapperを作ります。

```text
aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/simple_delay_aware_control.launch.xml
```

中身は薄い include です。

```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <arg name="simulation" default="true"/>
  <arg name="use_sim_time" default="true"/>

  <include file="$(find-pkg-share simple_delay_aware_control)/launch/simple_delay_aware_control.launch.xml">
    <arg name="simulation" value="$(var simulation)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>
</launch>
```

この wrapper を挟むと、`reference.launch.xml` 側は他の制御方式と同じ形で扱えます。

## Step 3: reference.launch.xmlへcontrol_method分岐を足す

`reference.launch.xml` に control method の分岐を追加します。

対象ファイル:

```text
aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml
```

追加する分岐:

```xml
<group if="$(eval &quot;'$(var control_method)' == 'simple_delay_aware_control'&quot;)">
  <include file="$(find-pkg-share aichallenge_submit_launch)/launch/control/simple_delay_aware_control.launch.xml">
    <arg name="simulation" value="$(var simulation)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>
</group>
```

プロンプトで使った綴りも受ける場合は、同じinclude先でエイリアスを足します。

```xml
<group if="$(eval &quot;'$(var control_method)' == 'simple_dealay_aware_contorol'&quot;)">
  <include file="$(find-pkg-share aichallenge_submit_launch)/launch/control/simple_delay_aware_control.launch.xml">
    <arg name="simulation" value="$(var simulation)"/>
    <arg name="use_sim_time" value="$(var use_sim_time)"/>
  </include>
</group>
```

description にも候補を足しておくと、後から見つけやすくなります。

```xml
<arg name="control_method" default="delay_aware_mpc"
     description="Select control: mpc, delay_aware_mpc, simple_delay_aware_control, pure_pursuit, tiny_lidar_net, pilot_net, joycon"/>
```

## Step 4: package.xmlへ依存を足す

`aichallenge_submit_launch/package.xml` に実行依存を追加します。

```xml
<exec_depend>simple_delay_aware_control</exec_depend>
```

これを忘れると、提出パッケージやDocker内の依存解決で見落としやすくなります。

## Step 5: tuning_guiのカタログへ追加する

GUIで教材ファイルを開きやすくする場合は、`tools/tuning_gui/app.py` の `CATALOG` に追加します。

登録する代表ファイル:

```text
aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/config/simple_delay_aware_control.param.yaml
aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/control_core.cpp
aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/control_core_exercise.cpp
aichallenge/workspace/src/aichallenge_submit/simple_delay_aware_control/src/simple_delay_aware_control_node.cpp
```

注意点として、GUIに出ることと、実行時に使われることは別です。

実行に効くかは、必ず `control_method` 分岐とlaunch remapで確認します。

## Step 6: ビルドする

Docker内でビルドする例です。

```bash
cd aichallenge-racingkart
CMD='source /opt/ros/humble/setup.bash && source /autoware/install/setup.bash && cd /aichallenge/workspace && colcon build --packages-select simple_delay_aware_control aichallenge_submit_launch' \
  docker compose run -T --rm --no-deps autoware-command
```

テストも実行します。

```bash
cd aichallenge-racingkart
CMD='source /opt/ros/humble/setup.bash && source /autoware/install/setup.bash && cd /aichallenge/workspace && colcon test --packages-select simple_delay_aware_control --event-handlers console_direct+' \
  docker compose run -T --rm --no-deps autoware-command
```

一時的な clean build で確認したい場合は、`--build-base` と `--install-base` を `/tmp` に向けます。

```bash
colcon build \
  --packages-select simple_delay_aware_control aichallenge_submit_launch \
  --build-base /tmp/simple_delay_aware_control_build \
  --install-base /tmp/simple_delay_aware_control_install
```

## Step 7: launch解決を確認する

起動前に、launchが解決できるか確認します。

通常のworkspaceへビルドした場合:

```bash
source install/setup.bash
ros2 launch aichallenge_submit_launch reference.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_delay_aware_control --show-args
```

`--install-base /tmp/simple_delay_aware_control_install` を使った場合:

```bash
source /tmp/simple_delay_aware_control_install/setup.bash
ros2 launch aichallenge_submit_launch reference.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_delay_aware_control --show-args
```

エイリアスも確認する場合:

```bash
ros2 launch aichallenge_submit_launch reference.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_dealay_aware_contorol --show-args
```

## Step 8: devで起動する

通常のAI Challenge起動では、環境変数 `CONTROL_METHOD` を使えます。

```bash
cd aichallenge-racingkart
CONTROL_METHOD=simple_delay_aware_control make dev
```

手でlaunchする場合:

```bash
ros2 launch aichallenge_submit_launch aichallenge_submit.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_delay_aware_control
```

## Step 9: topicを確認する

起動後、まず補償済みodometryを確認します。

```bash
ros2 topic info /simple_delay_aware_control/localization/kinematic_state
ros2 topic echo /simple_delay_aware_control/localization/kinematic_state
```

debugも確認します。

```bash
ros2 topic echo /simple_delay_aware_control/debug
```

制御指令も確認します。

```bash
ros2 topic echo /control/command/control_cmd
```

## Step 10: 提出tarballへ入るか確認する

評価用に提出tarballを作る場合は、通常の手順を使います。

```bash
cd aichallenge-racingkart
./create_submit_file.bash
```

tarballに新パッケージが入っているか確認します。

```bash
tar tzf submit/aichallenge_submit.tar.gz | grep simple_delay_aware_control | head
```

launch wrapperも確認します。

```bash
tar tzf submit/aichallenge_submit.tar.gz | grep 'launch/control/simple_delay_aware_control.launch.xml'
```

## 回帰チェックリスト

```text
[ ] simple_delay_aware_control が colcon build できる
[ ] aichallenge_submit_launch が colcon build できる
[ ] test_control_core が通る
[ ] control_method:=simple_delay_aware_control の --show-args が通る
[ ] /simple_delay_aware_control/localization/kinematic_state がpublishされる
[ ] /simple_delay_aware_control/debug がpublishされる
[ ] /control/command/control_cmd がpublishされる
[ ] make devで走行開始できる
[ ] create_submit_file.bash後のtarballに新パッケージが入っている
```

## よくある詰まり

### Package not found

確認するもの:

```text
colcon build後に install/setup.bash を source しているか
aichallenge_submit_launch/package.xml に exec_depend があるか
Docker外でAutoware依存を探していないか
```

### control_methodを変えても起動しない

確認するもの:

```text
reference.launch.xml に group if 分岐があるか
wrapper launch のファイル名が正しいか
control_method の綴りが一致しているか
```

### Odometryが出ない

確認するもの:

```text
publish_delay_aware_odometry が true か
/localization/kinematic_state が入力されているか
remap output/kinematics が正しいか
```

### 補償済みOdometryをMPCへ渡したい

`delay_aware_mpc_ros` と同じ考え方で、下流ノードの `/localization/kinematic_state` 入力を次へ remap します。

```text
/simple_delay_aware_control/localization/kinematic_state
```

ただし、既存の `delay_aware_mpc_ros` と同時に使うと二重補償になります。どちらの補償を使うかを1つに決めてください。
