# simple_delay_aware_control

Educational controller for AI Challenge Racing Kart.

This package keeps the same runnable control path as `simple_pure_pursuit`, then adds two beginner-friendly ideas:

1. Predict the pose a short time ahead before calculating steering.
2. Reduce target speed when the upcoming trajectory curvature is large.

It also publishes the delay-compensated control state as `nav_msgs/msg/Odometry`, following the same front-end shape as `delay_aware_mpc_ros`:

```text
/localization/kinematic_state
  -> simple_delay_aware_control_node
  -> /simple_delay_aware_control/localization/kinematic_state
```

For the steering-delay prediction it subscribes to `AckermannControlCommand` on `input/control_cmd`, which is remapped to `/control/command/control_cmd_raw` by the launch file.

Runtime entrypoint:

```bash
ros2 launch aichallenge_submit_launch aichallenge_submit.launch.xml \
  simulation:=true use_sim_time:=true control_method:=simple_delay_aware_control
```

The intentionally incomplete exercise API is in:

```text
include/simple_delay_aware_control/exercise/control_core_exercise.hpp
src/control_core_exercise.cpp
```

The runnable answer implementation is in:

```text
include/simple_delay_aware_control/control_core.hpp
src/control_core.cpp
src/simple_delay_aware_control_node.cpp
```

Japanese teaching docs:

```text
docs/01_implementation_spec.md
docs/02_logic_explanation.md
docs/03_launch_integration_guide.md
```
