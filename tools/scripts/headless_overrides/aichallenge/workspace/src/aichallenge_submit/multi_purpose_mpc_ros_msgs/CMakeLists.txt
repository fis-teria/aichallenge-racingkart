cmake_minimum_required(VERSION 3.5.0)
project(multi_purpose_mpc_ros_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AckermannControlBoostCommand.msg"
  "msg/BorderCells.msg"
  "msg/CandidateExecutionPoint.msg"
  "msg/CandidateExecutionRequest.msg"
  "msg/CartesianTrajectoryApplicationStatus.msg"
  "msg/ControllerAppliedEnvelope.msg"
  "msg/ControllerAppliedTransportStatus.msg"
  "msg/ControllerCommandEnvelope.msg"
  "msg/ControllerExecutionEnvelope.msg"
  "msg/ControllerExecutionWitness.msg"
  "msg/ControllerGeometry.msg"
  "msg/ControllerBaseTrajectorySnapshot.msg"
  "msg/ControllerSampleKey.msg"
  "msg/ControllerTrackingStatus.msg"
  "msg/ExecutionSweepSample.msg"
  "msg/FreeRunExecutionAck.msg"
  "msg/FreeRunPlanKey.msg"
  "msg/FreeRunSourceKey.msg"
  "msg/MotionAuthorityGrant.msg"
  "msg/OvertakePlan.msg"
  "msg/AuthorizedCartesianTrajectory.msg"
  "msg/AuthorizedCartesianTrajectoryV2.msg"
  "msg/PlanSampleKey.msg"
  "msg/PathConstraints.msg"
  "msg/RecoveryControlCommand.msg"
  "msg/RecoveryPermit.msg"
  "msg/RecoveryStatus.msg"
  "msg/RecoveryTrajectory.msg"
  "msg/SafetyStopStatus.msg"
  "msg/SafetyConstraint.msg"
  "msg/StateLatticeControlCommand.msg"
  "msg/StateLatticeV2BindingStatus.msg"
  "msg/StateLatticeV2BaseAttestation.msg"
  "msg/StateLatticeV2FinalFence.msg"
  "msg/StateLatticeV2Identity.msg"
  "msg/StateLatticeV2QuiesceRequest.msg"
  DEPENDENCIES autoware_auto_control_msgs
               autoware_auto_planning_msgs
               builtin_interfaces
               geometry_msgs
               std_msgs
)

ament_auto_package()
