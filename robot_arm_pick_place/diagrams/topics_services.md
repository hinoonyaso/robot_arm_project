# Topics / Services / Actions

## Topics
- `/joint_states` (sensor_msgs/JointState)
- `/tf`, `/tf_static`
- `/move_group/feedback` (MoveIt2)

## Services
- `/attach` (std_srvs/Trigger) - enable EE-follow attachment
- `/detach` (std_srvs/Trigger) - disable EE-follow attachment
- `/gazebo/set_entity_state` (gazebo_msgs/SetEntityState)

## Actions
- `/arm_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectory)
- `/gripper_controller/follow_joint_trajectory` (control_msgs/FollowJointTrajectory)
