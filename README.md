# Maintenance Güncellemeleri
Değişiklikleri bu dosyalar içinde yaptım (Tamamı klasör sınırından dolayı yüklenemedi):

"controller" klasörünün dizini: /catkin_ws/src/universal_robot/ur_gazebo/controller

"launch" kaynağı dizini: catkin_ws/src/universal_robot/ur_gazebo/launch

"ur3_sim_description" dizini: catkin_ws/src/UR3_sim/ur3_sim_description

"urdf" dizini: catkin_ws/src/universal_robot/ur_description/urdf

"ur3_sim_gazebo" dizini: catkin_ws/src/UR3_sim/ur3_sim_gazebo

roslaunch ur3_sim simulation.launch
... logging to /home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/roslaunch-ubuntu7-2092.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

xacro: in-order processing became default in ROS Melodic. You can drop the option.
started roslaunch server http://ubuntu7:37019/

SUMMARY
========

PARAMETERS
 * /arm_controller/action_monitor_rate: 10
 * /arm_controller/constraints/elbow_joint/goal: 0.1
 * /arm_controller/constraints/elbow_joint/trajectory: 0.1
 * /arm_controller/constraints/goal_time: 0.6
 * /arm_controller/constraints/shoulder_lift_joint/goal: 0.1
 * /arm_controller/constraints/shoulder_lift_joint/trajectory: 0.1
 * /arm_controller/constraints/shoulder_pan_joint/goal: 0.1
 * /arm_controller/constraints/shoulder_pan_joint/trajectory: 0.1
 * /arm_controller/constraints/stopped_velocity_tolerance: 0.05
 * /arm_controller/constraints/wrist_1_joint/goal: 0.1
 * /arm_controller/constraints/wrist_1_joint/trajectory: 0.1
 * /arm_controller/constraints/wrist_2_joint/goal: 0.1
 * /arm_controller/constraints/wrist_2_joint/trajectory: 0.1
 * /arm_controller/constraints/wrist_3_joint/goal: 0.1
 * /arm_controller/constraints/wrist_3_joint/trajectory: 0.1
 * /arm_controller/joints: ['shoulder_pan_jo...
 * /arm_controller/state_publish_rate: 25
 * /arm_controller/stop_trajectory_duration: 0.5
 * /arm_controller/type: position_controll...
 * /gazebo/enable_ros_network: True
 * /gripper/action_monitor_rate: 10
 * /gripper/constraints/goal_time: 0.6
 * /gripper/constraints/left_gear_joint/goal: 0.1
 * /gripper/constraints/left_gear_joint/trajectory: 0.1
 * /gripper/constraints/stopped_velocity_tolerance: 0.05
 * /gripper/joint: left_gear_joint
 * /gripper/state_publish_rate: 25
 * /gripper/stop_trajectory_duration: 0.5
 * /gripper/type: position_controll...
 * /joint_group_position_controller/joints: ['shoulder_pan_jo...
 * /joint_group_position_controller/type: position_controll...
 * /joint_state_controller/publish_rate: 50
 * /joint_state_controller/type: joint_state_contr...
 * /move_group/allow_trajectory_execution: True
 * /move_group/capabilities: move_group/MoveGr...
 * /move_group/controller_list: [{'name': '', 'ac...
 * /move_group/endeffector/planner_configs: ['SBLkConfigDefau...
 * /move_group/jiggle_fraction: 0.05
 * /move_group/manipulator/longest_valid_segment_fraction: 0.01
 * /move_group/manipulator/planner_configs: ['SBLkConfigDefau...
 * /move_group/max_range: 5.0
 * /move_group/max_safe_path_cost: 1
 * /move_group/moveit_controller_manager: moveit_simple_con...
 * /move_group/moveit_manage_controllers: True
 * /move_group/octomap_resolution: 0.025
 * /move_group/planner_configs/BKPIECEkConfigDefault/border_fraction: 0.9
 * /move_group/planner_configs/BKPIECEkConfigDefault/failed_expansion_score_factor: 0.5
 * /move_group/planner_configs/BKPIECEkConfigDefault/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/BKPIECEkConfigDefault/range: 0.0
 * /move_group/planner_configs/BKPIECEkConfigDefault/type: geometric::BKPIECE
 * /move_group/planner_configs/ESTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/ESTkConfigDefault/range: 0.0
 * /move_group/planner_configs/ESTkConfigDefault/type: geometric::EST
 * /move_group/planner_configs/KPIECEkConfigDefault/border_fraction: 0.9
 * /move_group/planner_configs/KPIECEkConfigDefault/failed_expansion_score_factor: 0.5
 * /move_group/planner_configs/KPIECEkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/KPIECEkConfigDefault/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/KPIECEkConfigDefault/range: 0.0
 * /move_group/planner_configs/KPIECEkConfigDefault/type: geometric::KPIECE
 * /move_group/planner_configs/LBKPIECEkConfigDefault/border_fraction: 0.9
 * /move_group/planner_configs/LBKPIECEkConfigDefault/min_valid_path_fraction: 0.5
 * /move_group/planner_configs/LBKPIECEkConfigDefault/range: 0.0
 * /move_group/planner_configs/LBKPIECEkConfigDefault/type: geometric::LBKPIECE
 * /move_group/planner_configs/PRMkConfigDefault/max_nearest_neighbors: 10
 * /move_group/planner_configs/PRMkConfigDefault/type: geometric::PRM
 * /move_group/planner_configs/PRMstarkConfigDefault/type: geometric::PRMstar
 * /move_group/planner_configs/RRTConnectkConfigDefault/range: 0.0
 * /move_group/planner_configs/RRTConnectkConfigDefault/type: geometric::RRTCon...
 * /move_group/planner_configs/RRTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/RRTkConfigDefault/range: 0.0
 * /move_group/planner_configs/RRTkConfigDefault/type: geometric::RRT
 * /move_group/planner_configs/RRTstarkConfigDefault/delay_collision_checking: 1
 * /move_group/planner_configs/RRTstarkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/RRTstarkConfigDefault/range: 0.0
 * /move_group/planner_configs/RRTstarkConfigDefault/type: geometric::RRTstar
 * /move_group/planner_configs/SBLkConfigDefault/range: 0.0
 * /move_group/planner_configs/SBLkConfigDefault/type: geometric::SBL
 * /move_group/planner_configs/TRRTkConfigDefault/frountierNodeRatio: 0.1
 * /move_group/planner_configs/TRRTkConfigDefault/frountier_threshold: 0.0
 * /move_group/planner_configs/TRRTkConfigDefault/goal_bias: 0.05
 * /move_group/planner_configs/TRRTkConfigDefault/init_temperature: 10e-6
 * /move_group/planner_configs/TRRTkConfigDefault/k_constant: 0.0
 * /move_group/planner_configs/TRRTkConfigDefault/max_states_failed: 10
 * /move_group/planner_configs/TRRTkConfigDefault/min_temperature: 10e-10
 * /move_group/planner_configs/TRRTkConfigDefault/range: 0.0
 * /move_group/planner_configs/TRRTkConfigDefault/temp_change_factor: 2.0
 * /move_group/planner_configs/TRRTkConfigDefault/type: geometric::TRRT
 * /move_group/planning_plugin: ompl_interface/OM...
 * /move_group/planning_scene_monitor/publish_geometry_updates: True
 * /move_group/planning_scene_monitor/publish_planning_scene: True
 * /move_group/planning_scene_monitor/publish_state_updates: True
 * /move_group/planning_scene_monitor/publish_transforms_updates: True
 * /move_group/request_adapters: default_planner_r...
 * /move_group/start_state_max_bounds_error: 0.1
 * /move_group/trajectory_execution/allowed_execution_duration_scaling: 1.2
 * /move_group/trajectory_execution/allowed_goal_duration_margin: 0.5
 * /move_group/trajectory_execution/execution_duration_monitoring: False
 * /move_group/use_controller_manager: False
 * /robot_description: <?xml version="1....
 * /robot_description_kinematics/manipulator/kinematics_solver: kdl_kinematics_pl...
 * /robot_description_kinematics/manipulator/kinematics_solver_attempts: 3
 * /robot_description_kinematics/manipulator/kinematics_solver_search_resolution: 0.005
 * /robot_description_kinematics/manipulator/kinematics_solver_timeout: 0.005
 * /robot_description_planning/joint_limits/elbow_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/elbow_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/elbow_joint/max_acceleration: 3.15
 * /robot_description_planning/joint_limits/elbow_joint/max_velocity: 3.15
 * /robot_description_planning/joint_limits/left_gear_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/left_gear_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/left_gear_joint/max_acceleration: 3.2
 * /robot_description_planning/joint_limits/left_gear_joint/max_position: 1.34365
 * /robot_description_planning/joint_limits/left_gear_joint/max_velocity: 3.2
 * /robot_description_planning/joint_limits/left_gear_joint/min_position: 0.0
 * /robot_description_planning/joint_limits/shoulder_lift_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/shoulder_lift_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/shoulder_lift_joint/max_acceleration: 3.15
 * /robot_description_planning/joint_limits/shoulder_lift_joint/max_velocity: 3.15
 * /robot_description_planning/joint_limits/shoulder_pan_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/shoulder_pan_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/shoulder_pan_joint/max_acceleration: 3.15
 * /robot_description_planning/joint_limits/shoulder_pan_joint/max_velocity: 3.15
 * /robot_description_planning/joint_limits/wrist_1_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/wrist_1_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/wrist_1_joint/max_acceleration: 3.2
 * /robot_description_planning/joint_limits/wrist_1_joint/max_velocity: 3.2
 * /robot_description_planning/joint_limits/wrist_2_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/wrist_2_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/wrist_2_joint/max_acceleration: 3.2
 * /robot_description_planning/joint_limits/wrist_2_joint/max_velocity: 3.2
 * /robot_description_planning/joint_limits/wrist_3_joint/has_acceleration_limits: True
 * /robot_description_planning/joint_limits/wrist_3_joint/has_velocity_limits: True
 * /robot_description_planning/joint_limits/wrist_3_joint/max_acceleration: 3.2
 * /robot_description_planning/joint_limits/wrist_3_joint/max_velocity: 3.2
 * /robot_description_semantic: <?xml version="1....
 * /robot_state_publisher/publish_frequency: 50.0
 * /robot_state_publisher/tf_prefix: 
 * /rosdistro: noetic
 * /rosversion: 1.16.0
 * /rviz_ubuntu7_2092_8292307145150743386/manipulator/kinematics_solver: kdl_kinematics_pl...
 * /rviz_ubuntu7_2092_8292307145150743386/manipulator/kinematics_solver_attempts: 3
 * /rviz_ubuntu7_2092_8292307145150743386/manipulator/kinematics_solver_search_resolution: 0.005
 * /rviz_ubuntu7_2092_8292307145150743386/manipulator/kinematics_solver_timeout: 0.005
 * /use_sim_time: True

NODES
  /
    arm_controller_spawner (controller_manager/controller_manager)
    fake_joint_calibration (rostopic/rostopic)
    gazebo (gazebo_ros/gzserver)
    gripper_control (ur3_sim/gripper.py)
    gripper_controller_spawner (controller_manager/spawner)
    joint_state_controller_spawner (controller_manager/controller_manager)
    move_group (moveit_ros_move_group/move_group)
    robot_state_publisher (robot_state_publisher/robot_state_publisher)
    ros_control_controller_manager (controller_manager/controller_manager)
    rviz_ubuntu7_2092_8292307145150743386 (rviz/rviz)
    spawn_gazebo_model (gazebo_ros/spawn_model)

auto-starting new master
process[master]: started with pid [2104]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 73747b4c-ddb5-11ed-9485-cb205e015775
process[rosout-1]: started with pid [2114]
started core service [/rosout]
process[gazebo-2]: started with pid [2117]
process[spawn_gazebo_model-3]: started with pid [2119]
process[robot_state_publisher-4]: started with pid [2123]
process[fake_joint_calibration-5]: started with pid [2124]
process[joint_state_controller_spawner-6]: started with pid [2128]
process[arm_controller_spawner-7]: started with pid [2130]
process[gripper_controller_spawner-8]: started with pid [2131]
process[ros_control_controller_manager-9]: started with pid [2132]
process[move_group-10]: started with pid [2133]
process[rviz_ubuntu7_2092_8292307145150743386-11]: started with pid [2134]
process[gripper_control-12]: started with pid [2135]
[INFO] [1681800701.922815, 0.000000]: Waiting for /clock to be available...
[ WARN] [1681800702.417123531]: Falling back to using the move_group node's namespace (deprecated Melodic behavior).
[ INFO] [1681800702.510502490]: Loading robot model 'ur3'...
[INFO] [1681800705.467796, 0.000000]: Loading model XML from ros parameter robot_description
[INFO] [1681800705.579153, 0.000000]: Waiting for service /gazebo/spawn_urdf_model
[ INFO] [1681800707.528921164]: rviz version 1.14.19
[ INFO] [1681800707.552199277]: compiled against Qt version 5.12.8
[ INFO] [1681800707.552237049]: compiled against OGRE version 1.9.0 (Ghadamon)
[ INFO] [1681800708.021495744]: Forcing OpenGl version 0.
[ WARN] [1681800710.608040674]: Kinematics solver doesn't support #attempts anymore, but only a timeout.
Please remove the parameter '/robot_description_kinematics/manipulator/kinematics_solver_attempts' from your configuration.
[ INFO] [1681800713.273654808]: Stereo is NOT SUPPORTED
[ INFO] [1681800713.281143863]: OpenGL device: llvmpipe (LLVM 12.0.0, 128 bits)
[ INFO] [1681800713.281486189]: OpenGl version: 3,1 (GLSL 1,4).
Start
[ INFO] [1681800715.078838645]: Loading robot model 'ur3'...
[ INFO] [1681800716.606877651]: Publishing maintained planning scene on 'monitored_planning_scene'
[ INFO] [1681800716.611751233]: Listening to 'joint_states' for joint states
[ INFO] [1681800716.621910004]: Listening to '/attached_collision_object' for attached collision objects
[ INFO] [1681800716.623230928]: Starting planning scene monitor
[ INFO] [1681800716.628137125]: Listening to '/planning_scene'
[ INFO] [1681800716.628384654]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[ INFO] [1681800716.670592302]: Listening to '/collision_object'
[ INFO] [1681800716.676822298]: Listening to '/planning_scene_world' for planning scene world geometry
[ INFO] [1681800716.679098648]: No 3D sensor plugin(s) defined for octomap updates
[ WARN] [1681800718.685931641]: Kinematics solver doesn't support #attempts anymore, but only a timeout.
Please remove the parameter '/robot_description_kinematics/manipulator/kinematics_solver_attempts' from your configuration.
[ INFO] [1681800720.117238810]: Finished loading Gazebo ROS API Plugin.
[ INFO] [1681800720.140268383]: Loading planning pipeline ''
[ INFO] [1681800720.186666107]: waitForService: Service [/gazebo/set_physics_properties] has not been advertised, waiting...
[ INFO] [1681800722.078878659]: Using planning interface 'OMPL'
[ INFO] [1681800722.291638930]: Param 'default_workspace_bounds' was not set. Using default value: 10
[ INFO] [1681800722.296701219]: Param 'start_state_max_bounds_error' was set to 0.1
[ INFO] [1681800722.313008836]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1681800722.349672576]: Param 'start_state_max_dt' was not set. Using default value: 0.5
[ INFO] [1681800722.389096216]: Param 'jiggle_fraction' was set to 0.05
[ INFO] [1681800722.397798770]: Param 'max_sampling_attempts' was not set. Using default value: 100
[ INFO] [1681800722.398026259]: Using planning request adapter 'Add Time Parameterization'
[ INFO] [1681800722.398101587]: Using planning request adapter 'Fix Workspace Bounds'
[ INFO] [1681800722.404841145]: Using planning request adapter 'Fix Start State Bounds'
[ INFO] [1681800722.405032062]: Using planning request adapter 'Fix Start State In Collision'
[ INFO] [1681800722.423519565]: Using planning request adapter 'Fix Start State Path Constraints'
[INFO] [1681800730.677731, 0.000000]: Calling service /gazebo/spawn_urdf_model
[ INFO] [1681800731.479101340]: waitForService: Service [/gazebo/set_physics_properties] is now available.
[INFO] [1681800740.826511, 0.000000]: Spawn status: SpawnModel: Successfully spawned entity
[INFO] [1681800740.831475, 0.000000]: Waiting for service /gazebo/set_model_configuration
[INFO] [1681800740.859543, 0.000000]: temporary hack to **fix** the -J joint position option (issue #93), sleeping for 1 second to avoid race condition.
[ INFO] [1681800740.872576019]: Physics dynamic reconfigure ready.
[INFO] [1681800741.865762, 0.000000]: Calling service /gazebo/set_model_configuration
[INFO] [1681800741.891450, 0.000000]: Set model configuration status: SetModelConfiguration: success
[INFO] [1681800741.897358, 0.000000]: Unpausing physics
[ INFO] [1681800742.743373387]: Camera Plugin: Using the 'robotNamespace' param: '/'
[ INFO] [1681800742.769322586]: Camera Plugin (ns = /)  <tf_prefix_>, set to ""
[spawn_gazebo_model-3] process has finished cleanly
log file: /home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/spawn_gazebo_model-3*.log
[ INFO] [1681800743.721577219]: Loading gazebo_ros_control plugin
[ INFO] [1681800743.721938544]: Starting gazebo_ros_control plugin in namespace: /
[ INFO] [1681800743.730797579]: gazebo_ros_control plugin is waiting for model URDF in parameter [robot_description] on the ROS param server.
[ERROR] [1681800743.921586978]: Unsupported Gazebo ImageFormat

[ERROR] [1681800743.943182057]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_pan_joint
[ERROR] [1681800743.954752711]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_lift_joint
[ERROR] [1681800743.957630754]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/elbow_joint
[ERROR] [1681800743.967048492]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_1_joint
[ERROR] [1681800743.971840771]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_2_joint
[ERROR] [1681800743.976035173]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_3_joint
[ERROR] [1681800743.982863809]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/left_gear_joint
[ INFO] [1681800744.100336268]: Loaded gazebo_ros_control.
[ INFO] [1681800744.311299203]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "right_gear_joint", Multiplier: 1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.311750456]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "right_arm_bottom_joint", Multiplier: -1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.314131756]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "left_arm_bottom_joint", Multiplier: 1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.314400517]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "right_arm_top_joint", Multiplier: -1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.314943200]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "left_arm_top_joint", Multiplier: 1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.315109361]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "left_stick_joint", Multiplier: -1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.315391997]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "right_stick_joint", Multiplier: 1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.318148155]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "left_stick_top_joint", Multiplier: -1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
[ INFO] [1681800744.321809642]: MimicJointPlugin loaded! Joint: "left_gear_joint", Mimic joint: "right_stick_top_joint", Multiplier: 1, Offset: 0, MaxEffort: 1000, Sensitiveness: 0
Loaded 'arm_controller'
Loaded 'joint_state_controller'
Loaded 'joint_group_position_controller'
Started ['arm_controller'] successfully
Started ['joint_state_controller'] successfully
[ INFO] [1681800744.862759225, 0.063000000]: Added FollowJointTrajectory controller for 
Traceback (most recent call last):
  File "/home/leopard/catkin_ws/src/UR3_sim/ur3_sim/scripts/gripper.py", line 68, in <module>
    main()
  File "/home/leopard/catkin_ws/src/UR3_sim/ur3_sim/scripts/gripper.py", line 57, in main
    gripper = GripperController()
  File "/home/leopard/catkin_ws/src/UR3_sim/ur3_sim/scripts/gripper.py", line 33, in __init__
    self.move_group = MoveGroupPythonInteface()
  File "/home/leopard/catkin_ws/src/UR3_sim/ur3_sim/scripts/gripper.py", line 16, in __init__
    move_group = moveit_commander.MoveGroupCommander("gripper")
  File "/opt/ros/noetic/lib/python3/dist-packages/moveit_commander/move_group.py", line 66, in __init__
    self._g = _moveit_move_group_interface.MoveGroupInterface(
RuntimeError: Unable to connect to move_group action server 'move_group' within allotted time (5s)
[ros_control_controller_manager-9] process has finished cleanly
log file: /home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/ros_control_controller_manager-9*.log
[INFO] [1681800745.637117, 0.201000]: /clock is published. Proceeding to load the controller(s).
[INFO] [1681800745.640348, 0.203000]: Controller Spawner: Waiting for service controller_manager/load_controller
[joint_state_controller_spawner-6] process has finished cleanly
log file: /home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/joint_state_controller_spawner-6*.log
[arm_controller_spawner-7] process has finished cleanly
log file: /home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/arm_controller_spawner-7*.log
[INFO] [1681800745.651508, 0.208000]: Controller Spawner: Waiting for service controller_manager/switch_controller
[INFO] [1681800745.659810, 0.213000]: Controller Spawner: Waiting for service controller_manager/unload_controller
[INFO] [1681800745.667767, 0.217000]: Loading controller: gripper
[INFO] [1681800745.844823, 0.291000]: Controller Spawner: Loaded controllers: gripper
[INFO] [1681800745.897964, 0.302000]: Started controllers: gripper
[ INFO] [1681800746.088315240, 0.353000000]: Added GripperCommand controller for gripper
[ INFO] [1681800746.088526742, 0.354000000]: Returned 2 controllers in list
[ INFO] [1681800746.137933226, 0.378000000]: Trajectory execution is managing controllers
[ INFO] [1681800746.138313665, 0.378000000]: MoveGroup debug mode is OFF
[gripper_control-12] process has died [pid 2135, exit code 1, cmd /home/leopard/catkin_ws/src/UR3_sim/ur3_sim/scripts/gripper.py __name:=gripper_control __log:=/home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/gripper_control-12.log].
log file: /home/leopard/.ros/log/73747b4c-ddb5-11ed-9485-cb205e015775/gripper_control-12*.log
Loading 'move_group/ApplyPlanningSceneService'...
Loading 'move_group/ClearOctomapService'...
Loading 'move_group/MoveGroupCartesianPathService'...
Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
Loading 'move_group/MoveGroupGetPlanningSceneService'...
Loading 'move_group/MoveGroupKinematicsService'...
Loading 'move_group/MoveGroupMoveAction'...
Loading 'move_group/MoveGroupPickPlaceAction'...
Loading 'move_group/MoveGroupPlanService'...
Loading 'move_group/MoveGroupQueryPlannersService'...
Loading 'move_group/MoveGroupStateValidationService'...
[ INFO] [1681800746.897421387, 0.608000000]: 

********************************************************
* MoveGroup using: 
*     - ApplyPlanningSceneService
*     - ClearOctomapService
*     - CartesianPathService
*     - ExecuteTrajectoryAction
*     - GetPlanningSceneService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
********************************************************

[ INFO] [1681800746.901169566, 0.608000000]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1681800746.902954755, 0.609000000]: MoveGroup context initialization complete

You can start planning now!

[ INFO] [1681800748.583561767, 0.846000000]: Loading robot model 'ur3'...
[ WARN] [1681800749.138036643, 0.874000000]: Kinematics solver doesn't support #attempts anymore, but only a timeout.
Please remove the parameter '/rviz_ubuntu7_2092_8292307145150743386/manipulator/kinematics_solver_attempts' from your configuration.
[ INFO] [1681800750.508636724, 0.920000000]: Starting planning scene monitor
[ INFO] [1681800750.525281736, 0.921000000]: Listening to '/move_group/monitored_planning_scene'
[ INFO] [1681800751.633938191, 1.000000000]: Constructing new MoveGroup connection for group 'manipulator' in namespace ''
[ INFO] [1681800752.825689664, 1.090000000]: Ready to take commands for planning group manipulator.

