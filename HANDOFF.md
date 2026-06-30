# Handoff: zivid_artist_bot / MoveIt migration

## Current State
The original execution failure was not just a namespace problem. Planning worked, but execution broke because the stack was loading inconsistent robot models and joint names across source and installed overlays.

The important mismatch was:
- source tree and controller side used `lbr_A1..lbr_A7`
- an installed overlay copy still exposed `iiwa_joint_1..iiwa_joint_7`

That meant MoveIt could plan, but execution and controller matching were unreliable because the trajectory joints and controller joints did not describe the same robot.

## What Changed
The workspace was moved away from a single monolithic custom setup and toward a package split.

Completed changes:
- created `src/iiwa14_description` as the first migration package
- redirected `src/zivid_artist_bot/launch/view_system_launch.py` to use `iiwa14_description`
- updated `src/zivid_artist_bot/launch/test_parse.py` to read the new description package
- updated `src/zivid_artist_bot/package.xml` to depend on `iiwa14_description`
- built `iiwa14_description` and `zivid_artist_bot` successfully after the redirect

## Verified
- `iiwa14_description` builds cleanly on its own
- `zivid_artist_bot` builds cleanly after the launch redirect
- the installed launch files now resolve `iiwa14_description`
- the existing RViz config from upstream `iiwa14_moveit_config` was still the safer config to use because it already carries the correct `Move Group Namespace: lbr`

## Root Cause Summary
The earlier fix focused on namespace wiring, but the runtime failure persisted because the real issue was a split-brain robot definition:
- MoveIt runtime data
- controller config
- URDF/SRDF source files
- installed overlay artifacts

Those were not all describing the same joint naming scheme.

## Recommended Next Step
Return to a clean upstream `lbr_fri_ros2_stack` base, then reapply only the custom pieces in separate layers:
1. description package
2. MoveIt config package
3. app/demo package

This is safer than continuing to patch the current mixed workspace in place.

## Files To Review First
- [src/zivid_artist_bot/launch/view_system_launch.py](src/zivid_artist_bot/launch/view_system_launch.py)
- [src/zivid_artist_bot/config/moveit_controllers.yaml](src/zivid_artist_bot/config/moveit_controllers.yaml)
- [src/iiwa14_description/urdf/iiwa14/iiwa14_description.xacro](src/iiwa14_description/urdf/iiwa14/iiwa14_description.xacro)
- [src/lbr_fri_ros2_stack/lbr_description/urdf/iiwa14/iiwa14_description.xacro](src/lbr_fri_ros2_stack/lbr_description/urdf/iiwa14/iiwa14_description.xacro)
- [src/lbr_fri_ros2_stack/lbr_moveit_config/iiwa14_moveit_config/config/iiwa14.srdf](src/lbr_fri_ros2_stack/lbr_moveit_config/iiwa14_moveit_config/config/iiwa14.srdf)

## Notes For The Next Agent
- Do not assume the controller action path is the main issue; it was only part of the earlier symptom.
- Be careful with source vs install overlay divergence.
- Treat `zivid_artist_bot` as the app layer, not the place to keep shared robot description logic long term.

## Recent terminal output when a move it plan and execution was attempted

mark@mark-ThinkPad-L14-Gen-3:~/ros2_ws$ LIBGL_ALWAYS_SOFTWARE=1 ros2 launch zivid_artist_bot view_system_launch.py
[INFO] [launch]: All log files can be found below /home/mark/.ros/log/2026-06-30-09-49-40-350874-mark-ThinkPad-L14-Gen-3-215462
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [move_group-1]: process started with pid [215464]
[INFO] [rviz2-2]: process started with pid [215466]
[INFO] [robot_state_publisher-3]: process started with pid [215468]
[INFO] [joint_state_publisher-4]: process started with pid [215470]
[rviz2-2] Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome. Use QT_QPA_PLATFORM=wayland to run on Wayland anyway.
[robot_state_publisher-3] [INFO] [1782809380.549179379] [lbr.robot_state_publisher]: got segment lbr_link_0
[robot_state_publisher-3] [INFO] [1782809380.549282733] [lbr.robot_state_publisher]: got segment lbr_link_1
[robot_state_publisher-3] [INFO] [1782809380.549293579] [lbr.robot_state_publisher]: got segment lbr_link_2
[robot_state_publisher-3] [INFO] [1782809380.549301694] [lbr.robot_state_publisher]: got segment lbr_link_3
[robot_state_publisher-3] [INFO] [1782809380.549308347] [lbr.robot_state_publisher]: got segment lbr_link_4
[robot_state_publisher-3] [INFO] [1782809380.549315368] [lbr.robot_state_publisher]: got segment lbr_link_5
[robot_state_publisher-3] [INFO] [1782809380.549321648] [lbr.robot_state_publisher]: got segment lbr_link_6
[robot_state_publisher-3] [INFO] [1782809380.549328001] [lbr.robot_state_publisher]: got segment lbr_link_7
[robot_state_publisher-3] [INFO] [1782809380.549334175] [lbr.robot_state_publisher]: got segment lbr_link_ee
[robot_state_publisher-3] [INFO] [1782809380.549340459] [lbr.robot_state_publisher]: got segment table_link
[robot_state_publisher-3] [INFO] [1782809380.549346836] [lbr.robot_state_publisher]: got segment world
[move_group-1] [INFO] [1782809380.578201460] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00311648 seconds
[move_group-1] [INFO] [1782809380.578275288] [moveit_robot_model.robot_model]: Loading robot model 'iiwa14'...
[move_group-1] [INFO] [1782809380.578284904] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[move_group-1] [INFO] [1782809380.619664258] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[move_group-1] [INFO] [1782809380.619970458] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[move_group-1] [INFO] [1782809380.621009565] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[move_group-1] [INFO] [1782809380.621463927] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/lbr/attached_collision_object' for attached collision objects
[move_group-1] [INFO] [1782809380.621489497] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[move_group-1] [INFO] [1782809380.621761148] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/lbr/planning_scene'
[move_group-1] [INFO] [1782809380.621775958] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[move_group-1] [INFO] [1782809380.622201951] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[move_group-1] [INFO] [1782809380.622489192] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[move_group-1] [WARN] [1782809380.623096667] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[move_group-1] [ERROR] [1782809380.623120543] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[move_group-1] [INFO] [1782809380.625850193] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'pilz_industrial_motion_planner'
[move_group-1] [INFO] [1782809380.629154450] [moveit.pilz_industrial_motion_planner.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[move_group-1] [INFO] [1782809380.632835961] [moveit.pilz_industrial_motion_planner]: Available plugins: pilz_industrial_motion_planner/PlanningContextLoaderCIRC pilz_industrial_motion_planner/PlanningContextLoaderLIN pilz_industrial_motion_planner/PlanningContextLoaderPTP 
[move_group-1] [INFO] [1782809380.632888283] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderCIRC
[move_group-1] [INFO] [1782809380.634196520] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [CIRC]
[move_group-1] [INFO] [1782809380.634232257] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderLIN
[move_group-1] [INFO] [1782809380.635171840] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [LIN]
[move_group-1] [INFO] [1782809380.635190937] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderPTP
[move_group-1] [INFO] [1782809380.635900488] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [PTP]
[move_group-1] [INFO] [1782809380.635926802] [moveit.ros_planning.planning_pipeline]: Using planning interface 'Pilz Industrial Motion Planner'
[move_group-1] [INFO] [1782809380.639065123] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[move_group-1] [INFO] [1782809380.647776601] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[move_group-1] [INFO] [1782809380.650254598] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1782809380.650294555] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1782809380.650298758] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1782809380.650316680] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1782809380.650327936] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1782809380.650330795] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1782809380.650338052] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1782809380.650340669] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was set to 0.050000
[move_group-1] [INFO] [1782809380.650344528] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1782809380.650351130] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1782809380.650353819] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-1] [INFO] [1782809380.650355518] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1782809380.650357246] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1782809380.650358835] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1782809380.650360353] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [INFO] [1782809380.650859352] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'chomp'
[move_group-1] [INFO] [1782809380.653185339] [moveit.ros_planning.planning_pipeline]: Using planning interface 'CHOMP'
[move_group-1] [INFO] [1782809380.654197385] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.path_tolerance' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1782809380.654211722] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.resample_dt' was not set. Using default value: 0.100000
[move_group-1] [INFO] [1782809380.654236912] [moveit_ros.add_time_optimal_parameterization]: Param 'chomp.min_angle_change' was not set. Using default value: 0.001000
[move_group-1] [INFO] [1782809380.654252039] [moveit_ros.fix_workspace_bounds]: Param 'chomp.default_workspace_bounds' was not set. Using default value: 10.000000
[move_group-1] [INFO] [1782809380.654261580] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_bounds_error' was set to 0.100000
[move_group-1] [INFO] [1782809380.654264410] [moveit_ros.fix_start_state_bounds]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1782809380.654271076] [moveit_ros.fix_start_state_collision]: Param 'chomp.start_state_max_dt' was not set. Using default value: 0.500000
[move_group-1] [INFO] [1782809380.654273574] [moveit_ros.fix_start_state_collision]: Param 'chomp.jiggle_fraction' was set to 0.050000
[move_group-1] [INFO] [1782809380.654275865] [moveit_ros.fix_start_state_collision]: Param 'chomp.max_sampling_attempts' was not set. Using default value: 100
[move_group-1] [INFO] [1782809380.654282283] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[move_group-1] [INFO] [1782809380.654284639] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[move_group-1] [INFO] [1782809380.654286278] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[move_group-1] [INFO] [1782809380.654288115] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[move_group-1] [INFO] [1782809380.654290232] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[move_group-1] [INFO] [1782809380.654291810] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[move_group-1] [INFO] [1782809380.681115143] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for joint_trajectory_controller
[move_group-1] [INFO] [1782809380.681519164] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1782809380.681562664] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1782809380.682498658] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
[move_group-1] [INFO] [1782809380.682534931] [move_group.move_group]: MoveGroup debug mode is ON
[move_group-1] [INFO] [1782809380.698024542] [move_group.move_group]: 
[move_group-1] 
[move_group-1] ********************************************************
[move_group-1] * MoveGroup using: 
[move_group-1] *     - ApplyPlanningSceneService
[move_group-1] *     - ClearOctomapService
[move_group-1] *     - CartesianPathService
[move_group-1] *     - ExecuteTrajectoryAction
[move_group-1] *     - GetPlanningSceneService
[move_group-1] *     - KinematicsService
[move_group-1] *     - MoveAction
[move_group-1] *     - MotionPlanService
[move_group-1] *     - QueryPlannersService
[move_group-1] *     - StateValidationService
[move_group-1] ********************************************************
[move_group-1] 
[move_group-1] [INFO] [1782809380.698072453] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[move_group-1] [INFO] [1782809380.698083301] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete
[move_group-1] Loading 'move_group/ApplyPlanningSceneService'...
[move_group-1] Loading 'move_group/ClearOctomapService'...
[move_group-1] Loading 'move_group/MoveGroupCartesianPathService'...
[move_group-1] Loading 'move_group/MoveGroupExecuteTrajectoryAction'...
[move_group-1] Loading 'move_group/MoveGroupGetPlanningSceneService'...
[move_group-1] Loading 'move_group/MoveGroupKinematicsService'...
[move_group-1] Loading 'move_group/MoveGroupMoveAction'...
[move_group-1] Loading 'move_group/MoveGroupPlanService'...
[move_group-1] Loading 'move_group/MoveGroupQueryPlannersService'...
[move_group-1] Loading 'move_group/MoveGroupStateValidationService'...
[move_group-1] 
[move_group-1] You can start planning now!
[move_group-1] 
[rviz2-2] [INFO] [1782809380.878397036] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] [INFO] [1782809380.878518093] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[joint_state_publisher-4] [INFO] [1782809380.893172394] [lbr.joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...
[rviz2-2] [INFO] [1782809380.932931820] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-2] Warning: class_loader.impl: SEVERE WARNING!!! A namespace collision has occurred with plugin factory for class rviz_default_plugins::displays::InteractiveMarkerDisplay. New factory will OVERWRITE existing one. This situation occurs when libraries containing plugins are directly linked against an executable (the one running right now generating this message). Please separate plugins out into their own library or just don't link against the library and use either class_loader::ClassLoader/MultiLibraryClassLoader to open.
[rviz2-2]          at line 253 in /opt/ros/humble/include/class_loader/class_loader/class_loader_core.hpp
[rviz2-2] [ERROR] [1782809384.014568803] [moveit_ros_visualization.motion_planning_frame]: Action server: /recognize_objects not available
[rviz2-2] [INFO] [1782809384.032019695] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> . Reloading params.
[rviz2-2] [INFO] [1782809384.034142528] [moveit_ros_visualization.motion_planning_frame]: MoveGroup namespace changed: / -> lbr. Reloading params.
[rviz2-2] [INFO] [1782809384.147470362] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00300271 seconds
[rviz2-2] [INFO] [1782809384.147521075] [moveit_robot_model.robot_model]: Loading robot model 'iiwa14'...
[rviz2-2] [INFO] [1782809384.147534031] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[rviz2-2] [INFO] [1782809384.213683056] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1782809384.214275452] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1782809384.323667833] [interactive_marker_display_110768591045216]: Connected on namespace: lbr/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
[rviz2-2] [INFO] [1782809384.362376471] [interactive_marker_display_110768591045216]: Sending request for interactive markers
[rviz2-2] [INFO] [1782809384.364576198] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
[rviz2-2] [INFO] [1782809384.371301752] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00211866 seconds
[rviz2-2] [INFO] [1782809384.371347102] [moveit_robot_model.robot_model]: Loading robot model 'iiwa14'...
[rviz2-2] [INFO] [1782809384.371354314] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[rviz2-2] [WARN] [1782809384.407582223] [interactive_marker_display_110768591045216]: Server not available during initialization, resetting
[rviz2-2] [INFO] [1782809384.407653617] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[rviz2-2] [INFO] [1782809384.408307388] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/monitored_planning_scene'
[rviz2-2] [INFO] [1782809384.417747673] [moveit_ros_visualization.motion_planning_frame]: group arm
[rviz2-2] [INFO] [1782809384.417779631] [moveit_ros_visualization.motion_planning_frame]: Constructing new MoveGroup connection for group 'arm' in namespace 'lbr'
[rviz2-2] [INFO] [1782809384.425218158] [move_group_interface]: Ready to take commands for planning group arm.
[rviz2-2] [INFO] [1782809384.426946173] [moveit_ros_visualization.motion_planning_frame]: group arm
[rviz2-2] [INFO] [1782809384.455541472] [interactive_marker_display_110768591045216]: Sending request for interactive markers
[rviz2-2] [INFO] [1782809384.493076192] [interactive_marker_display_110768591045216]: Service response received for initialization
[rviz2-2] [INFO] [1782809392.423467849] [move_group_interface]: MoveGroup action client/server ready
[move_group-1] [INFO] [1782809392.424353938] [moveit_move_group_default_capabilities.move_action_capability]: Received request
[move_group-1] [INFO] [1782809392.424669153] [moveit_move_group_default_capabilities.move_action_capability]: executing..
[rviz2-2] [INFO] [1782809392.424961178] [move_group_interface]: Planning request accepted
[move_group-1] [INFO] [1782809392.501560461] [moveit_move_group_default_capabilities.move_action_capability]: Planning request received for MoveGroup action. Forwarding to planning pipeline.
[move_group-1] [INFO] [1782809392.501607704] [moveit_move_group_capabilities_base.move_group_capability]: Using planning pipeline 'chomp'
[move_group-1] [INFO] [1782809394.013023627] [chomp_planner]: CHOMP trajectory initialized using method: quintic-spline 
[move_group-1] [INFO] [1782809394.013095239] [chomp_optimizer]: Active collision detector is: HYBRID
[move_group-1] [INFO] [1782809394.388948660] [chomp_optimizer]: First coll check took 0.375829 sec
[move_group-1] [INFO] [1782809394.410460024] [chomp_optimizer]: Collision cost 0.128129, smoothness cost: 45.257578
[move_group-1] [INFO] [1782809394.410716267] [chomp_optimizer]: iteration: 0
[move_group-1] [INFO] [1782809394.415055406] [chomp_optimizer]: Chomp Got mesh to mesh safety at iter 0. Breaking out early.
[move_group-1] [INFO] [1782809394.415596584] [chomp_optimizer]: cCost 0.128129 over threshold 0.070000
[move_group-1] [INFO] [1782809394.415985210] [chomp_optimizer]: Chomp path is collision free
[move_group-1] [INFO] [1782809394.416341870] [chomp_optimizer]: Terminated after 1 iterations, using path from iteration 0
[move_group-1] [INFO] [1782809394.416719530] [chomp_optimizer]: Optimization core finished in 0.012469 sec
[move_group-1] [INFO] [1782809394.417052210] [chomp_optimizer]: Time per iteration 0.012801 sec
[move_group-1] [INFO] [1782809394.417387313] [chomp_planner]: Planned with Chomp Parameters (learning_rate, ridge_factor, planning_time_limit, max_iterations), attempt: # 1 
[move_group-1] [INFO] [1782809394.417400729] [chomp_planner]: Learning rate: 0.010000 ridge factor: 0.010000 planning time limit: 10.000000 max_iterations 200 
[move_group-1] [INFO] [1782809394.477303505] [moveit_move_group_default_capabilities.move_action_capability]: Motion plan was computed successfully.
[rviz2-2] [INFO] [1782809394.481983345] [move_group_interface]: Planning request complete!
[rviz2-2] [INFO] [1782809394.482722767] [move_group_interface]: time taken to generate plan: 0.40481 seconds
[move_group-1] [INFO] [1782809402.065977134] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Received goal request
[move_group-1] [INFO] [1782809402.066113714] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution request received
[move_group-1] [INFO] [1782809402.066154456] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1782809402.066178473] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[rviz2-2] [INFO] [1782809402.066234718] [move_group_interface]: Execute request accepted
[move_group-1] [INFO] [1782809402.066328156] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.01
[move_group-1] [INFO] [1782809402.102485726] [moveit_ros.trajectory_execution_manager]: Starting trajectory execution ...
[move_group-1] [INFO] [1782809402.102628652] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [INFO] [1782809402.102648754] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[move_group-1] [ERROR] [1782809402.102752100] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: Action client not connected to action server: joint_trajectory_controller/follow_joint_trajectory
[move_group-1] [ERROR] [1782809402.102789964] [moveit_ros.trajectory_execution_manager]: Failed to send trajectory part 1 of 1 to controller joint_trajectory_controller
[move_group-1] [INFO] [1782809402.102796910] [moveit_ros.trajectory_execution_manager]: Completed trajectory execution with status ABORTED ...
[move_group-1] [INFO] [1782809402.102983347] [moveit_move_group_default_capabilities.execute_trajectory_action_capability]: Execution completed: ABORTED
[rviz2-2] [INFO] [1782809402.103476661] [move_group_interface]: Execute request aborted
[rviz2-2] [ERROR] [1782809402.104481815] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached



