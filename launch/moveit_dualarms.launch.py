#!/usr/bin/python3

import os, sys, xacro, yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import yaml

os.environ["IGN_FUEL_DISABLE_DOWNLOAD"] = "true"

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None
        
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    print("absolute path: ",absolute_file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

# --- Custom Function to read and process XACRO ---
def load_xacro_content(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    # Check if the file exists before parsing
    if not os.path.exists(absolute_file_path):
        print(f"Error: XACRO file not found at {absolute_file_path}")
        return ""
        
    try:
        # Use xacro package to process the XACRO file into a URDF string
        # This replaces the need to manually parse and read the generated URDF
        doc = xacro.process_file(absolute_file_path)
        return doc.toprettyxml(indent='  ')
    except Exception as e:
        print(f"Error processing XACRO file {absolute_file_path}: {e}")
        return ""
# --------------------------------------------------

def generate_launch_description():
    LD = LaunchDescription()
    
    # GAZEBO: world file
    robot_gazebo = "/home/omar-magdy/gp_ws/src/worlds/empty_minimal.world"

    # Bridge Node (optional; uncomment if needed)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/irb120/rrbot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/dual_arms/joint_state', 'joint_states'),
        ],
        output='screen'
    )
    
    # Include Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])]
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {robot_gazebo}',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # ROBOT DESCRIPTION:
    robot_description_path = get_package_share_directory("dual_arms")

    # --- START OF MERGED XACRO LOADING ---
    # Load the new XACRO file that includes the environment.
    xacro_file_name = "dual_arms_with_environment.xacro"
    
    # Use the custom function to process the XACRO content robustly.
    robot_description_config = load_xacro_content("dual_arms", os.path.join('urdf', xacro_file_name))
    
    if not robot_description_config:
        print("FATAL ERROR: XACRO loading failed. Returning empty launch description.")
        return LaunchDescription() # Return empty LD if critical configuration fails

    robot_description = {'robot_description': robot_description_config}
    # --- END OF MERGED XACRO LOADING ---
    
    # Robot State Publisher Node:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {"use_sim_time": True}]
    )
    # NOTE: The static_tf target frame is updated from 'base_link' to 'abb_table' 
    # since the new XACRO likely uses the environment as the root link.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "abb_table"],
    )

    # Spawn Robot in Gazebo:
    spawn_entity = Node(
       package="ros_gz_sim",
       executable="create",
       arguments=["-name", "dual_arms",
                  "-topic", "robot_description",
                  "-x", "0", "-y", "0", "-z", "0"],
       output="both"
    )

      # Controller Manager Nodes
    ros2_controllers = PathJoinSubstitution(
    [
    get_package_share_directory('dual_arms'),
    "config",
    "dual_arms_controller.yaml",
    ]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers,
            {"use_sim_time": True}
        ],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--param-file", ros2_controllers],
    )
    # Controller Manager Spawner node to launch the arm group controller
    ar4_trajectory_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['ar4_trajectory_controller',"--param-file", ros2_controllers],
    output='screen'
    )
        # Controller Manager Spawner node to launch the arm group controller
    irb120_trajectory_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['irb120_trajectory_controller',"--param-file", ros2_controllers],
    output='screen'
    )
    gripper_controller_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['gripper_controller',"--param-file", ros2_controllers],
    output='screen'
    )
    # dual_arms_controller_spawner = Node( # This remains commented out from the original file
    # package='controller_manager',
    # executable='spawner',
    # arguments=['dual_arms_controller',"--param-file", ros2_controllers],
    # output='screen'
    # )
    robot_description_semantic_config = load_file("dual_arms", "config/" + "dual_arms.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}
    kinematics_path = "/home/omar-magdy/gp_ws/install/dual_arms/share/dual_arms/config/kinematics.yaml"
    with open(kinematics_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    kinematics_file = load_yaml("dual_arms", "config/kinematics.yaml")

    robot_description_kinematics = {"robot_description_kinematics": kinematics_file}
    print("robot_description_kinematics:", robot_description_semantic_config)
    joint_limits_yaml = load_yaml("dual_arms", "config/joint_limits.yaml")


    joint_limits = {'robot_description_planning': joint_limits_yaml}
    ompl_planning = load_yaml("dual_arms","config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["move_group"].update(ompl_planning)
    
    moveit_simple_controllers_yaml = load_yaml("dual_arms","config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }
    
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
          #  move_group_capabilities,
            {"use_sim_time": True},
        ]
    )
    
    
    rviz_full_config = os.path.join(robot_description_path+"/config/moveit.rviz")
    print(rviz_full_config)
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            trajectory_execution,
            ompl_planning_pipeline_config,
            moveit_controllers,
            planning_scene_monitor_parameters,
            move_group_capabilities,
            {"use_sim_time": True},
        ]
    )
    
    
    MoveInterface = Node(
            name="move",
            package="ros2srrc_execution",
            executable="move",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics_yaml,
                        {"use_sim_time": True}, {"ROB_PARAM":"irb120"},
                        {"EE_PARAM": "none"}, {"ENV_PARAM": "gazebo"}],
        )
    
    RobMoveInterface = Node(
        name="robmove",
        package="ros2srrc_execution",
        executable="robmove",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml,
                    {"use_sim_time": True}, {"ROB_PARAM": "irb120"}],
    )
    RobPoseInterface = Node(
        name="robpose",
        package="ros2srrc_execution",
        executable="robpose",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml,
                    {"use_sim_time": True}, {"ROB_PARAM":"irb120"}],
    )
    
    # Add nodes to the launch description:
    LD.add_action(gazebo)
    LD.add_action(node_robot_state_publisher)
    LD.add_action(static_tf)
    LD.add_action(spawn_entity)
    LD.add_action(ros2_control_node)
    LD.add_action(bridge) # Bridge node is included from the original file

    
    # Schedule controllers after a short delay
    LD.add_action(TimerAction(
        period=1.0,
        actions=[joint_state_broadcaster_spawner]
    ))
    LD.add_action(TimerAction(
        period=1.5,
        actions=[ar4_trajectory_controller_spawner]
    ))
    LD.add_action(TimerAction(
        period=1.5,
        actions=[irb120_trajectory_controller_spawner]
    ))
    # LD.add_action(TimerAction( # Keeping this commented out as it was in the base file
    #     period=1.5,
    #     actions=[dual_arms_controller_spawner]
    # ))
    LD.add_action(TimerAction(
        period=3.5,
        actions=[gripper_controller_spawner]
    ))
    # Schedule MoveIt!2 nodes after spawn_entity with a 2 second delay
    LD.add_action(TimerAction(
        period=4.0, # Kept the original 4.0 second delay
        actions=[rviz_node_full, run_move_group_node]
    ))
    
    # Schedule custom interfaces with a 5 second delay
    LD.add_action(TimerAction(
        period=6.0, # Kept the original 6.0 second delay
        actions=[MoveInterface, RobPoseInterface, RobMoveInterface]
    ))
    
    return LD

if __name__ == '__main__':
    ld = generate_launch_description()
