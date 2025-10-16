import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():

    # =============================================================================
    # === THE DEFINITIVE FIX for Invisible Models in Gazebo                     ===
    # =============================================================================
    # This finds the 'share' directory for each package with meshes and tells Gazebo
    # to look in its parent directory. This correctly resolves the path issue.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(get_package_share_directory('assembly_environment'), '..'),
            os.path.join(get_package_share_directory('annin_ar4_description'), '..'),
            os.path.join(get_package_share_directory('ros2srrc_robots'), '..'),
            os.path.join(get_package_share_directory('dual_arms'), '..')
        ]
    )
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Launch Clock Bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Robot Description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([get_package_share_directory('dual_arms'), "urdf", "dual_arms_with_environment.xacro"]),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'dual_arms_scene'],
        output='screen'
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ar4_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["ar4_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    irb120_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["irb120_trajectory_controller", "--controller-manager", "/controller_manager"],
    )
    irb120_gripper_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["irb120_gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # MoveIt 2 Configuration
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([get_package_share_directory('dual_arms'), "config", "dual_arms.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}
    
    kinematics_yaml = load_yaml("dual_arms", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    joint_limits_yaml = load_yaml("dual_arms", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_yaml = load_yaml("dual_arms", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_yaml)
    
    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml("dual_arms", "config/moveit_controllers.yaml"),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    trajectory_execution = {"moveit_manage_controllers": True}
    planning_scene_monitor_parameters = {"publish_planning_scene": True}

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )
    
    # RViz Node
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory('dual_arms'), "config", "moveit.rviz")],
        parameters=[robot_description, robot_description_semantic, ompl_planning_pipeline_config, robot_description_kinematics, robot_description_planning],
    )

    # Launch Description Assembly
    return LaunchDescription([
        gz_resource_path,
        gazebo,
        clock_bridge,
        robot_state_publisher_node,
        spawn_entity,
        move_group_node,
        rviz_node,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[joint_state_broadcaster_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[ar4_controller_spawner, irb120_controller_spawner,irb120_gripper_controller_spawner])
        ),
    ])