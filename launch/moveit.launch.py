import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ur_moveit_config.launch_common import load_yaml




def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description",
            default_value="",
            description="robot description string",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_semantic",
            default_value="",
            description="robot description semantic string",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "with_gripper",
            default_value="false",
            description="With gripper or not?",
        )
    )
    
    prefix = LaunchConfiguration("prefix")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    with_gripper_condition = LaunchConfiguration("with_gripper")
    robot_description_content = LaunchConfiguration("robot_description")
    robot_description_semantic_content = LaunchConfiguration("robot_description_semantic")

    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    use_sim_time = {"use_sim_time": use_sim_time}
    
    world_config_file = PathJoinSubstitution(
        [FindPackageShare("rbs_simulation"), "worlds", "mir.sdf"]
    )

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    
    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 100.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            use_sim_time,
        ],
    )
    
    move_topose_action_server = Node(
        package="rbs_skill_servers",
        executable="move_topose_action_server",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            use_sim_time,
        ]
    )

    gripper_control_node = Node(
        package="rbs_skill_servers",
        executable="gripper_control_action_server",
        parameters= [
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            use_sim_time,
        ],
        condition=IfCondition(with_gripper_condition)
    )

    move_cartesian_path_action_server = Node(
        package="rbs_skill_servers",
        executable="move_cartesian_path_action_server",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            use_sim_time,
        ]
    )
    
    move_joint_state_action_server = Node(
        package="rbs_skill_servers",
        executable="move_to_joint_states_action_server",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            use_sim_time,
        ]
    )
    
    moveit_planning_scene_init = Node(
        package="rbs_skill_servers",
        executable="moveit_update_planning_scene_service_server",
        output="screen",
        parameters=[
            {'init_scene': world_config_file},
            {'models_paths': os.environ['IGN_GAZEBO_RESOURCE_PATH']}
        ]
    )
    nodes_to_start = [
        move_group_node,
        move_topose_action_server,
        gripper_control_node,
        move_cartesian_path_action_server,
        move_joint_state_action_server,
        moveit_planning_scene_init
    ]
    return LaunchDescription(declared_arguments + nodes_to_start)
