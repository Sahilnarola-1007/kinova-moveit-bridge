# kinova_bringup.launch.py
#
# Single launch file to bring up the full simulated Kinova + MoveIt + bridge stack.
# Starts: robot_state_publisher, joint_state_publisher_gui, move_group, rviz2, and our bridge.
#
# Usage:
#   ros2 launch kinova_moveit_bridge kinova_bringup.launch.py
 
import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument, TimerAction 
 
def generate_launch_description():
    # -------------------------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------------------------
    launch_args = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="Robot IP passed to KinovaInterface (used only for mock in sim).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level for move_group: debug|info|warn|error|fatal.",
        ),
    ]
    log_level = LaunchConfiguration("log_level")
    robot_ip = LaunchConfiguration("robot_ip")
 
    # -------------------------------------------------------------------------
    # MoveIt config (URDF + SRDF + kinematics + controllers + planners)
    # MoveItConfigsBuilder reads the .setup_assistant file so we don't repeat
    # xacro args here.
    # -------------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
 
    # -------------------------------------------------------------------------
    # robot_state_publisher — URDF -> /tf
    # -------------------------------------------------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
 
    # -------------------------------------------------------------------------
    # joint_state_publisher_gui — slider-driven /joint_states
    # (acts as the fake hardware source for Day 16/17 sim)
    # -------------------------------------------------------------------------
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
 
    # -------------------------------------------------------------------------
    # move_group — OMPL/Pilz planners, trajectory execution, scene monitor
    # Note: output="screen", log-level as launch arg. No silent logging.
    # -------------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", log_level],
    )
 
    # -------------------------------------------------------------------------
    # rviz2 — MoveIt's shipped config
    # -------------------------------------------------------------------------
    rviz_config_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )
 
    # -------------------------------------------------------------------------
    # Our bridge — hosts KinovaMoveitBridge + KinovaGripperBridge in one process
    # -------------------------------------------------------------------------
    bridge_node = Node(
        package="kinova_moveit_bridge",
        executable="kinova_bridge_node",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )

    # Delay move_group so bridge action servers are online before MoveIt's
    # controller manager tries its initial connection handshake.
    delayed_move_group = TimerAction(period=3.0, actions=[move_group_node])
    
    return LaunchDescription(
        launch_args
        + [
            rsp_node,
            jsp_gui_node,
            delayed_move_group,
            rviz_node,
            bridge_node,
        ]
    )
 