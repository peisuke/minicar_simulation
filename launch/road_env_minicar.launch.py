#!/usr/bin/env python3

import json
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
    OpaqueFunction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# Runtime output directories
RUNTIME_MODELS_DIR = "/tmp/minicar_simulation/models"
RUNTIME_OUTPUT_DIR = "/tmp/minicar_simulation/output"
SPAWN_POSE_FILE = os.path.join(RUNTIME_OUTPUT_DIR, "spawn_pose.json")


def load_spawn_pose():
    """Load spawn pose from JSON file if it exists."""
    if os.path.exists(SPAWN_POSE_FILE):
        with open(SPAWN_POSE_FILE, "r") as f:
            return json.load(f)
    return {"x": 0.0, "y": 1.5, "z": 0.05, "yaw": 0.0}


def create_spawn_actions(context, *args, **kwargs):
    """Create spawn actions after course generation, reading pose from JSON."""
    # Load spawn pose
    pose = load_spawn_pose()

    robot_ns = LaunchConfiguration("robot_ns").perform(context)
    entity = LaunchConfiguration("entity").perform(context)
    jsb_name = LaunchConfiguration("joint_state_broadcaster").perform(context)
    diff_name = LaunchConfiguration("diff_controller").perform(context)

    # Spawn entity with pose from course
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", f"/{robot_ns}/robot_description",
            "-entity", entity,
            "-x", str(pose["x"]),
            "-y", str(pose["y"]),
            "-z", str(pose["z"]),
            "-Y", str(pose["yaw"]),
        ],
    )

    # Controller manager path
    cm_fqn = f"/{robot_ns}/controller_manager"

    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[jsb_name, "--controller-manager", cm_fqn],
    )

    spawn_diff = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[diff_name, "--controller-manager", cm_fqn],
    )

    return [
        LogInfo(msg=f"Spawning at x={pose['x']:.3f}, y={pose['y']:.3f}, yaw={pose['yaw']:.3f}"),
        TimerAction(period=3.0, actions=[spawn]),
        TimerAction(period=6.0, actions=[spawn_jsb, spawn_diff]),
    ]


def generate_launch_description():
    # --- Launch args ---
    world = LaunchConfiguration("world")
    entity = LaunchConfiguration("entity")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_ns = LaunchConfiguration("robot_ns")
    jsb_name = LaunchConfiguration("joint_state_broadcaster")
    diff_name = LaunchConfiguration("diff_controller")
    seed = LaunchConfiguration("seed")
    start_position = LaunchConfiguration("start_position")
    generate_course = LaunchConfiguration("generate_course")
    gui = LaunchConfiguration("gui")

    # --- Paths ---
    pkg_minicar = FindPackageShare("minicar_simulation")
    default_world = PathJoinSubstitution([pkg_minicar, "worlds", "road_env.world"])

    # Gazebo model path setup - include both install and runtime directories
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    pkg_share_path = get_package_share_directory("minicar_simulation")
    install_models_path = os.path.join(pkg_share_path, "models")

    # Build model path: runtime dir (priority) + install dir + existing
    model_paths = [RUNTIME_MODELS_DIR, install_models_path]
    if current_model_path:
        model_paths.append(current_model_path)
    combined_model_path = ':'.join(model_paths)

    model_path_env = SetEnvironmentVariable('GAZEBO_MODEL_PATH', combined_model_path)

    # Course generation script path
    script_path = os.path.join(pkg_share_path, "scripts", "generate_course.py")

    # Course generation process
    generate_course_cmd = ExecuteProcess(
        cmd=[
            'python3', script_path,
            '--models-dir', RUNTIME_MODELS_DIR,
            '--output-dir', RUNTIME_OUTPUT_DIR,
            '--seed', seed,
            '--start-position', start_position,
        ],
        name='generate_course',
        output='screen',
        condition=IfCondition(generate_course),
    )

    # Gazebo launch
    gazebo_ros_share = FindPackageShare("gazebo_ros")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": world, "gui": gui}.items(),
    )

    # Robot state publisher launch
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_minicar, "description", "launch", "robot_state_publisher.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_ns": robot_ns
        }.items(),
    )

    # Actions to run after course generation (when generate_course is true)
    # Use OpaqueFunction to read spawn pose from file after generation
    delayed_gazebo_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=generate_course_cmd,
            on_exit=[
                LogInfo(msg="Course generation completed. Starting Gazebo..."),
                gazebo_launch,
                rsp_launch,
                OpaqueFunction(function=create_spawn_actions),
            ],
        ),
        condition=IfCondition(generate_course),
    )

    # --- Actions for when generate_course is false (use provided x, y, z, yaw) ---
    spawn_immediate = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", ["/", robot_ns, "/robot_description"],
            "-entity", entity,
            "-x", x, "-y", y, "-z", z,
            "-Y", yaw,
        ],
    )

    cm_fqn = ["/", robot_ns, "/controller_manager"]

    spawn_jsb_immediate = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[jsb_name, "--controller-manager", cm_fqn],
    )

    spawn_diff_immediate = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[diff_name, "--controller-manager", cm_fqn],
    )

    immediate_spawn = TimerAction(
        period=3.0,
        actions=[spawn_immediate],
        condition=UnlessCondition(generate_course),
    )

    immediate_controllers = TimerAction(
        period=6.0,
        actions=[spawn_jsb_immediate, spawn_diff_immediate],
        condition=UnlessCondition(generate_course),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("entity", default_value="minicar"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="1.5"),
        DeclareLaunchArgument("z", default_value="0.05"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_ns", default_value="sim_robot"),
        DeclareLaunchArgument("joint_state_broadcaster", default_value="joint_state_broadcaster"),
        DeclareLaunchArgument("diff_controller", default_value="diff_drive_controller"),
        DeclareLaunchArgument(
            "seed",
            default_value="",
            description="Random seed for course generation. Empty for random course."
        ),
        DeclareLaunchArgument(
            "start_position",
            default_value="0.0",
            description="Start position on course as ratio (0.0 to 1.0)."
        ),
        DeclareLaunchArgument(
            "generate_course",
            default_value="true",
            description="Whether to generate a new course at runtime (true/false)"
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Set to 'false' to run Gazebo headless (no GUI)"
        ),

        model_path_env,
        generate_course_cmd,
        delayed_gazebo_launch,

        # These only run when generate_course is false
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([gazebo_ros_share, "launch", "gazebo.launch.py"])
            ),
            launch_arguments={"world": world, "gui": gui}.items(),
            condition=UnlessCondition(generate_course),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_minicar, "description", "launch", "robot_state_publisher.launch.py"])
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "robot_ns": robot_ns
            }.items(),
            condition=UnlessCondition(generate_course),
        ),
        immediate_spawn,
        immediate_controllers,
    ])
