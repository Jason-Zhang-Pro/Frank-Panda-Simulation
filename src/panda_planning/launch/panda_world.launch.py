import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    folder = get_package_share_directory("panda_planning")
    gui = LaunchConfiguration("gui")
    auto_play = LaunchConfiguration("auto_play")
    # 1. Locate resources and expand Xacro.
    package_share = get_package_share_directory("franka_description")
    model_path = os.path.join(
        package_share, "robots", "panda", "panda_sim.urdf.xacro"
    )
    robot_description = xacro.process_file(model_path).toxml()

    # Preserve existing Gazebo resource paths.
    resource_paths = [os.path.dirname(package_share)]
    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    if existing_path:
        resource_paths.append(existing_path)

    # 2. Publish the robot model and, later, joint transforms.
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
        output="screen",
    )

    # 3. Start Fortress paused: deliberately omit "-r".
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", os.path.join(folder, "worlds", "world.sdf")],
        output="screen",
    )

    gazebo_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-g"], condition=IfCondition(gui), output="screen")
    torque_control = Node(
        package="panda_planning", executable="control_node", output="screen")
    planner = Node(
        package="panda_planning", executable="planner_node", output="screen",
        parameters=[{"use_sim_time": True}])
    startup = ExecuteProcess(
        cmd=["ros2", "run", "panda_planning", "startup", "--auto-play", auto_play],
        output="screen")

    # Forward simulation time from Gazebo to ROS.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                   "/target_contacts@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts",
                   "/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
        output="screen",
    )

    # 4. Spawn exactly the same model used by state_publisher.
    # create waits for the world service and robot_description.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-name", "panda",
            "-topic", "/robot_description",
            "-allow_renaming", "false",
        ],
        output="screen",
    )

    # 5. Configure both controllers without activating them.
    state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--inactive",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    effort_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_effort_controller",
            "-c", "/controller_manager",
            "--inactive",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["panda_gripper_controller", "-c", "/controller_manager",
                   "--inactive", "--controller-manager-timeout", "60"], output="screen")

    # Advance only when the preceding process succeeds.
    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason="Panda spawning failed; inspect the logs."
            ))]
        return [state_spawner]

    def after_state_spawner(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason="Joint state controller setup failed."
            ))]
        return [effort_spawner]

    def after_effort_spawner(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(
                reason="Effort controller setup failed."
            ))]
        return [gripper_spawner]

    def after_gripper_spawner(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Gripper configuration failed"))]
        return [LogInfo(msg="手臂及夹爪已配置，正在自动激活…"), startup]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("auto_play", default_value="true"),
        SetEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            os.pathsep.join(resource_paths),
        ),

        # Register handlers before starting their target processes.
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=after_spawn,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=state_spawner,
            on_exit=after_state_spawner,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=effort_spawner,
            on_exit=after_effort_spawner,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gazebo,
            on_exit=[EmitEvent(event=Shutdown(reason="Gazebo exited."))],
        )),

        RegisterEventHandler(OnProcessExit(
            target_action=startup,
            on_exit=lambda event, context: [] if event.returncode == 0 else [
                EmitEvent(event=Shutdown(reason="Automatic controller startup failed"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=torque_control,
            on_exit=[EmitEvent(event=Shutdown(reason="Torque control script stopped"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper_spawner, on_exit=after_gripper_spawner,
        )),
        state_publisher,
        torque_control,
        planner,
        gazebo,
        gazebo_gui,
        clock_bridge,
        spawn_robot,
    ])
