import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
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
        cmd=["ign", "gazebo", "empty.sdf"],
        output="screen",
    )

    # Forward simulation time from Gazebo to ROS.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
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
        return [LogInfo(
            msg="Panda ready: simulation paused; both controllers inactive."
        )]

    return LaunchDescription([
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

        state_publisher,
        gazebo,
        clock_bridge,
        spawn_robot,
    ])
