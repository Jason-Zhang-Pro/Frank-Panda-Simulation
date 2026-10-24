# Third-party notices

## Franka robot description and meshes

Assets are adapted from [frankarobotics/franka_ros](https://github.com/frankarobotics/franka_ros), reference checkout `35e1f654426e04bc9f83b73af4ab68a3fb145c84`.

- Copyright: Franka Emika GmbH; upstream license: Apache-2.0.
- Preserved upstream [LICENSE](third_party/franka_ros/LICENSE) and [NOTICE](third_party/franka_ros/NOTICE), also included in `src/franka_description`.
- The [mesh SHA-256 manifest](third_party/franka_ros/mesh-sha256.txt) is inherited provenance from the earlier repository; it was not regenerated during this migration.
- Local adaptations include ROS 2 packaging, Gazebo integration, actuated fingers and controller interfaces.

Original integration/learning code has no project-wide reuse license selected (`UNLICENSED` in package metadata). Upstream assets retain their licenses; no ownership of the Franka assets or original planning algorithms is claimed.

## Separately installed dependencies

OMPL, MoveIt Core/FCL, Eigen, URDF/SRDFdom, ROS 2, ros2_control, Gazebo Fortress and ros_gz remain separately distributed libraries/tools under their respective licenses. This repository invokes their public APIs and does not vendor dependency binaries.
