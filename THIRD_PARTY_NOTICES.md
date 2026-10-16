# Third-party notices

## Franka robot description and meshes

The robot-description assets are adapted from [frankarobotics/franka_ros](https://github.com/frankarobotics/franka_ros), reference checkout `35e1f654426e04bc9f83b73af4ab68a3fb145c84`.

- Upstream copyright: Franka Emika GmbH.
- Upstream license: Apache-2.0.
- Preserved copies: [LICENSE](third_party/franka_ros/LICENSE), [NOTICE](third_party/franka_ros/NOTICE).
- All 19 mesh files were compared with the local upstream checkout; [SHA-256 manifest](third_party/franka_ros/mesh-sha256.txt).
- Local changes include ROS 2 packaging, Gazebo Sim launch/control integration, an actuated two-finger hand, controller interfaces and scene assets.

The original integration/learning code currently has no project-wide reuse license declaration (`UNLICENSED` in package metadata). Upstream assets retain their own license; no ownership of the Franka models or original RRT algorithms is claimed.

## Runtime dependencies

ROS 2, Gazebo Sim, NumPy, SciPy and PyBullet remain separately distributed dependencies under their respective licenses. No dependency binaries are vendored.
