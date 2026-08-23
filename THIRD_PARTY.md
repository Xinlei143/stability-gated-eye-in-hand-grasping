# Third-party components

The project-owned `foam_grasp` ROS 2 package is licensed under Apache-2.0.
The following upstream projects are not copied into this repository. They are
resolved at the exact commits recorded in `dependencies/*.repos` and retain
their own upstream licenses and notices:

- OrbbecSDK_ROS2: <https://github.com/orbbec/OrbbecSDK_ROS2>
- Piper ROS: <https://github.com/binb1nwu/piper_ros>
- MoveIt 2: <https://github.com/moveit/moveit2>
- MoveIt messages: <https://github.com/ros-planning/moveit_msgs>
- MoveIt resources: <https://github.com/ros-planning/moveit_resources>
- gazebo_grasp_fix: <https://github.com/JenniferBuehler/gazebo-pkgs>

`gazebo_grasp_fix` is imported at commit
`d217a4cd3045cb9622936784658ea687e0e6b69f` as recorded in
`dependencies/gazebo_grasp_plugin.repos`. This repository carries the small
standalone Gazebo 11 CMake integration in
`patches/gazebo_grasp_plugin/0001-standalone-gazebo11-cmake.patch`; it does not
modify the plugin's grasp logic. The upstream repository's root notice is
BSD-3-Clause while the plugin package declares GPL-3.0-or-later, so retain both
notices and review redistribution obligations before publishing a binary.

The reproducible import step applies the project-owned configuration patch
`patches/piper_ros/0001-use-kdl-kinematics.patch` after restoring the pinned
Piper ROS commit. The upstream checkout itself remains unmodified and its
commit is verified before the generated workspace mirror is built.

Before redistributing a deployment archive that embeds any of these source
trees, review and preserve the license files shipped by each upstream project.
