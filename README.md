# fairino_ros2

This fork of [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2) attempts to fix some outstanding issues and add some new features. `ros2_cmd_server` has been removed, and all control is performed via the hardware plugin.

**Fixed issues/New features:**
- URDFs for FR10 and FR20 rewritten with more modularity, simplified collision meshes, and corrected mass and effort values
    - These models were addressed because they are the hardware we have, but it should be easy to update the others (PRs are welcomed!)
- Controller IP address exposed as a parameter for hardware interface, allowing connection to multiple robots simultaneously
- Full state feedback exposed to ROS2 control (joint velocity, acceleration, and effort)

**Still to do:**
- Fairino library wrapper to support multiple software versions
    - Currently you must build against the desired version by setting `LIBFAIRINO_FILE` in `fairino_hardware/CMakeLists.txt` (see the upstream for how to match library version to controller software version)
- Expose GPIO to ROS2 control
- Expose FTS to ROS2 control
- Implement and test command mode switching for velocity and torque commands
- Match error codes to meaningful error messages

Fairino's [original documentation can be found here.](https://fairino-doc-en.readthedocs.io/latest/)
