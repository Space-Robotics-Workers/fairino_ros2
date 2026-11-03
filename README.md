# fairino_ros2
## About
This fork of [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2) attempts to fix some outstanding issues and add some new features. `ros2_cmd_server` has been removed, and all control is performed via the hardware plugin.

**Fixed issues/New features:**
- URDFs for FR10 and FR20 rewritten with more modularity, simplified collision meshes, and corrected mass, inertia, and effort values
    - These models were addressed because they are the hardware we have, but it should be easy to update the others (PRs are welcomed!)
- Controller IP address exposed as a parameter for hardware interface, allowing connection to multiple robots simultaneously
- Full joint state feedback exposed to ROS2 control (position, velocity, acceleration, and effort)
- FTS state interfaces exposed to ROS2 control

**Still to do:**
- Fairino library wrapper to support multiple software versions
- Expose GPIO to ROS2 control
- Implement and test command mode switching for velocity and torque commands
- Match error codes to meaningful error messages

Fairino's [original documentation can be found here.](https://fairino-doc-en.readthedocs.io/latest/)

## Installation
1. Ensure your controller is running software version `v3.9.7`, as our testing has found this to be the most stable, and so that is what this package is designed to use
    - If you must use a different version, make changes to the following
        1. `set(LIBFAIRINO_FILE "libfairino.so.X.Y.Z")` in `fairino_hardware/CMakeLists.txt`
        1. `#include "libfairinoX.Y.Z/robot.h"` and `const std::string BUILD_VERSION = "vA.B.C";` in `fairino_hardware/include/fairino_hardware_interface.hpp`
    - Where `A.B.C` is the controller software version and `X.Y.Z` is the library version (see the upstream for how to match these)
1. Install dependencies and build with the standard commands from the root of your workspace
    1. `rosdep install --from-paths src/fairino_ros2 --ignore-src -y`
    1. `colcon build --packages-up-to fairino_ros2`
    1. `source install/setup.bash`

## Usage
### Robot
1. Add the plugin to your URDF (see [this file](fairino_description/urdf/fairino10_v6/fairino10_v6.ros2_control.xacro) for an example)
    - If you are using a non-default controller IP address, you can pass it with `<param name="controller_ip">${my_controller_ip}</param>` in the same block as the plugin
1. Add desired controllers to your `ros2_controllers.yaml` file (see [this file](fairino10_v6_moveit2_config/config/ros2_controllers.yaml) for an example)
1. Start `robot_state_publisher`, `ros2_control_node`, `spawner`, and any other required nodes

### Force Torque Sensor
1. Configure and zero your FTS from the Fairino web interface
1. Add the sensor to your robot system in the URDF, like
    ```xml
    <sensor name="${prefix}ft_sensor" type="force_torque">
        <state_interface name="force.x" />
        ...
        <state_interface name="torque.z" />

        <param name="frame_id">${prefix}fts_link</param>
        <param name="fx_range">200</param>
        ...
        <param name="tz_range">5</param>
    </sensor>
    ```
1. Add the broadcaster to your `ros2_controllers.yaml` file, like
    ```yaml
    controller_manager:
      ros__parameters:
        update_rate: 100 # Hz

        fts_broadcaster:
          type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster
    ```
1. Start the control nodes as normal
