# mrv1a_ros2 #

This stack includes `ros2_control` drivers for the Mitsubishi RV1A robotic manipulator.

## Features ##
- integration with `ros2_control`
- integration with Gazebo
- integration with Moveit2

## Available Packages in this Repository ##
- `mrv1a_bringup` - launch and run-time configurations
- `mrv1a_description` - robot description and configuration files
- `mrv1a_hardware` - hardware interfaces for communication with the robot
- `mrv1a_moveit2` - some tools for Moveit2 integration

## Getting Started
***Required setup : Ubuntu 20.04 LTS***

1.  Install `ros2` packages. The current developpment is based of `ros2 galactic`. Installation steps are decribed [here](https://docs.ros.org/en/galactic/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/galactic/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/mrv1a_ros2.git src/mrv1a_ros2
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```

## Usage

:warning: **SAFETY FIRST**:warning:
*An industrial robot is not a toy and you may harm yourself due to misuse. In general it is best practice to test your code at first in simulation and then in low speed before using the robot, make yourself familiar with the safety instructions provided by the robot manuals.*

The `mrv1a_bringup` package contains the follwing launchers :
- `mrv1a.launch.py` - is the main launcher giving access to all feaures of the driver.

The arguments for launch files can be listed using
```shell
ros2 launch mrv1a_bringup <launch_file_name>.launch.py --show-args
```
The most relevant arguments of `mrv1a.launch.py` are:

- `runtime_config_package` (default: "mrv1a_description") - name of the package with the controller's configuration in `config` folder. Usually the argument is not set, it enables use of a custom setup.
- `controllers_file` (default: "mrv1a_controllers.yaml") - YAML file with the controllers configuration.
- `description_package` (default: "mrv1a_description") - Description package with robot URDF/xacro files. Usually the argument is not set, it enables use of a custom description.
- `description_file` (default: "mrv1a.config.xacro") - URDF/XACRO description file with the robot.
- `prefix` (default: "") - Prefix of the joint names, useful for multi-robot setup. If changed than also joint names in the controllers' configuration have to be updated.
- `use_sim` (default: "false") - Start robot in Gazebo simulation.
- `use_fake_hardware` (default: "true") -Start robot with fake hardware mirroring command to its states.
- `robot_controller` (default: "mrv1a_arm_controller") - Robot controller to start.
- `start_rviz` (default: "true") - Start RViz2 automatically with this launch file.
- `robot_ip` (default: "192.168.122.200") - Robot IP.
- `robot_port` (defaut: "10000") - Robot port.
- `initial_positions_file` (default: "initial_positions.yaml") - Configuration file of robot initial positions for simulation.

**HINT**: list all loaded controllers using `ros2 control list_controllers` command.

**NOTE**: The package can simulate hardware with the ros2_control `FakeSystem`. This is the default behavior. This emulator enables an environment for testing of "piping" of hardware and controllers, as well as testing robot's descriptions. For more details see ros2_control documentation for more details.

### Example commands for setup testing
1. Start the simulated hardware, in a sourced terminal run
    ```shell
    ros2 launch mrv1a_bringup mrv1a.launch.py
    ```
    add the parameter `use_fake_hardware:="false"` to control the real robot.
2. Send joint trajectory goals to the hardware by using a demo node from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) package by running
    ```shell
    ros2 launch mrv1a_bringup mrv1a_test_joint_trajectory_controller.launch.py
    ```
After a few seconds the robot should move.


## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://plateforme.icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [m.bednarczyk@unistra.fr](mailto:m.bednarczyk@unistra.fr), @github: [mcbed](mailto:macbednarczyk@gmail.com)
