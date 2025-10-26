# Project Orion: A ROS 2 Stack for an Encoder-less Mecanum UGV

> **Note from SG-Robotics-Open-Source:** This repository is a library version of a project originally created by Shahazad Abdulla. We maintain this copy as a stable, well-documented starting point for our community.

> **Original Author:** [ShahazadAbdulla](https://github.com/ShahazadAbdulla)
> **Original Repository:** [ShahazadAbdulla/ros2-project-orion-ugv](https://github.com/ShahazadAbdulla/ros2-project-orion-ugv)

---

"Project Orion" is a complete ROS 2 software stack for a custom-built, Mecanum-wheeled Unmanned Ground Vehicle (UGV).

This repository runs a **high-fidelity "Digital Twin"** of the robot. This means that instead of a physics simulation (like Gazebo), we are running the robot's actual control software and visualizing its state in real-time in RViz. It's a virtual copy that perfectly mirrors the real robot's geometry, sensors, and intended motion.

![ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/7f846976-a327-46c5-8511-2c049ef6a6b8)

This platform is designed for students who want to build advanced projects on a robust foundation without starting from scratch.

To understand this project, it's helpful to know how the physical robot and the software are structured.

### The Hardware Hierarchy

The physical Orion UGV uses a multi-computer setup:
*   🧠 **The Brain (NVIDIA Orin Nano):** Runs the main ROS 2 system and all high-level software nodes.
*   🦾 **The I/O Hub (Teensy 4.1):** Acts as a real-time bridge. It takes commands from the Orin and generates the precise PWM signals to control the wheel motors.
*   ⚖️ **The IMU Co-processor (Arduino):** A dedicated board that processes data from the 9-DOF IMU sensor.

### The Software Packages

The code is organized into four main ROS 2 packages:
*   `ugv_description`: Contains the 3D model (URDF/XACRO) of the robot, based on its real-world CAD files.
*   `ugv_hardware`: The crucial hardware interface node that communicates with the Teensy over USB serial to send motor commands and receive IMU data.
*   `ugv_teleop`: A simple keyboard teleoperation node to drive the robot by publishing `/cmd_vel` messages.
*   `ugv_bringup`: Contains the master `launch` file to start the entire system with a single command.

## System Architecture
<img width="940" height="240" alt="ZL71IWCn4BtdAm8zU5gguCc355jQH4HGgeSYIvfCjs6JJ9cTMEhNszQLZJ47vvRttanuxqqwLiJHEz6rDkmKAI_NIhSDOGncYWv9ZkRdIHG6DYewdcFW5_i9ykew8HKHxllDo_Caf4Q_mm37FJuua0IQLMWS2B5QBbxVvMXs7h5wNjnLCpbL7aWbyFtZCdczRG6SVB01l6AhkuIYPcrgDEY1wP3Wri" src="https://github.com/user-attachments/assets/fb251b66-eefd-4532-be1e-f2e84b2fa31c" />

 
This guide assumes you have completed our `ros2_tutorial_ws` and understand the basic workflow.

### 4.1 - Install Dependencies

This project requires a few extra ROS 2 packages. Open a terminal and run the following commands:
```bash
#For Software
# Install xterm, which our launch file uses to open a new terminal for keyboard control
sudo apt install xterm
```
```bash
#For full stack with Hardware
# Install the Robot Localization package (used for sensor fusion)
sudo apt install ros-humble-robot-localization

# Install the IMU Tools package (for working with IMU data)
sudo apt install ros-humble-imu-tools
```

### 2. Clone and Build the Project
```bash
# In you home directory or wherever,
# Clone this repository from the SG-Robotics-Open-Source organization
git clone https://github.com/SG-Robotics-Open-Source/ros2_orion_ws

# Navigate back to the root of your workspace to build
cd ~/ros2_orion_ws

# Build the project using colcon
colcon build 
```
### 3. Source and Launch the Digital Twin
To run the software-only simulation, you will use the teleop_rviz.launch.py file. This file starts everything you need (RViz, robot_state_publisher, teleop, and the forward_kinematics simulator) without trying to connect to any hardware.
```bash  
# Source your workspace in your terminal
source install/setup.bash

# Launch the dedicated simulation file
ros2 launch ugv_bringup teleop_rviz.launch.py
```

## How to Know It's Working

After running the launch command, an **RViz** window and a small **xterm** window will open. Follow these steps to verify your setup.

### 1. **Crucial RViz Step: Set the Fixed Frame to `odom`**

The "Fixed Frame" in RViz is the reference point for everything you see. To correctly visualize the robot moving around its world, this **must be set to `odom`**. By default, it might be set to `base_footprint`, which will cause errors and make it seem like the robot isn't moving.

**Follow these images to fix it:**

<table>
  <tr>
    <td align="center"><strong>1. Initial View (Notice the errors and Fixed Frame is `base_footprint`)</strong></td>
  </tr>
  <tr>
    <td><img src="[<img width="1920" height="1077" alt="screenshot-20251027_022138" src="https://github.com/user-attachments/assets/e76bfd22-bc44-4731-9caa-2ad038c5c25f" />]" alt="RViz with base_footprint" width="600"></td>
  </tr>
  <tr>
    <td align="center"><strong>2. Click the dropdown next to "Fixed Frame".</strong></td>
  </tr>
  <tr>
    <td><img src="[PASTE-LINK-TO-YOUR-CLOSEUP-OF-BASE_FOOTPRINT-SELECTED]" alt="Selecting the Fixed Frame" width="400"></td>
  </tr>
  <tr>
    <td align="center"><strong>3. Select `odom` from the list. The errors should disappear.</strong></td>
  </tr>
  <tr>
    <td><img src="[PASTE-LINK-TO-YOUR-CLOSEUP-OF-ODOM-SELECTED]" alt="Odom selected" width="400"></td>
  </tr>
</table>

### 2. Drive the Digital Twin

Now that RViz is set up correctly, you can drive the robot:

*   Click on the **xterm window** to make it active.
*   Use the keyboard keys (`u`, `i`, `o`, etc.) to drive the robot.
*   You will see the robot model move within the main RViz window, leaving its starting point.

This confirms that the entire software simulation stack is working correctly. You have successfully visualized the TF transform from the `odom` frame to the `base_footprint` frame, which is a fundamental concept in mobile robotics.

**What you are seeing is the end-to-end data flow:** Your keyboard creates a `Twist` message, the hardware interface calculates the kinematics, and `robot_state_publisher` visualizes the result in RViz. This confirms the entire foundational software stack is working correctly.

## For Hardware Users (Advanced)

This repository also contains other launch files, such as digital_twin.launch.py. These files are intended for use with the physical Orion robot.

They are different because they also start the hardware_interface_node, which will actively try to connect to the Teensy microcontroller via a USB port (/dev/ttyACM0). If you run these launch files without the hardware connected, they will not work as expected.

## Acknowledgment
This work was made possible through an internship at CSIR-NAL. The URDF model and control concepts were inspired by and adapted from the work of Automatic Addison and Yahboom Technology on their Rosmaster X3.

## License
This project is licensed under the Apache License 2.0. See the LICENSE file for details.
