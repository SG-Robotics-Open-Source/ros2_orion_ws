# Project Orion: A ROS 2 Stack for an Encoder-less Mecanum UGV

![ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/7f846976-a327-46c5-8511-2c049ef6a6b8)

This repository contains the complete ROS 2 Humble software stack for "Project Orion," a custom-built, encoder-less Mecanum-wheeled Unmanned Ground Vehicle (UGV) designed for autonomous inspection tasks. This project was developed as part of an internship at the CSIR - National Aerospace Laboratories (NAL).

The core focus of this project is to provide a robust localization and control solution for a challenging hardware platform, leveraging modern robotics tools and sensor fusion techniques.

## Key Features

*   **High-Fidelity Digital Twin:** A detailed, kinematically accurate URDF model created using XACRO, including all chassis components and sensor placements.
*   **Hardware Interface:** A robust, multi-tiered control pipeline using an Orin Nano (ROS 2), a Teensy 4.1 (I/O Hub), and an Arduino (IMU processing).
*   **Encoder-less Odometry:** A `forward_kinematics` node that provides a wheel odometry estimate derived from calibrated motor commands (using our State-Based Kinematic Calibration model).
*   **Teleoperation:** A stateful `teleop` node for smooth manual control, ideal for mapping.
*   **Launch System:** A master launch file (`digital_twin.launch.py`) to bring up the entire robot stack with a single command.

## System Architecture
<img width="940" height="240" alt="ZL71IWCn4BtdAm8zU5gguCc355jQH4HGgeSYIvfCjs6JJ9cTMEhNszQLZJ47vvRttanuxqqwLiJHEz6rDkmKAI_NIhSDOGncYWv9ZkRdIHG6DYewdcFW5_i9ykew8HKHxllDo_Caf4Q_mm37FJuua0IQLMWS2B5QBbxVvMXs7h5wNjnLCpbL7aWbyFtZCdczRG6SVB01l6AhkuIYPcrgDEY1wP3Wri" src="https://github.com/user-attachments/assets/fb251b66-eefd-4532-be1e-f2e84b2fa31c" />


## Dependencies

This project is built on ROS 2 Humble and requires several external hardware drivers and libraries.

### Core Dependencies
*   **ROS 2 Humble:** [Installation Instructions](https://docs.ros.org/en/humble/Installation.html)
*   **`robot_localization` (EKF):** `sudo apt install ros-humble-robot-localization`
*   **`imu_tools`:** `sudo apt install ros-humble-imu-tools`

### Sensor Drivers
*   **YDLIDAR HP60C Camera:** The driver for this camera is provided by Yahboom Technology and can be found in their `ascam_ros2_ws` workspace. [Link to their GitHub or the Google Drive you used].
*   **Pangolin:** Required by some visualization tools. It must be built from source. [Link to Pangolin GitHub](https://github.com/stevenlovegrove/Pangolin).

## Getting Started

1.  **Clone this repository:**
    ```bash
    git clone https://github.com/YourUsername/project_orion_ugv.git
    cd project_orion_ugv
    ```
2.  **Install all dependencies** as listed above.
3.  **Build the workspace:**
    ```bash
    colcon build --symlink-install
    ```
4.  **Source the workspace:**
    ```bash
    source install/setup.bash
    ```
5.  **Launch the Digital Twin:**
    ```bash
    ros2 launch ugv_bringup digital_twin.launch.py
    ```

## Acknowledgment
This work was made possible through an internship at CSIR-NAL. The URDF model and control concepts were inspired by and adapted from the work of Automatic Addison and Yahboom Technology on their Rosmaster X3.

## License
This project is licensed under the Apache License 2.0. See the LICENSE file for details.
# ros2-project-orion-ugv
A complete ROS 2 Humble software stack for the 'Project Orion' encoder-less Mecanum-wheeled UGV, featuring a high-fidelity URDF, hardware interface nodes, and teleoperation with optional RTABmap(VSLAM) support.
