# PID and Pure Pursuit Controller

This repository provides a comprehensive implementation of both a PID (Proportional-Integral-Derivative) controller and a Pure Pursuit controller for a robotic system. These controllers are used for precise path tracking and maintaining desired velocities in robotic systems, especially for mobile robots. This documentation will guide you through understanding the system, installation, configuration, usage, and contributing to this project.

## Table of Contents
- [Introduction](#introduction)
- [Technical Explanation](#technical-explanation)
    - [PID Controller](#pid-controller)
    - [Pure Pursuit Controller](#pure-pursuit-controller)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Visualization](#visualization)
- [Troubleshooting and Common Issues](#troubleshooting-and-common-issues)
- [Contributing](#contributing)
- [License](#license)

## Introduction

The PID controller is a well-known control loop mechanism widely used across different domains to maintain a desired output in dynamic systems. It is especially helpful in reducing steady-state error and maintaining stability. The Pure Pursuit controller, on the other hand, is a geometric path tracking algorithm commonly used in mobile robotics for efficiently following predefined paths.

This repository provides ROS (Robot Operating System) implementations of both controllers, which can be used to control the motion of a robot along a predefined trajectory, offering practical solutions for precise navigation and control of robots.

## Technical Explanation

### PID Controller

The PID controller in this project is designed to control the steering angle of a robot to minimize the lateral error, i.e., the distance between the robot and a predefined path. The ROS-based implementation allows the robot to use odometry data to determine its current position and adjust its steering accordingly. The controller subscribes to odometry messages and publishes steering commands to ensure the vehicle stays on track.

The controller logic continuously adjusts the steering based on three components:

- **Proportional (P):** Corrects the steering angle proportionally to the lateral error.
- **Integral (I):** Eliminates the residual steady-state error by integrating past errors.
- **Derivative (D):** Predicts future errors and applies corrective action to minimize overshoot.

### Pure Pursuit Controller

The Pure Pursuit controller is a geometric approach to path tracking, where the robot follows a target path by iteratively calculating the steering angle required to reach a **lookahead point**. The controller determines this lookahead point on the path ahead of the robot and adjusts the steering accordingly. This method is computationally simple yet effective for path following.

The ROS implementation involves subscribing to odometry data, identifying the closest point on the path, computing the lookahead point, and publishing steering commands. The visualized trajectory allows you to monitor the robot's progress and make real-time adjustments.

## Installation

### Ensure ROS Noetic is Installed

Follow the ROS Noetic installation guide to install ROS Noetic on your system.

### Create a ROS Workspace

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

### Clone the Repository

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/yourusername/pid_and_pp.git
    ```

### Install Dependencies using Rosdep

    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

### Build the ROS Workspace

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## Usage

To run the controllers, use the following commands:

1. **Launch the PID controller:**

    ```bash
    roslaunch pid_and_pp pid_controller.launch
    ```

2. **Launch the Pure Pursuit controller:**

    ```bash
    roslaunch pid_and_pp pure_pursuit_controller.launch
    ```

These launch files will initialize the necessary ROS nodes and parameters for running the respective controllers. You can use RViz to visualize the robot's movement and monitor its path tracking performance.

## Configuration

The configuration files for the controllers are located in the `config` directory. You can modify these files to change the parameters of the controllers.

- `pid_params.yaml`: Contains the parameters for the PID controller.
- `pure_pursuit_params.yaml`: Contains the parameters for the Pure Pursuit controller.

Example of `pid_params.yaml`:
    ```yaml
    pid:
    kp: 1.0
    ki: 0.1
    kd: 0.01
    ```

Example of `pure_pursuit_params.yaml`:
    ```yaml
    pure_pursuit:
    lookahead_distance: 1.0
    max_velocity: 2.0
    ```

## Visualization

To visualize the robot's performance and ensure that the controllers are working correctly, RViz can be used. Launch RViz using the following command:

    ```bash
    rosrun rviz rviz
    ```

- You can add **odometry**, **path**, and **lookahead point** markers to monitor the robot's state and trajectory.

Visualization provides a real-time view of:

- The target trajectory.
- The robot's current path.
- The lookahead points used by the Pure Pursuit controller.

## Troubleshooting and Common Issues

- **Robot oscillating excessively:** This may be due to improperly tuned PID parameters. Try reducing the `kp` value or adjusting `kd` to reduce the oscillations.
- **Robot not following the path correctly:** Ensure that the **lookahead distance** in the Pure Pursuit controller is appropriate for the given speed. A longer lookahead distance might smooth out the trajectory, while a shorter one may allow for tighter path tracking.
- **ROS packages missing:** Use `rosdep` to install any missing dependencies.

## Contributing

Contributions are welcome! Please follow these steps to contribute:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -am 'Add new feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Create a pull request.

Ensure your code adheres to the coding standards, and include tests where possible.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

---
If you have any questions or need further clarification, feel free to open an issue in the repository.

