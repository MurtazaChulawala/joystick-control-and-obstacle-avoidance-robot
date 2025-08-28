# *Mobile Robot with Joystick Control & automatic obstacle Avoidance*

### *This project builds upon the ROS2 From Scratch book by Edouard Renard, extending the basic Gazebo mobile robot simulation into a more advanced Autonomous Mobile Robot (AMR) system.*

## The Project allows
- Spawning and controlling the robot inside Gazebo using a joystick
- Integrating real sensor data (ultrasonic via Arduino)
- Enforcing a safety fallback feature to avoid collisions automatically
- Demonstrate modular robotics development, integrating simulation, sensor data, and teleoperation into a unified ROS 2 workspace.
---
## 📹 Demo Video
👉 [YouTube link – coming soon]

## 🚀 Features
- **Robot description & state publishing**
    - robot_state_publisher reads URDF and publishes robot_description to TF
    - ros_gz_sim uses robot_description to spawn robot in Gazebo

- **ROS-Gazebo bridging**
    - ros_gz_bridge bridges cmd_vel_safe and joint_states

- **Joystick control**
    - Using teleop_twist_joy package for gamepad inputs

- **Ultrasonic sensor integration**
    - Arduino Uno + HC-SR04 ultrasonic sensor
    - Data relayed via python serial library

- **Mediator node (safety override)**
    - Allows joystick motion when safe
    - Blocks forward motion if obstacle < 10 cm
    - Publishes geometry_msgs/msg/Twist accordingly

- **Unified Launch File**
    - Modular execution of all components in one command


## Project Structure
```text
📂 amr_system_ws/
├── src/
│   ├── sensor_integration/     > Reads & publishes ultrasonic data
│   ├── amr_bringup/            > Launch & config files for Gazebo
│   ├── amr_control/            > Safety override (mediator node)
│   └── amr_description/        > URDF & robot simulation setup
📂 arduino_setup/
│   ├── setup.png               > Wiring diagram
│   └── sensor_data.ino         > Arduino sketch for HC-SR04
└── README.md
```

## ⚙️ Requirements
* ROS 2 Jazzy (or compatible)
* Python 3.10+
* Arduino Uno + HC-SR04 ultrasonic sensor
* Joystick / Gamepad
* Install dependencies:
`sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy`

## ▶️ Running the Project
### **Build the workspace**
```bash
git clone https://github.com/MurtazaChulawala/joystick-control-and-obstacle-avoidance-robot.git
cd joystick-control-and-obstacle-avoidance-robot/
colcon build --symlink-install
source install/setup.bash
```

### **Launch the full system**
`ros2 launch amr_bringup amr.launch.xml`


### **Control the robot**
- Use the joystick left stick for motion
- Keep RB button pressed to activate teleop
- Robot moves freely when no obstacles are detected
- Forward motion is blocked if obstacle < 10 cm

## ✨ Acknowledgements
* ROS 2 From Scratch (Edouard Renard) for foundational learning
* teleop_twist_joy package by the ROS community
* Custom mediator node developed from scratch for this project
* ROS-Gazebo bridge maintainers
