# My Robotics Journey 🤖

> **📌 Important**: This repository is organized into multiple branches:
> - **`main`** - Contains all code (ROS1, ROS2, and MoveIt2) together
> - **`ros1`** - Dedicated branch for ROS1 packages only
> - **`ros2`** - Dedicated branch for ROS2 packages only  
> - **`moveit2`** - Dedicated branch for MoveIt2 packages only
> 
> **Switch to the appropriate branch** based on what you want to explore, or use `main` to see everything at once.

This repository documents my learning journey in robotics, covering ROS1, ROS2, and MoveIt2. It contains various projects, examples, and implementations that demonstrate fundamental concepts and practical applications in robotic systems.

## 📋 Overview

This repository is a collection of robotics projects and learning materials organized into three main categories:
- **ROS1** - Classic ROS (Robot Operating System) implementations
- **ROS2** - Next-generation ROS2 implementations
- **MoveIt2** - Motion planning framework for robotic arms

## 🌿 Branch Structure

This repository uses a branch-based organization:
- **`main`** - Contains all code (ROS1, ROS2, and MoveIt2) in one place
- **`ros1`** - Dedicated branch for ROS1 packages only
- **`ros2`** - Dedicated branch for ROS2 packages only
- **`moveit2`** - Dedicated branch for MoveIt2 packages only

> **Note**: The `main` branch contains all packages together. For a cleaner, focused view of specific ROS versions, check out the respective branches.

## 🗂️ Repository Structure

### ROS1 Packages

#### `ros1_my_py_pkg/`
Python-based ROS1 nodes covering fundamental concepts:
- **Basic Nodes**: `my_first_node.py` - Introduction to ROS1 nodes
- **Topics**: Publishers and subscribers for number counting, robot news, smartphone communication
- **Services**: Client-server implementations (`add_two_ints_client.py`, `add_two_ints_server.py`)
- **TurtleSim**: `turtle_controller.py`, `circle.py`, `draw_circle.py` - TurtleSim control examples
- **Hardware Simulation**: `battery.py`, `hw_status_publisher.py`, `led_panel.py` - Simulated hardware components
- **OOP Examples**: `oop_number_counter.py` - Object-oriented programming patterns
- **Odometry**: `pose_subscriber.py` - Robot pose tracking

#### `ros1_my_robot_interfaces/`
Custom message and service definitions for ROS1:
- **Messages**: `HardwareStatus.msg` - Hardware status information
- **Services**: 
  - `SetLed.srv` - LED control service
  - `ComputeDiskArea.srv` - Geometry calculation service

### ROS2 Packages

#### `my_py_pkg/`
Python-based ROS2 nodes implementing modern ROS2 patterns:
- **Basic Nodes**: `my_first_node.py` - ROS2 node fundamentals
- **Topics**: `number_publisher.py`, `number_counter.py` - Publisher/subscriber patterns
- **Services**: `add_two_ints_client.py`, `add_two_ints_server.py` - Service communication
- **Hardware Simulation**: `battery.py`, `hardware_status_publisher.py`, `led_panel.py`
- **Robot Communication**: `robot_news_station.py`, `smartphone.py`
- **Navigation**: `scanandodom.py` - Scanning and odometry integration

#### `my_cpp_pkg/`
C++ implementations of ROS2 nodes:
- **Basic Nodes**: `my_first_node.cpp`, `my_first_node_oop.cpp` - C++ node examples
- **Topics**: `number_publisher.cpp`, `number_counter.cpp`
- **Services**: `add_two_ints_client.cpp`, `add_two_ints_server.cpp`
- **Robot Communication**: `robot_news_station.cpp`, `smartphone.cpp`

#### `my_robot_interfaces/`
Custom interface definitions for ROS2:
- **Messages**:
  - `HardwareStatus.msg` - Temperature, motor status, debug messages
  - `LedStateArray.msg` - Array of LED states
- **Services**:
  - `SetLed.srv` - LED control with success feedback
  - `ComputeRectangleArea.srv` - Rectangle area calculation
- **Actions**:
  - `CountUntil.action` - Count until target with feedback

#### `actions_py/`
ROS2 action server and client implementation:
- `count_until_server.py` - Action server for counting operations
- `count_until_client.py` - Action client with goal, feedback, and result handling

### MoveIt2 Configuration

#### `moveit2/my_robot_description/`
URDF/Xacro files defining a 6-axis robotic arm:
- **Robot Model**: `my_robot.urdf.xacro`, `arm.xacro`, `gripper.xacro`
- **ROS2 Control**: `my_robot.ros2_control.xacro` - Hardware interface configuration
- **Specifications**: 6-DOF arm with base, shoulder, arm, elbow, forearm, wrist, hand, and tool links
- **Launch Files**: `display.launch.py` - Visualize robot in RViz

#### `moveit2/my_robot_moveit_config/`
Complete MoveIt2 configuration package:
- **Configuration Files**:
  - `joint_limits.yaml` - Joint limits and constraints
  - `kinematics.yaml` - IK solver configuration
  - `initial_positions.yaml` - Default joint positions
  - `pilz_cartesian_limits.yaml` - Cartesian motion limits
- **Launch Files**:
  - `demo.launch.py` - Complete MoveIt2 demo
  - `move_group.launch.py` - MoveGroup interface
  - `moveit_rviz.launch.py` - RViz visualization
  - `spawn_controllers.launch.py` - Controller spawning

#### `moveit2/my_robot_bringup/`
Launch files for bringing up the robot system:
- `arm.launch.py` - Arm controller launch
- `armmoveit.launch.py` - MoveIt2 integration
- `number_app.launch.py` - Custom application launch files
- Configuration files for controllers and RViz

#### `moveit2/my_robot_commander_cpp/`
C++ MoveIt2 commander implementation:
- `test_moveit.cpp` - MoveIt2 C++ API examples

## 🎯 Key Concepts Covered

### ROS Fundamentals
- ✅ Node creation and lifecycle management
- ✅ Topics (publish/subscribe communication)
- ✅ Services (request/response communication)
- ✅ Actions (long-running tasks with feedback)
- ✅ Custom messages, services, and actions
- ✅ Parameter handling
- ✅ Launch files

### Programming Patterns
- ✅ Procedural vs Object-Oriented approaches
- ✅ Python and C++ implementations
- ✅ ROS1 vs ROS2 differences

### Robot Control
- ✅ URDF/Xacro robot modeling
- ✅ ROS2 Control integration
- ✅ MoveIt2 motion planning
- ✅ Joint and Cartesian space control
- ✅ RViz visualization

### Hardware Simulation
- ✅ Battery status simulation
- ✅ Hardware status monitoring
- ✅ LED panel control
- ✅ Sensor data publishing

## 🚀 Getting Started

### Prerequisites
- ROS1 (Melodic/Noetic) or ROS2 (Humble/Iron) installed
- MoveIt2 (for MoveIt2 packages)
- Python 3.x
- C++ compiler (for C++ packages)

### Building the Workspace

#### For ROS2 packages:
```bash
cd ~/your_workspace
colcon build
source install/setup.bash
```

#### For ROS1 packages:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Running Examples

#### ROS2 Node Example:
```bash
ros2 run my_py_pkg my_first_node
```

#### ROS2 Service Example:
```bash
# Terminal 1
ros2 run my_py_pkg add_two_ints_server

# Terminal 2
ros2 run my_py_pkg add_two_ints_client
```

#### MoveIt2 Demo:
```bash
ros2 launch my_robot_moveit_config demo.launch.py
```

## 📚 Learning Path

This repository follows a progressive learning approach:
1. **Basics**: Simple nodes and communication patterns
2. **Intermediate**: Services, custom interfaces, hardware simulation
3. **Advanced**: Actions, MoveIt2, motion planning, robot control

## 🤝 Contributing

This is a personal learning repository, but suggestions and improvements are welcome!

## 📝 License

This repository is for educational purposes as part of my robotics learning journey.

---

**Note**: 
- The `main` branch contains all code (ROS1, ROS2, and MoveIt2) together
- For a focused view, check out the dedicated branches: `ros1`, `ros2`, or `moveit2`
- Make sure to use the appropriate ROS version when working with different packages
