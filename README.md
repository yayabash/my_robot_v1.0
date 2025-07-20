# my_robot_v1.0 – ROS2 Mobile Robot with 2-Axis Arm

This repository contains a ROS2-based mobile robot simulation with a 2-axis robotic arm, developed as the final project for the *ROS2 For Beginners Level 2* course. The robot integrates a mobile base with a robotic arm, simulated in Gazebo and visualized in RViz. The arm is controlled using `JointStatePublisher` and `JointPositionController` plugins.

📽️ **Simulation Video**: Watch the robot in action: [ROS2 Mobile Robot with Arm Simulation](https://drive.google.com/file/d/1nkZ1LZqUfsTICFmxhn91IZOp4ncV8mBN/view?usp=sharing)

## 📦 Prerequisites

- ✅ **OS**: Ubuntu 24.04
- ✅ **ROS 2**: Jazzy
- ✅ **Gazebo and RViz**: Included in ROS 2 Desktop Full
- ✅ **Python 3.10+**

## 🚀 Step-by-Step Installation

### 1️⃣ Install ROS 2 Jazzy
Follow the official ROS 2 installation instructions:
- [ROS 2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)

### 2️⃣ Source ROS 2 and Add to `.bashrc`
```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
### 3️⃣ Set RMW Implementation (CycloneDDS)
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```
### 4️⃣ Source .bashrc
```bash
source ~/.bashrc
```
## 🧠 Workspace Setup
### 5️⃣ Install Git (if not already)
```bash
sudo apt install git
```
### 6️⃣ Clone This Repository
```bash
cd ~
git clone https://github.com/yayabash/my_robot_v1.0.git
cd ~/my_robot_v1.0
```
## ⚙️ Setup ROS Dependencies
### 7️⃣ Install rosdep and Initialize
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
### 8️⃣ Install Package Dependencies
```bash
rosdep install --from-paths src -y --ignore-src
```
## 🧱 Build the Workspace
```bash
cd ~/my_robot_v1.0
colcon build
```
### 9️⃣ Source the Workspace
```bash
source install/setup.bash
echo "source ~/my_robot_v1.0/install/setup.bash" >> ~/.bashrc
```
🧩 (Optional) Shell Autocompletion
```bash
sudo apt install python3-colcon-argcomplete
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```
🚀 Run the Simulation
### 1️⃣ Terminal 1: Launch Gazebo with the Robot
```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```
This launches the Gazebo simulation with the mobile robot and arm in the test_world.sdf environment.

### 2️⃣ Terminal 2: Launch RViz for Visualization
```bash
ros2 launch my_robot_description display.launch.py
```
This opens RViz with the robot’s URDF and arm visualization.

### 3️⃣ Terminal 3: Test Arm Control

Publish joint positions to test the arm’s JointPositionController plugins:
```bash
# Example: Control first joint
ros2 topic pub /arm_base_joint_position_controller/command std_msgs/msg/Float64 "{data: 1.57}"
# Example: Control second joint
ros2 topic pub /forearm_joint_position_controller/command std_msgs/msg/Float64 "{data: 0.785}"
```
Adjust values to test arm motion (e.g., between 0 and π/2 as per joint limits).

You should now see:

Gazebo with the mobile robot, 2-axis arm, and test_world.sdf environment.
RViz displaying the robot’s URDF and arm transformations.
The arm moving in response to joint position commands.



## 🛠️ Project Details

    Mobile Base: Defined in mobile_base.xacro and mobile_base_gazebo.xacro.
    Robotic Arm: 2-axis arm defined in arm.xacro and standalone_arm.urdf.xacro.
        Links: arm_base_link (box, 0.1x0.1x0.2 m, 5.0 kg, orange), forearm_link (cylinder, 0.02 m radius, 0.3 m length, 0.3 kg, yellow), hand_link (cylinder, 0.02 m radius, 0.3 m length, 0.3 kg, orange).
        Joints: Revolute joints with limits (0 to π/2), friction (0.3), damping (0.3), and JointPositionController plugins (P-gains: 5.0 for first joint, 3.0 for second joint).
    Gazebo Integration: Uses JointStatePublisher and JointPositionController plugins for arm control, with ROS2-Gazebo bridge (gazebo_bridge.yaml).
    RViz: Visualizes the robot and arm transformations using urdf_config.rviz.

## 📜 License

This project is licensed under the MIT License. See the LICENSE file for details.
