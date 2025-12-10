# Quickstart Guide: Module 2 - The Digital Twin (Gazebo & Unity)

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Gazebo Garden (Ignition) installed
- Unity 2022.3 LTS installed
- Docusaurus development environment

## Setup Environment

### 1. Install ROS 2 Humble
```bash
# Follow official ROS 2 Humble installation guide for Ubuntu 22.04
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosInstall.sh | bash
source /opt/ros/humble/setup.bash
```

### 2. Install Gazebo Garden
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install gz-garden
```

### 3. Install ros_gz bridge
```bash
sudo apt install ros-humble-ros-gz
```

## Create Your First Digital Twin

### 1. Set up the workspace
```bash
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws
colcon build
source install/setup.bash
```

### 2. Start Gazebo with a simple world
```bash
gz sim -r empty.sdf
```

### 3. Create a basic humanoid model
Create a simple URDF file for your humanoid robot with basic joint structure and sensor mounts.

### 4. Test the digital twin pipeline
Run the simulation bridge to connect Gazebo physics to your visualization system.

## Key Commands

- `gz sim -r world_name.sdf` - Launch Gazebo with a specific world
- `ros2 run ros_gz_bridge parameter_bridge` - Start the ROS-Gazebo bridge
- `docusaurus start` - Start the documentation server

## Verification Steps

1. Verify Gazebo Garden is running: `gz --version`
2. Check ROS 2 installation: `ros2 topic list`
3. Test the bridge connection: `ros2 topic echo /world/empty/state`
4. Confirm documentation builds: `npm run build` in docs directory

## Next Steps

1. Follow Chapter 1: Foundations of the Digital Twin to learn core concepts
2. Proceed to Chapter 2: Physics in Gazebo to set up your first simulation
3. Continue with Chapter 3: High-Fidelity Environments in Unity
4. Complete Chapter 4: Simulated Sensors to add perception capabilities
5. Review Chapter 5: Bridging Sim to Real for transfer considerations