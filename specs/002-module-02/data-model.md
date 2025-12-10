# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

## Key Entities

### 1. Digital Twin Pipeline
- **Description**: The connection system that synchronizes data between Gazebo physics simulation and Unity visualization environments
- **Attributes**:
  - Connection protocol (e.g., ROS 2 topics, custom bridge)
  - Data synchronization frequency
  - Latency tolerance parameters
- **Relationships**: Links Gazebo Simulation Environment to Unity Visualization Environment

### 2. Humanoid Robot Model
- **Description**: The virtual representation of a human-like robot that interacts with simulated physics and sensor systems
- **Attributes**:
  - Physical properties (mass, dimensions, joint limits)
  - Kinematic structure (URDF/SDF representation)
  - Visual appearance parameters
- **Relationships**: Used in both Gazebo Simulation Environment and Unity Visualization Environment

### 3. Gazebo Simulation Environment
- **Description**: The physics-based simulation environment where gravity, collisions, and physical interactions are calculated
- **Attributes**:
  - Physics engine parameters (gravity, friction, damping)
  - World configuration (SDF files)
  - Simulation time parameters
- **Relationships**: Contains Humanoid Robot Model, connected to Digital Twin Pipeline

### 4. Unity Visualization Environment
- **Description**: The high-fidelity rendering environment where visual representations of the simulation are displayed
- **Attributes**:
  - Rendering quality settings
  - Scene configuration (lighting, textures, materials)
  - Camera parameters
- **Relationships**: Contains Humanoid Robot Model, connected to Digital Twin Pipeline

### 5. Virtual Sensors
- **Description**: Simulated sensor systems (LiDAR, depth camera, IMU) that provide perception data for the humanoid robot in simulation
- **Attributes**:
  - Sensor type (LiDAR, depth camera, IMU)
  - Measurement parameters (range, resolution, noise characteristics)
  - Mounting position on robot
- **Relationships**: Attached to Humanoid Robot Model, generates sensor data

## State Transitions

### Humanoid Robot Model States
- **Initial**: Robot model loaded in simulation
- **Physics Active**: Robot responds to physical forces and collisions
- **Controlled**: Robot executes commands from control system
- **Sensor Equipped**: Robot has virtual sensors attached and active

### Digital Twin Pipeline States
- **Disconnected**: No data synchronization between environments
- **Connecting**: Establishing connection between Gazebo and Unity
- **Synchronized**: Real-time data flow between environments
- **Latent**: Connection established but with delay or reduced frequency

## Validation Rules

1. **Physics Consistency**: Physical properties of Humanoid Robot Model must be valid in both Gazebo and Unity environments
2. **Data Type Matching**: Data types transmitted via Digital Twin Pipeline must match between source and destination
3. **Sensor Configuration**: Virtual Sensors must be properly configured with realistic parameters for their type
4. **Synchronization Timing**: Digital Twin Pipeline must maintain acceptable latency for real-time simulation
5. **Coordinate System Alignment**: Coordinate systems between Gazebo and Unity must be properly aligned for accurate representation