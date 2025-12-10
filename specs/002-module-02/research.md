# Research: Gazebo Classic vs Garden for ROS 2 Compatibility

## Decision: Use Gazebo Garden (Ignition) for ROS 2 Humble

**Rationale**: Gazebo Garden (Ignition) is the recommended version for ROS 2 Humble Hawksbill. It provides better integration with ROS 2's architecture, improved performance, and active development support. While Gazebo Classic (deprecated version) still works with ROS 2, Garden is the forward-looking choice that aligns with the ROS 2 ecosystem's evolution.

## Alternatives Considered:

1. **Gazebo Classic (version 11)**:
   - Pros: More documentation, established workflows, simpler for beginners
   - Cons: Deprecated for ROS 2, no longer actively developed, potential compatibility issues with newer ROS 2 features

2. **Gazebo Garden (Ignition)**:
   - Pros: Native ROS 2 integration, actively maintained, better performance, aligned with future ROS 2 development
   - Cons: Less documentation for ROS 2 integration, steeper learning curve

3. **NVIDIA Isaac Sim**:
   - Pros: High-fidelity rendering, advanced physics, robotics-specific features
   - Cons: More complex setup, commercial license requirements, overkill for basic educational content

## Recommendation for Module 2:

Use Gazebo Garden for the following reasons:
- Better ROS 2 Humble compatibility
- Officially recommended for ROS 2 development
- More sustainable choice for educational content
- Supports the digital twin pipeline requirements

## Technical Details:

- **ROS 2 Humble** officially supports Gazebo Garden through the `ros_gz` bridge packages
- The `ros_gz` ecosystem provides seamless integration between ROS 2 and Gazebo Garden
- Sensor plugins (LiDAR, depth camera, IMU) are well-supported in Garden
- Physics simulation capabilities meet the humanoid robotics requirements

## Action Items:

1. All Gazebo examples in Module 2 will use Garden syntax and APIs
2. Dependencies will specify Gazebo Garden installation
3. Documentation will reference Garden-specific commands and configuration
4. Digital twin pipeline will use `ros_gz` bridge for Gazebo ↔ ROS 2 communication