// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Fundamentals and Architecture',
          items: [
            'module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts',
            'module-01-ros2/chapter-01/lesson-02-setting-up-ros2-environment',
            'module-01-ros2/chapter-01/lesson-03-basic-command-line-tools'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Nodes, Topics, and Message Passing',
          items: [
            'module-01-ros2/chapter-02/lesson-01-understanding-nodes-and-lifecycle',
            'module-01-ros2/chapter-02/lesson-02-publishers-and-subscribers-deep-dive',
            'module-01-ros2/chapter-02/lesson-03-working-with-standard-messages'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Services, Actions, and Advanced Communication',
          items: [
            'module-01-ros2/chapter-03/lesson-01-services-request-response-communication',
            'module-01-ros2/chapter-03/lesson-02-actions-goal-based-communication',
            'module-01-ros2/chapter-03/lesson-03-parameters-and-configuration-management'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Launch Systems and Real-World Applications',
          items: [
            'module-01-ros2/chapter-04/lesson-01-launch-files-and-composable-nodes',
            'module-01-ros2/chapter-04/lesson-02-building-production-ready-ros2-systems',
            'module-01-ros2/chapter-04/lesson-03-simulation-to-reality-bridging-the-gap'
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-02-digital-twin/index',
        {
          type: 'category',
          label: 'Setup Guide',
          items: [
            'module-02-digital-twin/setup/index',
            'module-02-digital-twin/setup/environment-setup',
            'module-02-digital-twin/setup/ros-gz-bridge-setup',
            'module-02-digital-twin/setup/basic-humanoid-urdf',
            'module-02-digital-twin/setup/version-control-setup'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 1: Foundations',
          items: [
            'module-02-digital-twin/chapter-01-foundations/index',
            'module-02-digital-twin/chapter-01-foundations/lesson-01-introduction',
            'module-02-digital-twin/chapter-01-foundations/lesson-02-ros2-integration',
            'module-02-digital-twin/chapter-01-foundations/lesson-03-digital-twin-concepts'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Gazebo Physics',
          items: [
            'module-02-digital-twin/chapter-02-gazebo-physics/index',
            'module-02-digital-twin/chapter-02-gazebo-physics/lesson-01-gazebo-setup',
            'module-02-digital-twin/chapter-02-gazebo-physics/lesson-02-world-creation',
            'module-02-digital-twin/chapter-02-gazebo-physics/lesson-03-humanoid-interaction'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Unity Environments',
          items: [
            'module-02-digital-twin/chapter-03-unity-environments/index',
            'module-02-digital-twin/chapter-03-unity-environments/lesson-01-unity-scene-setup',
            'module-02-digital-twin/chapter-03-unity-environments/lesson-02-lighting-and-rendering',
            'module-02-digital-twin/chapter-03-unity-environments/lesson-03-humanoid-avatars'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Simulated Sensors',
          items: [
            'module-02-digital-twin/chapter-04-simulated-sensors/index',
            'module-02-digital-twin/chapter-04-simulated-sensors/lesson-01-lidar-simulation',
            'module-02-digital-twin/chapter-04-simulated-sensors/lesson-02-depth-camera-simulation',
            'module-02-digital-twin/chapter-04-simulated-sensors/lesson-03-imu-simulation'
          ],
        },
        {
          type: 'category',
          label: 'Chapter 5: Bridging Sim to Real',
          items: [
            'module-02-digital-twin/chapter-05-sim-to-real/index',
            'module-02-digital-twin/chapter-05-sim-to-real/lesson-01-limitations',
            'module-02-digital-twin/chapter-05-sim-to-real/lesson-02-transfer-considerations',
            'module-02-digital-twin/chapter-05-sim-to-real/lesson-03-best-practices'
          ],
        },
      ],
    },
  ],
};

module.exports = sidebars;