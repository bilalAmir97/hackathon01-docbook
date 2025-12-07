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
  ],
};

module.exports = sidebars;