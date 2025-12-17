import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', 'ebe'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '6fd'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', '188'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'a09'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '020'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', 'e94'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', 'b25'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', '6d1'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '962'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '4dc'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', 'a8c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/chat-assistant',
                component: ComponentCreator('/docs/chat-assistant', 'f3b'),
                exact: true
              },
              {
                path: '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts',
                component: ComponentCreator('/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts', 'ffa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-01/lesson-02-setting-up-ros2-environment',
                component: ComponentCreator('/docs/module-01-ros2/chapter-01/lesson-02-setting-up-ros2-environment', '6bd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-01/lesson-03-basic-command-line-tools',
                component: ComponentCreator('/docs/module-01-ros2/chapter-01/lesson-03-basic-command-line-tools', '564'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-02/lesson-01-understanding-nodes-and-lifecycle',
                component: ComponentCreator('/docs/module-01-ros2/chapter-02/lesson-01-understanding-nodes-and-lifecycle', 'ace'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-02/lesson-02-publishers-and-subscribers-deep-dive',
                component: ComponentCreator('/docs/module-01-ros2/chapter-02/lesson-02-publishers-and-subscribers-deep-dive', '861'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-02/lesson-03-working-with-standard-messages',
                component: ComponentCreator('/docs/module-01-ros2/chapter-02/lesson-03-working-with-standard-messages', '871'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-03/lesson-01-services-request-response-communication',
                component: ComponentCreator('/docs/module-01-ros2/chapter-03/lesson-01-services-request-response-communication', '779'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-03/lesson-02-actions-goal-based-communication',
                component: ComponentCreator('/docs/module-01-ros2/chapter-03/lesson-02-actions-goal-based-communication', '8ea'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-03/lesson-03-parameters-and-configuration-management',
                component: ComponentCreator('/docs/module-01-ros2/chapter-03/lesson-03-parameters-and-configuration-management', '66c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-04/lesson-01-launch-files-and-composable-nodes',
                component: ComponentCreator('/docs/module-01-ros2/chapter-04/lesson-01-launch-files-and-composable-nodes', 'd6e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-04/lesson-02-building-production-ready-ros2-systems',
                component: ComponentCreator('/docs/module-01-ros2/chapter-04/lesson-02-building-production-ready-ros2-systems', '79b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/chapter-04/lesson-03-simulation-to-reality-bridging-the-gap',
                component: ComponentCreator('/docs/module-01-ros2/chapter-04/lesson-03-simulation-to-reality-bridging-the-gap', 'b56'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-01-ros2/frontmatter-template',
                component: ComponentCreator('/docs/module-01-ros2/frontmatter-template', '36e'),
                exact: true
              },
              {
                path: '/docs/module-02-digital-twin',
                component: ComponentCreator('/docs/module-02-digital-twin', '75b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-01-foundations/',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-01-foundations/', '267'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-01-foundations/lesson-01-introduction',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-01-foundations/lesson-01-introduction', 'e0a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-01-foundations/lesson-02-ros2-integration',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-01-foundations/lesson-02-ros2-integration', '48e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-01-foundations/lesson-03-digital-twin-concepts',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-01-foundations/lesson-03-digital-twin-concepts', '216'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-02-gazebo-physics/',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-02-gazebo-physics/', '810'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-02-gazebo-physics/lesson-01-gazebo-setup',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-02-gazebo-physics/lesson-01-gazebo-setup', '7d7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-02-gazebo-physics/lesson-02-world-creation',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-02-gazebo-physics/lesson-02-world-creation', '0ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-02-gazebo-physics/lesson-03-humanoid-interaction',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-02-gazebo-physics/lesson-03-humanoid-interaction', 'c2a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-03-unity-environments/',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-03-unity-environments/', '03c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-03-unity-environments/lesson-01-unity-scene-setup',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-03-unity-environments/lesson-01-unity-scene-setup', 'fbf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-03-unity-environments/lesson-02-lighting-and-rendering',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-03-unity-environments/lesson-02-lighting-and-rendering', '3ed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-03-unity-environments/lesson-03-humanoid-avatars',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-03-unity-environments/lesson-03-humanoid-avatars', '3a8'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-04-simulated-sensors',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-04-simulated-sensors', '6e9'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-04-simulated-sensors/lesson-01-lidar-simulation',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-04-simulated-sensors/lesson-01-lidar-simulation', 'e1d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-04-simulated-sensors/lesson-02-depth-camera-simulation',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-04-simulated-sensors/lesson-02-depth-camera-simulation', '747'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-04-simulated-sensors/lesson-03-imu-simulation',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-04-simulated-sensors/lesson-03-imu-simulation', 'a77'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-05-sim-to-real',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-05-sim-to-real', '9fd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-05-sim-to-real/lesson-01-limitations',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-05-sim-to-real/lesson-01-limitations', 'e73'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-05-sim-to-real/lesson-02-transfer-considerations',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-05-sim-to-real/lesson-02-transfer-considerations', 'b41'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/chapter-05-sim-to-real/lesson-03-best-practices',
                component: ComponentCreator('/docs/module-02-digital-twin/chapter-05-sim-to-real/lesson-03-best-practices', 'fd7'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/setup/',
                component: ComponentCreator('/docs/module-02-digital-twin/setup/', '8cc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/setup/basic-humanoid-urdf',
                component: ComponentCreator('/docs/module-02-digital-twin/setup/basic-humanoid-urdf', '829'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/setup/environment-setup',
                component: ComponentCreator('/docs/module-02-digital-twin/setup/environment-setup', 'f55'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/setup/ros-gz-bridge-setup',
                component: ComponentCreator('/docs/module-02-digital-twin/setup/ros-gz-bridge-setup', 'b07'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-02-digital-twin/setup/version-control-setup',
                component: ComponentCreator('/docs/module-02-digital-twin/setup/version-control-setup', '58c'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '68e'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
