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
    component: ComponentCreator('/docs', '3b9'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', 'd8e'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '470'),
            routes: [
              {
                path: '/docs/',
                component: ComponentCreator('/docs/', 'a8c'),
                exact: true,
                sidebar: "tutorialSidebar"
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
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
