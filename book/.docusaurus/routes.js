import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/markdown-page',
    component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/markdown-page', '9fb'),
    exact: true
  },
  {
    path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs',
    component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs', '2cf'),
    routes: [
      {
        path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs',
        component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs', '520'),
        routes: [
          {
            path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs',
            component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs', 'caa'),
            routes: [
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/intro',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/intro', '85d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-1-foundations/chapter-1-intro-to-physical-ai',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-1-foundations/chapter-1-intro-to-physical-ai', '330'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-1-foundations/chapter-2-embodied-intelligence',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-1-foundations/chapter-2-embodied-intelligence', '62a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-2-ros-2/chapter-3-ros-2-architecture',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-2-ros-2/chapter-3-ros-2-architecture', 'd3b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-2-ros-2/chapter-4-ros-2-nodes-and-messages',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-2-ros-2/chapter-4-ros-2-nodes-and-messages', '245'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-3-simulation/chapter-5-simulation-first-development',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-3-simulation/chapter-5-simulation-first-development', 'c6d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-3-simulation/chapter-6-gazebo-and-unity-for-robotics',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-3-simulation/chapter-6-gazebo-and-unity-for-robotics', '4be'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-4-perception-and-control/chapter-7-nvidia-isaac-for-perception',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-4-perception-and-control/chapter-7-nvidia-isaac-for-perception', 'b48'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-4-perception-and-control/chapter-8-control-pipelines',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-4-perception-and-control/chapter-8-control-pipelines', '68e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-5-vision-language-action/chapter-10-building-vla-systems',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-5-vision-language-action/chapter-10-building-vla-systems', '47d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-5-vision-language-action/chapter-9-connecting-llms-to-ros',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-5-vision-language-action/chapter-9-connecting-llms-to-ros', 'd2b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-6-capstone-project/chapter-11-autonomous-conversational-humanoid',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-6-capstone-project/chapter-11-autonomous-conversational-humanoid', '4b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-6-capstone-project/chapter-12-future-directions',
                component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/docs/part-6-capstone-project/chapter-12-future-directions', 'e6e'),
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
    path: '/Hackathon_Physical_AI_-Humanoid_Robotics_Book/',
    component: ComponentCreator('/Hackathon_Physical_AI_-Humanoid_Robotics_Book/', '77c'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
