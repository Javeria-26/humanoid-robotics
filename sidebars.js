// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'ROS 2 Educational Module',
      items: [
        'modules/ros2-nervous-system/index',
        'modules/ros2-nervous-system/ros2-fundamentals',
        'modules/ros2-nervous-system/python-ros-bridge',
        'modules/ros2-nervous-system/urdf-humanoids',
      ],
    },
  ],
  module2Sidebar: [
    {
      type: 'category',
      label: 'Module 2: Digital Twin Simulation',
      items: [
        'module-2/index',
        'module-2/digital-twins-robotics',
        'module-2/physics-simulation',
        'module-2/high-fidelity-env',
        'module-2/sensor-simulation',
        'module-2/glossary',
        'module-2/assessment-questions',
      ],
    },
    'references/index',
  ],
  aiRobotBrainSidebar: [
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'ai-robot-brain/chapter-1-overview',
        'ai-robot-brain/chapter-2-perception',
        'ai-robot-brain/chapter-3-navigation',
        'ai-robot-brain/chapter-4-path-planning',
        'ai-robot-brain/summary',
      ],
    },
    'references/index',
  ],
  vlaModuleSidebar: [
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Research',
      items: [
        'module-4-vla/overview',
        'module-4-vla/chapter-1-foundations',
        'module-4-vla/chapter-2-voice-action',
        'module-4-vla/chapter-3-cognitive-planning',
        'module-4-vla/chapter-4-capstone',
        'module-4-vla/summary',
      ],
    },
    'module-4-vla/references',
  ],
};

module.exports = sidebars;