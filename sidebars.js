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
};

module.exports = sidebars;