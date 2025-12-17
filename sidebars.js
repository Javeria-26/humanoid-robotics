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
};

module.exports = sidebars;