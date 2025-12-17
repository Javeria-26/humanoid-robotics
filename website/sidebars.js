/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

module.exports = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Tutorial',
      items: [
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-page',
        'tutorial-basics/deploy-your-site',
      ],
    },
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
    {
      type: 'category',
      label: 'Digital Twin Simulation Module',
      items: [
        'modules/module-2/index',
        'modules/module-2/digital-twins-robotics',
        'modules/module-2/physics-simulation',
        'modules/module-2/high-fidelity-env',
        'modules/module-2/sensor-simulation',
        'modules/module-2/glossary',
        'modules/module-2/assessment-questions',
      ],
    },
  ],
};
