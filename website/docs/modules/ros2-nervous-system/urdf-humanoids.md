---
sidebar_position: 4
---

# URDF for Humanoids: Modeling Robot Structure

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the basics of URDF (Unified Robot Description Format)
- Identify links, joints, and other components in a URDF file
- Understand how humanoid robot structure is represented in URDF
- Read and interpret URDF files for humanoid robots
- Create simple URDF models for basic robot structures

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial characteristics and visual appearance.

For humanoid robots, URDF provides a standardized way to represent the complex structure of robots with human-like characteristics, including torso, head, arms, and legs.

## URDF Components

### Links

Links represent the rigid parts of a robot. In a humanoid robot, links might include:
- Torso
- Head
- Upper arm, lower arm, hand
- Upper leg, lower leg, foot
- Any other rigid components

Each link can have:
- Visual properties (shape, color, mesh)
- Collision properties (shape for collision detection)
- Inertial properties (mass, center of mass, inertia tensor)

### Joints

Joints define how links connect and move relative to each other. In a humanoid robot, joints represent:
- Shoulder joints (connecting torso to arms)
- Elbow joints (connecting upper and lower arms)
- Hip joints (connecting torso to legs)
- Knee joints (connecting upper and lower legs)
- Wrist, ankle, and other joints

Joint types include:
- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint with unlimited range
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement (used to connect rigidly)
- **Floating**: 6 DOF (degrees of freedom)
- **Planar**: Movement in a plane

### Materials and Colors

URDF allows defining materials and colors for visual representation of the robot.

## Example URDF Structure

Here's a simplified example of a humanoid robot structure in URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link (torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left arm (simplified) -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1"/>
  </joint>
</robot>
```

## Humanoid Robot Structure in URDF

Humanoid robots have a specific structure that typically includes:

### Torso
- The central body of the robot
- Usually the heaviest and most stable part
- Connects to head, arms, and legs

### Head
- Contains sensors (cameras, microphones, etc.)
- Connected to torso via neck joint
- Often has limited movement compared to human neck

### Arms
- Shoulders connect arms to torso
- Upper arms, lower arms, and hands
- Shoulders typically have multiple DOF for wide range of motion

### Legs
- Hips connect legs to torso
- Upper legs, lower legs, and feet
- Knees allow for flexion and extension

## Reading URDF Files

When reading a URDF file, look for:

1. **Robot tag**: The root element containing the robot name
2. **Links**: Named elements describing rigid parts
3. **Joints**: Connections between links with types and limits
4. **Visual elements**: How the robot appears in simulation
5. **Collision elements**: How the robot interacts physically
6. **Inertial elements**: Physical properties for physics simulation

## URDF Tools in ROS 2

ROS 2 provides several tools to work with URDF:

- **robot_state_publisher**: Publishes joint states and transforms for visualization
- **rviz2**: Visualizes robot models with joint positions
- **xacro**: XML macro language that extends URDF with macros and expressions

## Best Practices

- Use meaningful names for links and joints
- Follow consistent naming conventions
- Include proper inertial properties for accurate simulation
- Use appropriate joint limits to prevent damage
- Organize complex robots hierarchically
- Use xacro to reduce repetition in complex models

## Summary

URDF provides a standardized way to describe robot models in ROS 2. For humanoid robots, it allows precise definition of the complex structure with multiple links and joints. Understanding URDF is crucial for working with humanoid robots in simulation and real-world applications.

To see how ROS 2 fundamentals apply to these models, review the [ROS 2 Fundamentals](./ros2-fundamentals) chapter. For understanding how Python can be used to interact with these models, see the [Python-ROS Bridge](./python-ros-bridge) chapter.