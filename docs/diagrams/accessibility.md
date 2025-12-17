# Diagram Accessibility Guidelines

This document outlines the accessibility features implemented in our ROS 2 diagrams to ensure they are usable by individuals with various abilities and assistive technologies.

## ROS 2 Architecture Diagram (ros2-architecture.svg)

**Title:** ROS 2 Architecture: The Robotic Nervous System
**Description:** Diagram showing ROS 2 nodes, topics, and message flow

### Accessibility Features:
- Contains `<title>` element describing the diagram content
- Includes `<desc>` element with detailed description of the diagram
- Uses color contrast that meets WCAG guidelines
- Includes legend to explain color coding
- Text elements are appropriately sized for readability
- Arrows clearly indicate message flow direction

### Content Summary:
The diagram shows 4 main nodes (Sensor, Processing, Control, Actuator) connected by topics that represent the flow of information in a ROS 2 system, similar to how the nervous system coordinates communication between different parts of the body.

## Node-Topic-Service Flow Diagram (node-topic-service-flow.svg)

**Title:** ROS 2 Communication Patterns: Nodes, Topics, and Services
**Description:** Diagram showing how nodes communicate via topics and services in ROS 2

### Accessibility Features:
- Contains `<title>` element describing the diagram content
- Includes `<desc>` element with detailed description of the diagram
- Uses distinct colors and patterns to differentiate communication types
- Includes comprehensive legend explaining all elements
- Text elements are clearly labeled
- Arrows with markers indicate direction of communication

### Content Summary:
The diagram illustrates three main communication patterns in ROS 2:
1. Publish-Subscribe pattern with publishers, topics, and multiple subscribers
2. Service pattern with client, service, and server
3. Action pattern with goal, feedback, and result flows