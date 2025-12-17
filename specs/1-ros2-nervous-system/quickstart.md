# Quickstart Guide: ROS 2 as the Robotic Nervous System Module

**Feature**: 1-ros2-nervous-system
**Date**: 2025-12-16

## Overview

This guide provides a quick setup and development workflow for the ROS 2 educational module. The module is designed as a Docusaurus-based documentation resource that teaches ROS 2 concepts with a focus on connecting AI agents to humanoid robot control systems.

## Prerequisites

- Node.js 18+ and npm/yarn
- Python 3.8+ with pip
- ROS 2 (Humble Hawksbill or later) installed and sourced
- Basic knowledge of ROS 2 concepts (optional but helpful)

## Local Development Setup

### 1. Clone and Initialize Repository

```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Navigate to the project directory
cd <project-directory>
```

### 2. Install Docusaurus Dependencies

```bash
# Install Docusaurus and related packages
npm install
```

### 3. Set up ROS 2 Environment

```bash
# Source ROS 2 (example for Humble Hawksbill on Ubuntu)
source /opt/ros/humble/setup.bash

# If using a ROS 2 workspace, source it as well
source install/setup.bash  # if you have a local workspace
```

### 4. Install Python Dependencies

```bash
# Install rclpy and other Python dependencies
pip3 install rclpy
```

## Development Workflow

### 1. Start Docusaurus Development Server

```bash
# Start the development server
npm run start

# This will start a local server at http://localhost:3000
```

### 2. Edit Module Content

The module content is organized in the `docs/modules/ros2-nervous-system/` directory:

```
docs/modules/ros2-nervous-system/
├── index.md              # Module overview
├── ros2-fundamentals.md  # Chapter 1: ROS 2 fundamentals
├── python-ros-bridge.md  # Chapter 2: Python-ROS bridge with rclpy
└── urdf-humanoids.md     # Chapter 3: URDF for humanoids
```

### 3. Add or Update Diagrams

Diagram files should be placed in the `docs/diagrams/` directory:

```
docs/diagrams/
├── ros2-architecture.svg
├── node-topic-service-flow.svg
└── humanoid-urdf-structure.svg
```

### 4. Add or Update Code Examples

Code examples should be placed in the `src/components/code-examples/` directory:

```
src/components/code-examples/
├── ros2-publisher.py
├── ros2-subscriber.py
└── ros2-service-client.py
```

## Building and Testing

### 1. Build the Documentation Site

```bash
# Build the static site
npm run build
```

### 2. Test Code Examples

```bash
# Navigate to the code examples directory
cd src/components/code-examples/

# Test a specific Python example (make sure ROS 2 is sourced)
python3 ros2-publisher.py
```

### 3. Validate Citations

```bash
# Run citation validation tests
npm run test:content
```

## Content Guidelines

### Writing Style
- Maintain Flesch-Kincaid Grade 10-12 readability level
- Use clear, concise language appropriate for students with basic Python knowledge
- Follow APA citation format for all references
- Include at least 15 sources with 50%+ peer-reviewed

### Code Examples
- Keep examples minimal and focused on the concept being taught
- Include comments explaining key parts of the code
- Ensure all examples are tested and functional
- Use Python with rclpy as specified

### Diagrams
- Provide alt text for accessibility
- Use SVG format when possible for scalability
- Include captions explaining what the diagram illustrates
- Keep diagrams simple and focused on key concepts

## Quality Validation

### 1. Run Content Validation Tests

```bash
# Run all content validation tests
npm run test:content

# Run specific validation checks
npm run test:citation-audit
npm run test:plagiarism-scan  # if available
npm run test:build-validation
```

### 2. Manual Review Checklist

Before finalizing content:

- [ ] All technical claims verified against official ROS 2 documentation
- [ ] All code examples tested and functional
- [ ] All diagrams have appropriate alt text
- [ ] All references properly cited in APA format
- [ ] Content maintains consistent terminology
- [ ] Learning objectives clearly stated and met
- [ ] Content meets readability requirements

## Deployment

### 1. Build for Production

```bash
# Build the final site
npm run build
```

### 2. Deploy to GitHub Pages

The site is configured to deploy automatically via GitHub Actions when changes are pushed to the main branch.

## Troubleshooting

### Common Issues

1. **ROS 2 not found when running Python examples**:
   - Ensure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`
   - Verify rclpy is installed: `pip3 install rclpy`

2. **Docusaurus build errors**:
   - Clear cache: `npm run clear`
   - Reinstall dependencies: `npm install`

3. **Code examples not working**:
   - Verify Python version compatibility (3.8+)
   - Check ROS 2 distribution compatibility
   - Ensure proper ROS 2 environment setup