# ROS 2 Installation Guide

This project assumes ROS 2 Humble Hawksbill is installed on the system. For educational purposes, we provide guidance on setting up the ROS 2 environment.

For actual implementation and testing of code examples, users should follow the official ROS 2 installation guide for their platform:

- Ubuntu: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Windows: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
- macOS: https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html

## Prerequisites

- Python 3.8 or higher
- Node.js 18+ (for Docusaurus documentation)
- Git

## Environment Setup

After installing ROS 2, source the setup script:

```bash
# For Ubuntu/Debian
source /opt/ros/humble/setup.bash

# For Windows (in PowerShell)
[Environment]::SetEnvironmentVariable('AMENT_PYTHON_EXEC_LIBRARY', 'python3.8')
```