# Gazebo Robot Models (ROS2)

This directory contains URDF/SDF robot model files for use with Gazebo and ROS2.

## Available Models

| Model | File | Description | Modules |
|-------|------|-------------|---------|
| TurtleBot 4 | `turtlebot4/` | Mobile platform with ROS2 support | 09, 10 |
| Humanoid ROS | `humanoid_ros/` | ROS2-compatible humanoid | 12-14 |
| OpenManipulator | `openmanipulator/` | Low-cost ROS2 arm | 09 |

## Directory Structure

Each robot model follows this structure:

```
robot_name/
├── urdf/
│   └── robot.urdf.xacro
├── meshes/
│   ├── visual/
│   └── collision/
├── config/
│   └── controllers.yaml
└── launch/
    └── spawn.launch.py
```

## ROS2 Integration

Models are designed for ROS2 Humble and include:
- URDF/xacro descriptions
- Gazebo plugins for sensors and actuators
- Controller configuration for ros2_control
- Launch files for simulation

## Docker Usage

```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  physical-ai-textbook:ros2-humble \
  ros2 launch robot_name spawn.launch.py
```

## References

- [Gazebo ROS2 Integration](https://gazebosim.org/docs/harmonic/ros2_integration)
- [URDF Specification](http://wiki.ros.org/urdf/XML)
