---
id: "09"
title: "ROS2 Integration"
slug: "ros2-integration"
week: 9
difficulty: intermediate
prerequisites: ["04", "06"]
learning_objectives:
  - "Design robot systems using ROS2 nodes, topics, services, and actions"
  - "Implement robot state publishing and visualization"
  - "Integrate sensors and actuators through ROS2 interfaces"
  - "Deploy robot software using launch files and parameters"
estimated_hours: 12
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 09: ROS2 Integration

## Introduction

ROS2 (Robot Operating System 2) is the industry-standard middleware for robot software development. This module covers the architecture and practical use of ROS2 for humanoid robotics.

## Section 1: ROS2 Architecture

### 1.1 Communication Primitives

<definition id="def-topic">
**Topic**: A named bus for asynchronous, many-to-many communication using publish-subscribe pattern.
</definition>

<definition id="def-service">
**Service**: Synchronous request-response communication for discrete queries.
</definition>

<definition id="def-action">
**Action**: Long-running tasks with feedback and cancellation support.
</definition>

### 1.2 Node Design

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_state)

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['hip', 'knee', 'ankle']
        msg.position = self.get_joint_positions()
        self.pub.publish(msg)
```

## Section 2: Robot Description

### 2.1 URDF/XACRO

<definition id="def-urdf">
**URDF (Unified Robot Description Format)**: XML format describing robot kinematics, dynamics, and visual/collision geometry.
</definition>

### 2.2 Robot State Publisher

Broadcasting transforms from joint states.

## Section 3: ros2_control

### 3.1 Hardware Interfaces

Standardized interfaces for actuators and sensors.

### 3.2 Controllers

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController
```

<warning>
Real-time control requires careful system configuration. ROS2 provides tools but doesn't guarantee real-time by default.
</warning>

## Section 4: Navigation and Manipulation

### 4.1 Navigation Stack

- Map server
- Localization (AMCL)
- Path planning
- Path following

### 4.2 MoveIt 2

Motion planning framework for manipulation.

## Summary

Key takeaways:
1. ROS2 provides standard communication patterns
2. URDF describes robot structure
3. ros2_control standardizes hardware interfaces
4. MoveIt 2 enables motion planning integration

## Key Concepts

- **Topic**: Publish-subscribe communication
- **Service**: Request-response communication
- **Action**: Long-running task with feedback
- **URDF**: Robot description format

## Further Reading

1. ROS2 Documentation: https://docs.ros.org/en/humble/
2. The Construct ROS2 Courses
3. MoveIt 2 Tutorials
