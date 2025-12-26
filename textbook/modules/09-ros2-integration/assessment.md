---
module_id: "09"
title: "Assessment Package: ROS2 Integration"
---

# Assessment Package: Module 09 - ROS2 Integration

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice + short answer | 40 minutes |
| Lab Exercises | 35% | ROS2 implementation | 3 labs |
| Integration Project | 35% | Complete ROS2 system | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

---

## Theory Quiz

**Time Limit**: 40 minutes
**Passing Score**: 70%
**Attempts**: 2

### Section A: Multiple Choice (40 points)

**Q1.** In ROS2, which communication pattern is best for continuous sensor data?
- a) Services
- b) Actions
- c) Topics
- d) Parameters

**Q2.** The primary difference between ROS1 and ROS2 communication is:
- a) ROS2 uses XML instead of messages
- b) ROS2 uses DDS instead of a custom protocol
- c) ROS2 only supports Python
- d) ROS2 requires a master node

**Q3.** QoS "reliability" setting RELIABLE vs BEST_EFFORT affects:
- a) Message serialization speed
- b) Whether delivery is guaranteed or not
- c) The message buffer size
- d) The communication protocol

**Q4.** In tf2, a static transform is used when:
- a) The transform changes slowly
- b) The transform never changes
- c) The transform is computed on-demand
- d) The transform involves velocity

**Q5.** The tf2 buffer stores transforms to enable:
- a) Faster computation
- b) Transform interpolation at past times
- c) Multiple parallel lookups
- d) Transform compression

**Q6.** In the ROS2-Gazebo bridge, "GZ_TO_ROS" direction means:
- a) ROS publishes to Gazebo
- b) Gazebo publishes to ROS
- c) Bidirectional communication
- d) Service call from ROS

**Q7.** For robot control loops, which QoS history setting is typically best?
- a) KEEP_ALL with large depth
- b) KEEP_LAST with depth 1
- c) KEEP_LAST with depth 100
- d) No history (transient)

**Q8.** A ROS2 lifecycle node in the "inactive" state:
- a) Cannot receive messages
- b) Is processing normally
- c) Is configured but not executing
- d) Is shutting down

**Q9.** The ros2_control framework provides:
- a) Only joint state publishing
- b) Hardware abstraction for controllers
- c) Only simulation interfaces
- d) Message type definitions

**Q10.** When use_sim_time is true, ROS2 nodes:
- a) Run faster than real-time
- b) Get time from /clock topic
- c) Ignore timestamps
- d) Use hardware clocks

### Section B: Short Answer (60 points)

**Q11.** (15 points) Explain the tf2 transform tree for a mobile manipulator:

a) Draw a diagram showing the transform tree for a robot with:
   - Base frame on mobile platform
   - 6-DOF arm mounted on base
   - Camera on end-effector
   - LIDAR on base

b) Which transforms would be static vs. dynamic?

c) Write the tf2 lookup call (Python) to get the camera pose in the LIDAR frame.

**Q12.** (15 points) Design QoS profiles for a robot system:

a) Choose QoS settings (reliability, durability, history, depth) for:
   - Joint state feedback (100 Hz)
   - Emergency stop command
   - Camera images (30 fps)
   - Robot status messages (1 Hz)

b) Justify each choice.

**Q13.** (15 points) A ROS2 node receives joint commands on `/cmd_joint` and publishes joint states on `/joint_states`. The control loop runs at 500 Hz.

a) Should the node use a timer callback or subscriber callback for the control loop? Why?

b) What happens if command messages arrive faster than 500 Hz?

c) How would you handle the case where no command has been received recently?

**Q14.** (15 points) Explain the simulation time system:

a) Why is use_sim_time important when using Gazebo?

b) What problems occur if some nodes use sim time and others don't?

c) How does tf2 handle time synchronization?

---

## Lab Exercises

### Lab 09-01: ROS2 Fundamentals (30% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Publisher | Correct topic, rate, QoS | Works with minor issues | Partial functionality | Non-functional |
| Subscriber | Proper message handling | Works but inefficient | Missing features | Non-functional |
| Service | Working request/response | Minor bugs | Partial implementation | Non-functional |
| Launch File | Complete system launch | Missing some nodes | Syntax errors | Non-functional |
| QoS Understanding | Correct analysis of tradeoffs | Basic understanding | Incomplete analysis | No understanding |

### Lab 09-02: tf2 Transforms (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Static Broadcaster | All sensor frames correct | Minor frame errors | Some frames missing | Non-functional |
| Dynamic Broadcaster | Proper joint transforms | Timing issues | Incorrect transforms | Non-functional |
| Transform Listener | Correct lookups, error handling | Basic lookups work | Missing error handling | Non-functional |
| Visualization | RViz shows all frames | Most frames visible | Partial visualization | No visualization |
| Integration | Complete tree working | Minor gaps | Significant gaps | Disconnected tree |

### Lab 09-03: Gazebo Integration (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| World Setup | Physics, objects, lighting | Working but basic | Partial setup | Non-functional |
| Robot Model | Joints, sensors, plugins | Missing some features | Basic model only | Non-functional |
| ROS Bridge | All topics bridged | Most topics working | Some topics only | No bridge |
| Controller | Tracking error < 5% | Tracking error < 15% | Unstable control | Non-functional |
| Launch System | Complete integration | Missing components | Errors in launch | Non-functional |

---

## Integration Project

### Project: Complete Robot Control System

**Objective**: Build a complete ROS2 system that controls a robot arm to pick and place objects using simulated sensors.

**Duration**: 1 week
**Deliverables**: Code repository + technical report

### Requirements

1. **ROS2 Infrastructure** (25%)
   - Proper node architecture with clear responsibilities
   - Appropriate QoS settings for all topics
   - Launch file that starts complete system
   - Parameter configuration via YAML

2. **Coordinate Frames** (25%)
   - Complete tf2 tree from world to all sensors
   - Static transforms for sensor mounts
   - Dynamic transforms from joint states
   - Correct transform lookups in perception

3. **Gazebo Simulation** (25%)
   - Working robot model with actuators
   - Simulated camera and/or LIDAR
   - Bridge configuration for all data
   - Reasonable physics behavior

4. **Application Logic** (25%)
   - Detect object using simulated sensor
   - Plan path to object
   - Execute pick operation
   - Place at target location

### Evaluation Scenarios

1. **Single pick-place**: Object at known position
2. **Perception-based**: Object position from camera
3. **Multiple objects**: Sequential pick-place
4. **Robust operation**: Handle sensor noise, minor errors

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Node Architecture | 25 | Clean, modular design |
| Transform System | 25 | Complete, correct tree |
| Simulation | 25 | Working Gazebo integration |
| Task Completion | 15 | Successful pick-place |
| Documentation | 10 | Clear README, architecture diagram |
| **Total** | **100** | |

### Bonus Challenges (+10 points each)

- Real-time performance analysis
- Multi-robot coordination
- Dynamic obstacle avoidance

---

## Ethics Discussion

### Prompt

*In a 500-word reflection, address the following scenario:*

A startup develops home robots using ROS2 as their middleware. To improve their product, they collect:
- Sensor data (camera, microphone) for debugging
- Usage patterns (which features are used)
- Error logs (when things go wrong)
- Environment maps (layout of customer homes)

This data is uploaded to cloud servers for analysis. The data helps the company:
- Fix bugs faster
- Understand user needs
- Train machine learning models
- Improve navigation

However, customers raise concerns:
- Camera data could capture private activities
- Audio might record conversations
- Home maps reveal security vulnerabilities
- Usage patterns profile daily routines

The company's EULA mentions data collection, but users rarely read it.

**Address the following:**

1. What data, if any, should the company be allowed to collect? How should consent be obtained?

2. Should ROS2 middleware provide built-in privacy controls that limit what data can be transmitted off-device?

3. Who bears responsibility for data breaches—the company, the middleware developers (ROS2 community), or the customers who accepted the EULA?

4. How should the robot software community balance the benefits of data-driven improvement against privacy risks?

### Rubric

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Data Collection | Nuanced analysis of different data types | Recognizes some distinctions | Treats all data the same | No analysis |
| Consent | Specific, practical proposals | General recommendations | Vague suggestions | Ignores consent |
| Responsibility | Multi-stakeholder analysis | Considers some parties | Single-party focus | No analysis |
| Balance | Thoughtful tradeoffs | Acknowledges tradeoffs | One-sided | Ignores tradeoffs |
| Writing | Clear, organized | Minor issues | Some problems | Unclear |

---

## Answer Key (Instructor Access Only)

### Quiz Answers

**Section A:**
1. c) Topics
2. b) ROS2 uses DDS instead of a custom protocol
3. b) Whether delivery is guaranteed or not
4. b) The transform never changes
5. b) Transform interpolation at past times
6. b) Gazebo publishes to ROS
7. b) KEEP_LAST with depth 1
8. c) Is configured but not executing
9. b) Hardware abstraction for controllers
10. b) Get time from /clock topic

**Section B:**

**Q11:**
a) Transform tree diagram:
```
world
  └── map
      └── odom
          └── base_link
              ├── lidar_link (static)
              └── arm_base_link (static)
                  └── link1 (dynamic)
                      └── link2 (dynamic)
                          └── ... (dynamic)
                              └── end_effector (dynamic)
                                  └── camera_link (static)
```

b) Static: lidar_link, arm_base_link, camera_link (fixed mounts)
   Dynamic: link1-6, end_effector (change with joint angles)

c) Python tf2 lookup:
```python
transform = self.tf_buffer.lookup_transform(
    'lidar_link',      # target frame
    'camera_link',     # source frame
    rclpy.time.Time(), # latest available
    timeout=rclpy.duration.Duration(seconds=0.1)
)
```

**Q12:**
| Topic | Reliability | Durability | History | Depth | Justification |
|-------|-------------|------------|---------|-------|---------------|
| Joint states | RELIABLE | VOLATILE | KEEP_LAST | 1 | Need every message for control, only latest matters |
| E-stop | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 | Must be delivered, late joiners need current state |
| Camera | BEST_EFFORT | VOLATILE | KEEP_LAST | 3 | OK to drop frames, small buffer for processing |
| Status | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 | Infrequent, late joiners need state |

**Q13:**
a) **Timer callback** is better. Using subscriber callback ties control rate to message rate, which may be inconsistent. Timer ensures exactly 500 Hz regardless of when commands arrive.

b) With KEEP_LAST depth=1, extra messages are dropped. The control loop uses only the most recent command each cycle. This is correct behavior—using stale commands is dangerous.

c) Options:
- Timeout: If no command for N cycles, stop motion
- Default: Hold last position if no new command
- Alert: Publish warning on status topic

**Q14:**
a) Simulation may run faster or slower than real-time. Without use_sim_time, nodes would use wall clock, causing transform lookups to fail (timestamps mismatch) and control loops to run incorrectly.

b) Problems: tf2 lookups fail due to timestamp mismatch. Sensor fusion fails because data has inconsistent timing. Control loops may run at wrong rate or use stale data.

c) tf2 stores transforms with their timestamps and interpolates. When a lookup is requested at time T, tf2 finds the two closest transforms and interpolates. This works correctly as long as all nodes use the same time source (wall or sim).

---

## Export Formats

This assessment package is available in:
- Markdown (this document)
- Canvas LMS import package
- PDF with answer key (instructor version)
- Gradescope autograder configuration
