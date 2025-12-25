---
id: "04"
title: "Sensors and Perception"
slug: "sensors-perception"
week: 4
difficulty: beginner
prerequisites: ["01"]
learning_objectives:
  - "Compare different sensor modalities and their applications in humanoid robotics"
  - "Implement sensor fusion algorithms including Kalman and complementary filters"
  - "Process camera and depth sensor data for object detection and pose estimation"
  - "Design perception systems that handle sensor noise and uncertainty"
estimated_hours: 12
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 04: Sensors and Perception

## Introduction

Perception is how robots understand their environment. This module covers the sensors used in humanoid robots and the algorithms that process sensor data into actionable information.

## Section 1: Proprioceptive Sensors

### 1.1 Joint Encoders

<definition id="def-encoder">
**Encoder**: A sensor that measures angular position or velocity of a motor shaft or joint.
</definition>

Types:
- Incremental encoders (relative position)
- Absolute encoders (absolute position)
- Resolution: typically 12-18 bits

### 1.2 Inertial Measurement Units (IMU)

IMUs combine:
- Accelerometers (linear acceleration)
- Gyroscopes (angular velocity)
- Magnetometers (heading, optional)

```python
class IMU:
    def __init__(self):
        self.accel = np.zeros(3)  # m/s^2
        self.gyro = np.zeros(3)   # rad/s

    def estimate_orientation(self, dt):
        # Complementary filter
        accel_angle = np.arctan2(self.accel[1], self.accel[2])
        gyro_angle = self.orientation + self.gyro[0] * dt
        self.orientation = 0.98 * gyro_angle + 0.02 * accel_angle
```

## Section 2: Exteroceptive Sensors

### 2.1 Cameras

RGB cameras provide:
- Object detection and recognition
- Visual servoing
- SLAM features

### 2.2 Depth Sensors

<definition id="def-rgbd">
**RGB-D Camera**: A sensor combining color imagery with per-pixel depth measurement, using structured light or time-of-flight.
</definition>

### 2.3 LIDAR

LIDAR provides precise distance measurements:
- 2D: Single scan plane
- 3D: Full point cloud
- Range: 10-200m typical

## Section 3: Sensor Fusion

### 3.1 Kalman Filter

<equation id="eq-kalman">
$$\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k(\mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1})$$
</equation>

### 3.2 Extended Kalman Filter

For nonlinear systems, linearize about current estimate.

<warning>
Sensor fusion requires careful calibration. Misaligned sensors will produce inconsistent estimates regardless of filter quality.
</warning>

## Section 4: Perception Pipelines

### 4.1 Object Detection

Modern approaches use deep learning:
- YOLO, SSD for real-time detection
- Point cloud processing for 3D objects

### 4.2 Pose Estimation

Estimating 6-DOF object pose enables manipulation:

```python
def estimate_object_pose(rgb_image, depth_image, object_model):
    # Detect object in RGB
    bbox = object_detector(rgb_image)

    # Extract point cloud for object
    points = depth_to_points(depth_image, bbox)

    # Align to model
    pose = icp(points, object_model)
    return pose
```

## Summary

Key takeaways:
1. Proprioceptive sensors measure internal state (joints, orientation)
2. Exteroceptive sensors measure the external world (cameras, LIDAR)
3. Sensor fusion combines noisy measurements optimally
4. Modern perception uses deep learning for robust detection

## Key Concepts

- **Proprioception**: Internal state sensing
- **Exteroception**: External world sensing
- **Sensor Fusion**: Combining multiple sensor readings
- **Kalman Filter**: Optimal state estimation for linear systems

## Further Reading

1. Thrun, S., Burgard, W., & Fox, D. (2005). "Probabilistic Robotics"
2. Szeliski, R. (2022). "Computer Vision: Algorithms and Applications"
