---
id: "03"
title: "Kinematics Fundamentals"
slug: "kinematics-fundamentals"
week: 3
difficulty: beginner
prerequisites: ["02"]
learning_objectives:
  - "Derive forward kinematics using Denavit-Hartenberg parameters"
  - "Solve inverse kinematics problems using analytical and numerical methods"
  - "Compute Jacobian matrices and analyze manipulator singularities"
  - "Apply coordinate transformations using rotation matrices and quaternions"
estimated_hours: 14
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 03: Kinematics Fundamentals

## Introduction

Kinematics describes the geometry of motion without considering forces. For humanoid robots, kinematics answers: "Given joint angles, where is the hand?" (forward kinematics) and "What joint angles place the hand here?" (inverse kinematics).

## Section 1: Coordinate Transformations

### 1.1 Rotation Matrices

<definition id="def-rotation-matrix">
**Rotation Matrix**: A 3×3 orthogonal matrix $\mathbf{R}$ with determinant +1 that transforms vectors between coordinate frames.
</definition>

Properties:
- $\mathbf{R}^T = \mathbf{R}^{-1}$
- $\det(\mathbf{R}) = 1$
- Columns are orthonormal

### 1.2 Homogeneous Transformations

<equation id="eq-homogeneous">
$$\mathbf{T} = \begin{bmatrix} \mathbf{R} & \mathbf{p} \\ \mathbf{0}^T & 1 \end{bmatrix}$$
</equation>

## Section 2: Forward Kinematics

### 2.1 DH Parameters

<definition id="def-dh">
**Denavit-Hartenberg (DH) Parameters**: A standard convention using four parameters ($a$, $\alpha$, $d$, $\theta$) to describe the relationship between consecutive links.
</definition>

```python
def dh_transform(a, alpha, d, theta):
    """Compute DH transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,     sa,     ca,    d],
        [0,      0,      0,    1]
    ])
```

### 2.2 Chain Multiplication

The end-effector pose is computed by chaining transforms:

<equation id="eq-fk">
$$\mathbf{T}_{0}^{n} = \mathbf{T}_{0}^{1} \cdot \mathbf{T}_{1}^{2} \cdots \mathbf{T}_{n-1}^{n}$$
</equation>

## Section 3: Inverse Kinematics

### 3.1 Analytical Solutions

For simple kinematic chains, closed-form solutions exist using geometric relationships.

### 3.2 Numerical Methods

For complex robots, iterative methods are required:

```python
def inverse_kinematics_jacobian(robot, target_pose, q_init, max_iter=100):
    """Solve IK using Jacobian pseudo-inverse."""
    q = q_init.copy()
    for _ in range(max_iter):
        current_pose = robot.forward_kinematics(q)
        error = pose_error(target_pose, current_pose)
```python
```python
        if np.linalg.norm(error) < 1e-6:
```
```
            break
        J = robot.jacobian(q)
        dq = np.linalg.pinv(J) @ error
        q += dq
    return q
```

<warning>
Inverse kinematics may have multiple solutions, no solutions (unreachable targets), or infinite solutions (redundant manipulators).
</warning>

## Section 4: Jacobians and Singularities

### 4.1 Velocity Kinematics


$$\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}$$


### 4.2 Singularities

When $\det(\mathbf{J}) = 0$, the robot is in a singular configuration where certain directions of motion are impossible.

## Summary

Key takeaways:
1. Forward kinematics maps joint angles to end-effector pose
2. DH parameters provide a systematic parameterization
3. Inverse kinematics finds joint angles for desired poses
4. The Jacobian relates joint and Cartesian velocities

## Key Concepts

- **Forward Kinematics**: Computing end-effector pose from joint angles
- **Inverse Kinematics**: Computing joint angles from desired pose
- **Jacobian**: Matrix relating joint and task space velocities
- **Singularity**: Configuration where Jacobian loses rank

## Further Reading

1. Murray, R., Li, Z., & Sastry, S. (1994). "A Mathematical Introduction to Robotic Manipulation"
2. Spong, M., Hutchinson, S., & Vidyasagar, M. (2020). "Robot Modeling and Control"
