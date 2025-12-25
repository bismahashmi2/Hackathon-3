---
id: "02"
title: "Rigid Body Dynamics"
slug: "rigid-body-dynamics"
week: 2
difficulty: beginner
prerequisites: ["01"]
learning_objectives:
  - "Apply Newton-Euler equations to compute forces and torques on rigid bodies"
  - "Calculate mass properties including center of mass and inertia tensors"
  - "Derive equations of motion for multi-body systems using free body diagrams"
  - "Implement numerical integration methods for dynamic simulation"
estimated_hours: 12
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 02: Rigid Body Dynamics

## Introduction

Understanding rigid body dynamics is fundamental to robotics. This module covers the physics that governs how robots move and interact with their environment, providing the mathematical foundation for control and simulation.

## Section 1: Newton-Euler Formulation

### 1.1 Newton's Laws for Rigid Bodies

<definition id="def-rigid-body">
**Rigid Body**: A solid object in which the distance between any two points remains constant regardless of external forces applied.
</definition>

For a rigid body, Newton's second law extends to:

<equation id="eq-newton-linear">
$$\mathbf{F} = m\mathbf{a}_{cm}$$
</equation>

where $\mathbf{F}$ is the net force, $m$ is mass, and $\mathbf{a}_{cm}$ is acceleration of the center of mass.

### 1.2 Euler's Equations for Rotation

The rotational equivalent involves angular momentum:

<equation id="eq-euler-rotation">
$$\boldsymbol{\tau} = \mathbf{I}\dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I}\boldsymbol{\omega})$$
</equation>

where:
- $\boldsymbol{\tau}$ is the net torque
- $\mathbf{I}$ is the inertia tensor
- $\boldsymbol{\omega}$ is angular velocity

## Section 2: Mass Properties

### 2.1 Center of Mass

<equation id="eq-com">
$$\mathbf{r}_{cm} = \frac{1}{M}\sum_{i} m_i \mathbf{r}_i$$
</equation>

### 2.2 Inertia Tensor

The inertia tensor captures rotational inertia about all axes:

```python
def compute_inertia_tensor(masses, positions):
    """Compute inertia tensor for a set of point masses."""
    I = np.zeros((3, 3))
    for m, r in zip(masses, positions):
        r_sq = np.dot(r, r)
        I += m * (r_sq * np.eye(3) - np.outer(r, r))
    return I
```

## Section 3: Equations of Motion

### 3.1 Free Body Diagrams

Every force and torque acting on a body must be identified:
- Gravitational forces
- Contact forces (normal and friction)
- Joint reaction forces
- External applied forces

### 3.2 Multi-Body Systems

For articulated robots, the dynamics become:

<equation id="eq-manipulator-dynamics">
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q}) = \boldsymbol{\tau}$$
</equation>

<warning>
The mass matrix $\mathbf{M}(\mathbf{q})$ is configuration-dependent and must be recomputed at each time step.
</warning>

## Summary

Key takeaways:
1. Newton-Euler equations govern rigid body motion
2. Mass properties (COM, inertia) are essential for dynamics
3. Multi-body dynamics leads to coupled differential equations
4. Numerical integration enables simulation

## Key Concepts

- **Rigid Body**: Idealized solid with fixed shape
- **Inertia Tensor**: 3×3 matrix describing rotational inertia
- **Equations of Motion**: ODEs describing system dynamics
- **Newton-Euler Method**: Algorithm for computing joint torques

## Further Reading

1. Featherstone, R. (2008). "Rigid Body Dynamics Algorithms"
2. Craig, J. (2005). "Introduction to Robotics: Mechanics and Control"
