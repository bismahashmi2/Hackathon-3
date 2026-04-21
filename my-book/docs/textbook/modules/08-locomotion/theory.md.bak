---
id: "08"
title: "Locomotion"
slug: "locomotion"
week: 8
difficulty: intermediate
prerequisites: ["05"]
learning_objectives:
  - "Calculate Zero Moment Point and analyze bipedal stability"
  - "Implement gait generation algorithms for bipedal walking"
  - "Design whole-body controllers for dynamic balance"
  - "Apply capture point dynamics for robust walking control"
estimated_hours: 14
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 08: Locomotion

## Introduction

Bipedal locomotion is perhaps the most challenging aspect of humanoid robotics. This module covers the dynamics, stability analysis, and control strategies for walking robots.

## Section 1: Stability Criteria

### 1.1 Zero Moment Point

<definition id="def-zmp">
**Zero Moment Point (ZMP)**: The point on the ground where the sum of moments due to gravity and inertial forces equals zero. When ZMP is within the support polygon, the robot is dynamically stable.
</definition>


$$\mathbf{p}_{ZMP} = \frac{\sum_i m_i(\ddot{z}_i + g)p_i - \sum_i m_i\ddot{p}_i z_i}{\sum_i m_i(\ddot{z}_i + g)}$$


### 1.2 Support Polygon

The convex hull of contact points defines stable ZMP locations.

## Section 2: Gait Generation

### 2.1 Central Pattern Generators

Neural-inspired oscillators for rhythmic movement:

```python
class CPGOscillator:
    def __init__(self, frequency, amplitude):
        self.freq = frequency
        self.amp = amplitude
        self.phase = 0

    def step(self, dt):
        self.phase += 2 * np.pi * self.freq * dt
        return self.amp * np.sin(self.phase)
```

### 2.2 Preview Control

Using ZMP trajectory preview for stable walking.

<warning>
ZMP-based control assumes flat ground. On uneven terrain, additional measures (foot placement adaptation, compliant control) are needed.
</warning>

## Section 3: Whole-Body Control

### 3.1 Task-Space Inverse Dynamics

Hierarchical control framework:

<equation id="eq-wbc">
$$\boldsymbol{\tau} = \mathbf{M}\ddot{\mathbf{q}}_d + \mathbf{h} - \mathbf{J}_c^T\mathbf{f}_c$$
</equation>

### 3.2 Operational Space Control

Decoupling tasks in Cartesian space.

## Section 4: Capture Point Dynamics

### 4.1 Linear Inverted Pendulum Model

Simplified dynamics for walking:

<definition id="def-capture-point">
**Capture Point**: The location where the robot must step to come to rest without falling. Enables push recovery and dynamic balance.
</definition>

### 4.2 Step Planning

Real-time foot placement based on capture point.

## Summary

Key takeaways:
1. ZMP within support polygon ensures stability
2. Gait generation can use CPGs or optimization
3. Whole-body control coordinates all DOFs
4. Capture point enables reactive balance control

## Key Concepts

- **ZMP**: Zero Moment Point stability criterion
- **Support Polygon**: Convex hull of contact points
- **Whole-Body Control**: Coordinating all robot DOFs
- **Capture Point**: Reactive balance metric

## Further Reading

1. Vukobratovic, M. & Borovac, B. (2004). "Zero-Moment Point—Thirty Five Years of Its Life"
2. Kajita, S. et al. (2014). "Introduction to Humanoid Robotics"
