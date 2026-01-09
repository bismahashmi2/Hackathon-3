---
module_id: "02"
---

# Assessment Package: Rigid Body Dynamics

## Overview

| Component | Weight | Format |
|-----------|--------|--------|
| Theory Quiz | 15% | Multiple choice + short answer + calculations |
| Lab Assessment | 35% | Jupyter notebooks with auto-grading |
| Simulation Project | 35% | Code submission with test harness |
| Ethics Component | 15% | Written reflection |

## Theory Quiz (15%)

**Time Limit**: 30 minutes
**Passing Score**: 70%
**Attempts Allowed**: 2

### Questions

#### Q1 (Multiple Choice, 2 points)
**Question**: A 10 kg robot arm segment is accelerating at 5 m/s². What force is required to produce this acceleration (ignoring gravity)?

**Options**:
a) 2 N
b) 15 N
c) 50 N
d) 500 N

**Correct Answer**: c
**Explanation**: F = ma = 10 kg × 5 m/s² = 50 N. This is a direct application of Newton's second law.
**Learning Objective**: Apply Newton-Euler equations to compute forces and torques

---

#### Q2 (Multiple Choice, 2 points)
**Question**: What happens to the kinetic energy of a body if its velocity doubles?

**Options**:
a) It doubles
b) It quadruples
c) It halves
d) It remains the same

**Correct Answer**: b
**Explanation**: KE = ½mv². If v → 2v, then KE → ½m(2v)² = 4 × ½mv². Kinetic energy scales with velocity squared.
**Learning Objective**: Analyze energy and momentum in robotic systems

---

#### Q3 (Calculation, 4 points)
**Question**: A humanoid robot has a total mass of 80 kg. It is standing on one foot with its center of mass 0.3 m horizontally from the foot center. Calculate the torque about the foot that gravity exerts on the robot. Should the robot apply an ankle torque in the same or opposite direction to maintain balance?

**Sample Answer**:
- Torque = Force × Distance = (80 kg × 9.81 m/s²) × 0.3 m = 235.4 Nm
- The robot must apply an ankle torque in the OPPOSITE direction to counteract the gravitational torque and maintain balance.

**Rubric**:
- 2 points: Correct calculation
- 2 points: Correct direction reasoning
**Learning Objective**: Compute torque requirements for static equilibrium

---

#### Q4 (Multiple Choice, 2 points)
**Question**: Two robots collide. Robot A (mass 50 kg, velocity 2 m/s) hits stationary Robot B (mass 100 kg). After a perfectly inelastic collision, what is their combined velocity?

**Options**:
a) 0.33 m/s
b) 0.67 m/s
c) 1.0 m/s
d) 2.0 m/s

**Correct Answer**: b
**Explanation**: Conservation of momentum: m₁v₁ + m₂v₂ = (m₁+m₂)v_f
50×2 + 100×0 = 150×v_f → v_f = 100/150 = 0.67 m/s
**Learning Objective**: Analyze momentum conservation in collisions

---

#### Q5 (Short Answer, 4 points)
**Question**: Explain why moment of inertia is called a "tensor" rather than a scalar. What does it mean for a body's rotational response to depend on the axis of rotation?

**Sample Answer**:
Moment of inertia is a tensor because a body's resistance to angular acceleration depends on the axis about which it rotates. A long bar has low inertia about its long axis but high inertia about perpendicular axes. The inertia tensor captures this directional dependence as a 3×3 matrix, where off-diagonal terms represent coupling between axes.
**Learning Objective**: Define moment of inertia and the inertia tensor

---

#### Q6 (Multiple Choice, 2 points)
**Question**: The coefficient of restitution for a collision is 0.5. If a ball strikes a floor at 4 m/s, at what speed does it rebound?

**Options**:
a) 1 m/s
b) 2 m/s
c) 4 m/s
d) 8 m/s

**Correct Answer**: b
**Explanation**: e = v_rebound / v_impact → v_rebound = e × v_impact = 0.5 × 4 = 2 m/s
**Learning Objective**: Understand collision dynamics and restitution

---

#### Q7 (Calculation, 4 points)
**Question**: A robot wheel has moment of inertia 0.05 kg⋅m² and is spinning at 100 rad/s. Calculate its rotational kinetic energy and the torque needed to stop it in 2 seconds.

**Sample Answer**:
- Rotational KE = ½Iω² = ½ × 0.05 × 100² = 250 J
- Angular deceleration α = Δω/Δt = 100/2 = 50 rad/s²
- Torque = Iα = 0.05 × 50 = 2.5 Nm

**Rubric**:
- 2 points: Correct KE calculation
- 2 points: Correct torque calculation
**Learning Objective**: Apply τ = Iα for rotational dynamics

---

**Total Quiz Points**: 20

## Lab Assessment (35%)

### Grading Rubric for Labs 02-01, 02-02, 02-03

| Criterion | Weight | Excellent (90-100%) | Good (70-89%) | Satisfactory (50-69%) | Needs Work (&lt;50%) |
|-----------|--------|---------------------|---------------|----------------------|-------------------|
| Code Correctness | 35% | All code runs correctly | Minor bugs | Some functionality | Major errors |
| Physics Understanding | 30% | Deep understanding demonstrated | Good grasp of concepts | Basic understanding | Misconceptions |
| Data Analysis | 20% | Thorough analysis with insights | Adequate analysis | Basic analysis | Minimal analysis |
| Documentation | 15% | Excellent explanations | Good comments | Basic documentation | Poor documentation |

### Auto-Grading Tests

```python
# tests/lab_02_tests.py
import numpy as np

def test_force_calculation():
    """Verify F=ma implementation."""
    mass = 2.0
    acceleration = 5.0
    expected_force = 10.0
    # Student implementation should return this value
```python
```python
    assert abs(calculate_force(mass, acceleration) - expected_force) < 0.01
```
```

def test_energy_conservation():
    """Verify energy tracking during simulation."""
    initial_energy = 29.43  # 1kg at 3m height
    final_energy = get_final_energy_from_simulation()
    # Allow 10% loss due to contact dissipation
```python
```python
    assert final_energy > 0.9 * initial_energy
```
```

def test_momentum_conservation():
    """Verify momentum conservation in collision."""
    p_before, p_after = simulate_collision_and_get_momenta()
```python
```python
    assert abs(p_before - p_after) < 0.1  # Allow small numerical error
```
```
```

## Simulation Project (35%)

### Project: Multi-Body Dynamics Simulation

**Description**: Create a simulation of a compound pendulum (double pendulum) and analyze its chaotic behavior.

### Deliverables

| Deliverable | Format | Points |
|-------------|--------|--------|
| MuJoCo model | XML | 20 |
| Simulation script | Python | 35 |
| Analysis report | Markdown | 25 |
| Visualization | Video/Animation | 20 |

### Requirements

1. **Model Creation** (20 points)
   - Create a double pendulum with two links
   - Specify appropriate masses, lengths, and joint types
   - Include ground plane and visualization

2. **Simulation** (35 points)
   - Implement simulation with configurable initial conditions
   - Record joint angles, velocities, and energies
   - Run multiple simulations with slightly different initial conditions

3. **Chaos Analysis** (25 points)
   - Show sensitivity to initial conditions
   - Create Poincaré section or Lyapunov exponent estimate
   - Discuss implications for prediction

4. **Visualization** (20 points)
   - Create animated visualization of motion
   - Phase space plots
   - Energy vs. time plots

### Rubric

| Criterion | Excellent | Good | Satisfactory | Needs Work |
|-----------|-----------|------|--------------|------------|
| Model Quality | Physically accurate | Minor issues | Some inaccuracies | Incorrect physics |
| Code Quality | Well-organized, documented | Functional | Works but messy | Buggy |
| Analysis Depth | Insightful, thorough | Good observations | Basic analysis | Superficial |
| Visualization | Publication quality | Clear and useful | Basic | Unclear |

## Ethics Component (15%)

### Format: Written Reflection

**Length**: 400-600 words

### Prompt

The physics of rigid body dynamics directly determines the potential for physical harm in robotics. Consider a scenario where you are designing a robot arm that will work near humans in a factory setting.

Address the following:
1. What specific dynamics calculations would you perform to ensure safe operation?
2. How would you determine appropriate safety margins for forces and velocities?
3. What failure modes related to dynamics would you need to consider?
4. How does your understanding of momentum and energy inform emergency stop design?

### Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Technical Accuracy | 30 | Correctly applies dynamics concepts to safety |
| Safety Analysis | 30 | Identifies key hazards and mitigation strategies |
| Practical Application | 25 | Demonstrates real-world applicability |
| Writing Quality | 15 | Clear, organized presentation |

## Export Formats

This assessment is available in:
- [ ] QTI 2.1 (IMS standard)
- [ ] Canvas quiz import
- [ ] Moodle GIFT format
- [ ] Blackboard import
- [ ] PDF (instructor version with answers)
- [ ] Gradescope (auto-grading enabled)

## Alignment with Learning Objectives

| Learning Objective | Quiz | Lab | Project | Ethics |
|-------------------|------|-----|---------|--------|
| Newton-Euler equations | Q1, Q3 | ✓ | ✓ | ✓ |
| Forces and torques | Q3, Q7 | ✓ | ✓ | ✓ |
| Inertia tensor | Q5 | ✓ | ✓ | |
| Energy and momentum | Q2, Q4 | ✓ | ✓ | ✓ |
| Collision dynamics | Q4, Q6 | ✓ | | ✓ |
