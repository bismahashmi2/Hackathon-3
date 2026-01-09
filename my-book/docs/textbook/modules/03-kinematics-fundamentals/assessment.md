---
module_id: "03"
---

# Assessment Package: Kinematics Fundamentals

## Overview

| Component | Weight | Format |
|-----------|--------|--------|
| Theory Quiz | 15% | Multiple choice + calculations |
| Lab Assessment | 35% | Jupyter notebooks with auto-grading |
| Simulation Project | 35% | Code submission with test harness |
| Ethics Component | 15% | Written reflection |

## Theory Quiz (15%)

**Time Limit**: 30 minutes
**Passing Score**: 70%
**Attempts Allowed**: 2

### Questions

#### Q1 (Multiple Choice, 2 points)
**Question**: In the Denavit-Hartenberg (DH) convention, the parameter 'a' represents:

**Options**:
a) Rotation about the z-axis
b) Translation along the z-axis
c) Translation along the x-axis (link length)
d) Rotation about the x-axis

**Correct Answer**: c
**Explanation**: In DH convention, 'a' is the link length—the distance along the x-axis from `z_{i-1}` to `z_i`.
**Learning Objective**: Apply DH parameters to derive transformation matrices

---

#### Q2 (Calculation, 4 points)
**Question**: A 2-DOF planar arm has link lengths L1 = 0.5m and L2 = 0.3m. If θ1 = 60° and θ2 = 45°, calculate the (x, y) position of the end-effector.

**Sample Answer**:
- Elbow: x1 = 0.5 × cos(60°) = 0.25m, y1 = 0.5 × sin(60°) = 0.433m
- End-effector: x = 0.25 + 0.3 × cos(105°) = 0.25 + (-0.078) = 0.172m
- y = 0.433 + 0.3 × sin(105°) = 0.433 + 0.290 = 0.723m
- Position: (0.172m, 0.723m)

**Rubric**:
- 2 points: Correct elbow position
- 2 points: Correct end-effector position
**Learning Objective**: Implement forward kinematics equations for serial manipulators

---

#### Q3 (Multiple Choice, 2 points)
**Question**: For a 2-DOF planar arm with L1 = L2 = 0.5m, what is the shape of the reachable workspace?

**Options**:
a) A circle with radius 1.0m
b) An annular region (donut) with outer radius 1.0m and inner radius 0m
c) A square with side length 1.0m
d) An annular region with outer radius 1.0m and inner radius 0.5m

**Correct Answer**: b
**Explanation**: When L1 = L2, the inner radius is |L1 - L2| = 0, so the workspace is a full disk (degenerate annulus) with radius L1 + L2 = 1.0m.
**Learning Objective**: Analyze workspace and reachability for manipulators

---

#### Q4 (Short Answer, 4 points)
**Question**: Explain why inverse kinematics can have multiple solutions and how a robot might choose between them.

**Sample Answer**:
Multiple solutions exist because different joint configurations can place the end-effector at the same location (e.g., elbow-up vs. elbow-down). Robots choose between solutions based on criteria such as:
- Minimizing joint motion from current configuration
- Avoiding obstacles or singularities
- Staying within joint limits
- Selecting the configuration closest to a "home" pose
- Minimizing energy consumption

**Rubric**:
- 2 points: Correctly explains why multiple solutions exist
- 2 points: Provides reasonable selection criteria
**Learning Objective**: Solve inverse kinematics with multiple solution handling

---

#### Q5 (Multiple Choice, 2 points)
**Question**: What happens at a kinematic singularity?

**Options**:
a) The robot gains additional degrees of freedom
b) The Jacobian matrix becomes singular (loses rank)
c) The robot cannot reach any targets
d) All inverse kinematics solutions become identical

**Correct Answer**: b
**Explanation**: At a singularity, the Jacobian loses rank, meaning the robot loses a degree of freedom in Cartesian space. Small Cartesian motions may require infinite joint velocities.
**Learning Objective**: Identify and handle kinematic singularities

---

#### Q6 (Calculation, 4 points)
**Question**: For the 2-DOF arm in Q2, compute the Jacobian matrix elements J11 and J21 (first column) when θ1 = 90° and θ2 = 0°.

The Jacobian for a 2-DOF arm is:
```
J = [−L1·sin(θ1) − L2·sin(θ1+θ2), −L2·sin(θ1+θ2)]
    [ L1·cos(θ1) + L2·cos(θ1+θ2),  L2·cos(θ1+θ2)]
```

**Sample Answer**:
- θ1 = 90°, θ2 = 0° → θ1 + θ2 = 90°
- J11 = −0.5·sin(90°) − 0.3·sin(90°) = −0.5 − 0.3 = −0.8
- J21 = 0.5·cos(90°) + 0.3·cos(90°) = 0 + 0 = 0

**Rubric**:
- 2 points: Correct J11 calculation
- 2 points: Correct J21 calculation
**Learning Objective**: Compute the Jacobian matrix for velocity kinematics

---

#### Q7 (Multiple Choice, 2 points)
**Question**: A humanoid robot arm has 7 DOF but only needs 6 DOF to specify arbitrary end-effector position and orientation. This extra DOF is called:

**Options**:
a) A redundant degree of freedom
b) A singular degree of freedom
c) A constrained degree of freedom
d) A virtual degree of freedom

**Correct Answer**: a
**Explanation**: When a manipulator has more DOF than needed for the task, the extra DOF are called redundant. Redundancy allows the robot to optimize secondary objectives (obstacle avoidance, joint limits) while achieving the primary task.
**Learning Objective**: Understand kinematic redundancy

---

**Total Quiz Points**: 20

## Lab Assessment (35%)

### Grading Rubric for Labs 03-01, 03-02, 03-03

| Criterion | Weight | Excellent (90-100%) | Good (70-89%) | Satisfactory (50-69%) | Needs Work (&lt;50%) |
|-----------|--------|---------------------|---------------|----------------------|-------------------|
| FK Implementation | 30% | Accurate, well-documented | Minor errors | Basic functionality | Major errors |
| IK Implementation | 30% | Multiple solutions handled | Single solution works | Partial IK | Doesn't work |
| Workspace Analysis | 20% | Complete with visualization | Good analysis | Basic analysis | Incomplete |
| Code Quality | 20% | Clean, documented, tested | Good code | Functional | Messy or buggy |

### Auto-Grading Tests

```python
# tests/lab_03_tests.py
import numpy as np

def test_fk_2dof():
    """Test forward kinematics accuracy."""
    # θ1=0, θ2=0 should give (L1+L2, 0)
    x, y = forward_kinematics_2dof(0, 0, 0.5, 0.3)
    assert abs(x - 0.8) < 1e-6
    assert abs(y) < 1e-6

def test_ik_reaches_target():
    """Test that IK solution reaches target."""
    target = (0.5, 0.3)
    solution = inverse_kinematics_2dof(*target)
    if solution:
        achieved = forward_kinematics_2dof(*solution)
        error = np.sqrt(
            (achieved[0] - target[0])**2 +
            (achieved[1] - target[1])**2
        )
        assert error < 1e-4

def test_ik_unreachable():
    """Test that unreachable points return None."""
    solution = inverse_kinematics_2dof(2.0, 0)
    assert solution is None

def test_jacobian_dimensions():
    """Test Jacobian matrix has correct shape."""
    J = jacobian_2dof(0, 0)
    assert J.shape == (2, 2)
```

## Simulation Project (35%)

### Project: Humanoid Reaching Task

**Description**: Implement a complete reaching task for a humanoid robot arm, including workspace analysis, IK solving, trajectory planning, and collision checking.

### Deliverables

| Deliverable | Format | Points |
|-------------|--------|--------|
| Workspace visualization | PNG/PDF | 15 |
| IK solver module | Python | 30 |
| Trajectory planner | Python | 25 |
| Demo notebook | Jupyter | 20 |
| Documentation | Markdown | 10 |

### Requirements

1. **Workspace Analysis** (15 points)
   - Compute reachable workspace for the right arm
   - Visualize in 3D with color-coded reachability
   - Identify singularity regions

2. **IK Solver** (30 points)
   - Implement numerical IK for 7-DOF arm
   - Handle joint limits
   - Return multiple solutions when applicable
   - Include convergence diagnostics

3. **Trajectory Planning** (25 points)
   - Generate smooth trajectories between poses
   - Implement both joint-space and Cartesian-space interpolation
   - Verify collision-free paths

4. **Demonstration** (20 points)
   - Interactive notebook showing complete pipeline
   - Multiple reaching scenarios
   - Performance metrics (solve time, accuracy)

5. **Documentation** (10 points)
   - Clear API documentation
   - Usage examples
   - Limitations and known issues

### Rubric

| Criterion | Excellent | Good | Satisfactory | Needs Work |
|-----------|-----------|------|--------------|------------|
| Correctness | All features work | Minor bugs | Partial functionality | Major issues |
| Completeness | All requirements met | Most met | Some missing | Many missing |
| Code Quality | Publication-ready | Good style | Functional | Poor quality |
| Analysis | Deep insights | Good observations | Basic analysis | Superficial |

## Ethics Component (15%)

### Format: Written Reflection

**Length**: 400-600 words

### Prompt

Consider a humanoid robot designed to work in a hospital environment, reaching for objects on shelves, opening doors, and assisting patients.

Address the following:

1. How would you define the robot's workspace boundaries to ensure patient safety?
2. When multiple IK solutions exist for reaching a target, what criteria should guide the selection? Consider:
   - A solution that keeps the elbow away from bedridden patients
   - A solution that minimizes motion through high-traffic areas
   - A solution that is most energy-efficient
3. How should the robot behave when approaching the edge of its workspace (near singularities)?
4. What kinematic constraints would you impose specifically for a healthcare environment?

### Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Safety Analysis | 30 | Identifies key workspace safety concerns |
| Solution Selection | 25 | Thoughtful criteria for IK selection |
| Singularity Handling | 20 | Practical approach to edge cases |
| Healthcare Context | 15 | Appropriate for healthcare environment |
| Writing Quality | 10 | Clear, organized presentation |

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
| DH parameters | Q1 | ✓ | ✓ | |
| Forward kinematics | Q2 | ✓ | ✓ | |
| Workspace analysis | Q3 | ✓ | ✓ | ✓ |
| Inverse kinematics | Q4 | ✓ | ✓ | ✓ |
| Singularities | Q5 | ✓ | ✓ | ✓ |
| Jacobian | Q6 | ✓ | ✓ | |
| Redundancy | Q7 | | ✓ | |
