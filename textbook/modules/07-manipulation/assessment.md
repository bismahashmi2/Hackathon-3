---
module_id: "07"
title: "Assessment Package: Manipulation"
---

# Assessment Package: Module 07 - Manipulation

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice + analysis | 45 minutes |
| Lab Exercises | 35% | Python implementations | 3 labs |
| Simulation Project | 35% | Complete manipulation system | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

---

## Theory Quiz

**Time Limit**: 45 minutes
**Passing Score**: 70%
**Attempts**: 2

### Section A: Multiple Choice (40 points)

**Q1.** Force closure in grasping means:
- a) The gripper is fully closed
- b) Contact forces can resist any external wrench
- c) The grasp uses friction
- d) The object cannot rotate

**Q2.** A parallel jaw gripper can achieve force closure on a sphere:
- a) Always, regardless of friction
- b) Only with sufficient friction
- c) Never, even with infinite friction
- d) Only if the sphere is deformable

**Q3.** In impedance control, increasing stiffness K will:
- a) Make the robot more compliant
- b) Increase tracking error
- c) Make the robot resist perturbations more strongly
- d) Reduce contact forces

**Q4.** The grasp matrix G relates:
- a) Joint torques to end-effector force
- b) Contact forces to object wrench
- c) Object position to finger positions
- d) Friction coefficient to normal force

**Q5.** For a 3-finger grasp on a planar object to achieve force closure, the minimum number of friction cone edges required is:
- a) 3
- b) 4
- c) 6
- d) Force closure is impossible with 3 frictionless contacts

**Q6.** In hybrid position/force control, the selection matrix S determines:
- a) Which fingers are active
- b) Which DOFs use force vs. position control
- c) The friction coefficient
- d) The grasp quality metric

**Q7.** The ε-metric (epsilon metric) for grasp quality measures:
- a) The number of contact points
- b) The largest perturbation wrench the grasp can resist
- c) The friction coefficient
- d) The grasp matrix rank

**Q8.** In-hand manipulation differs from regrasping because:
- a) The object is never released
- b) Only one finger moves at a time
- c) Force control is not used
- d) The object is always rotated

**Q9.** A collaborative robot should use impedance control instead of pure position control because:
- a) It's more accurate
- b) It's faster
- c) Contact forces are bounded by stiffness
- d) It uses less energy

**Q10.** The Jacobian transpose method for force control:
- a) Is exact for any robot configuration
- b) Assumes static equilibrium
- c) Requires force sensor on each joint
- d) Only works for planar robots

### Section B: Analysis Problems (60 points)

**Q11.** (20 points) A planar two-finger grasp contacts a circular object at points p₁ = (-r, 0) and p₂ = (r, 0), where r = 0.05m. The contact normals point toward the object center.

a) Write the grasp matrix G for this configuration (3×4 matrix for 2D).
b) What is the rank of G? What wrench space dimension is controllable?
c) If friction coefficient μ = 0.3, can this grasp resist a horizontal force of 1N applied to the object center? Show your analysis.

**Q12.** (20 points) An impedance controller has parameters K = 500 N/m and D = 50 Ns/m. The desired position is x_d = 0.5m.

a) If the current position is x = 0.48m and velocity is ẋ = 0.1 m/s, what force does the controller command?
b) The robot contacts a rigid wall at x = 0.49m. What is the steady-state contact force?
c) If we want maximum contact force of 20N, what stiffness K should we use?

**Q13.** (20 points) A manipulation task requires inserting a peg (radius 10mm) into a hole (radius 10.5mm). The peg and hole have alignment error of up to 2mm.

a) Can pure position control complete this task? Why or why not?
b) Describe how impedance control enables this task.
c) What compliance parameters would you recommend for the Z-axis (insertion direction) vs. XY-axes (alignment correction)?

---

## Lab Exercises

### Lab 07-01: Basic Grasping (30% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Grasp Pose Generation | Multiple valid grasps, considers object geometry | Single grasp approach works | Grasp often fails | Cannot generate grasps |
| Gripper Control | Smooth approach, proper grip force | Working but jerky motion | Inconsistent control | Non-functional |
| Pick Execution | Reliable pick across test objects | Occasional failures | Frequent failures | Cannot complete pick |
| Quality Evaluation | Contact metrics properly computed | Basic metrics work | Partial implementation | No evaluation |
| Code Quality | Well-structured, documented | Functional code | Partially working | Non-functional |

### Lab 07-02: Force Control (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Impedance Control | Correct implementation, tunable parameters | Working with fixed parameters | Partially working | Non-functional |
| Force Controller | PI force regulation within 10% error | Force regulation within 20% | Oscillatory or >20% error | Cannot regulate |
| Surface Following | Maintains contact through complex path | Works on simple paths | Frequent contact loss | Cannot follow |
| Analysis | Quantitative comparison of control modes | Basic comparison | Incomplete analysis | No analysis |
| Documentation | Clear explanation of tuning process | Basic documentation | Missing sections | No documentation |

### Lab 07-03: Dexterous Manipulation (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Finger Kinematics | All fingers working, proper Jacobians | Most fingers working | Some fingers working | Non-functional |
| Grasp Analysis | Force closure verified, ε-metric computed | Contact extraction works | Partial analysis | Cannot analyze |
| Multi-Finger Control | Coordinated grasp on multiple objects | Works on single object | Inconsistent | Non-functional |
| In-Hand Manipulation | Object rotation >45° while grasped | Some rotation achieved | Object often dropped | Cannot manipulate |
| Integration | Complete pick-manipulate sequence | Partial sequence | Individual pieces work | Non-functional |

---

## Simulation Project

### Project: Autonomous Pick-and-Place System

**Objective**: Build a complete manipulation pipeline that can pick objects from a table, manipulate them if needed, and place them at specified locations.

**Duration**: 1 week
**Deliverables**: Code repository + 4-page technical report

### Requirements

1. **Perception Integration** (20%)
   - Use simulated camera to detect objects
   - Estimate object pose for grasp planning
   - Handle multiple objects in scene

2. **Grasp Planning** (25%)
   - Generate multiple grasp candidates per object
   - Rank grasps by quality metric
   - Select grasp considering robot reachability

3. **Motion and Manipulation** (30%)
   - Plan collision-free approach motion
   - Execute grasp with force feedback
   - Regrasp or adjust if initial grasp fails

4. **Task Execution** (25%)
   - Complete pick-and-place for 5 different objects
   - Handle task failures gracefully
   - Report success/failure and timing metrics

### Object Set

The system must handle:
1. **Box** (6cm × 4cm × 3cm) - Easy
2. **Cylinder** (radius 2.5cm, height 8cm) - Easy
3. **Sphere** (radius 3cm) - Medium
4. **Irregular shape** (provided mesh) - Medium
5. **Thin plate** (10cm × 8cm × 0.5cm) - Hard

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Perception | 20 | Object detection and pose estimation |
| Grasp Planning | 25 | Quality-ranked candidate generation |
| Execution | 30 | Reliable grasp, manipulation, place |
| Robustness | 15 | Failure handling, multiple attempts |
| Report | 10 | Clear writing, quantitative results |
| **Total** | **100** | |

### Evaluation Scenarios

Each object is tested in 3 positions:
- Center of workspace (easy reach)
- Edge of workspace (challenging IK)
- Near other objects (collision avoidance)

**Success criteria**: Object placed within 2cm of target position, upright orientation.

---

## Ethics Discussion

### Prompt

*In a 500-word reflection, address the following scenario:*

A robotics company is developing a home assistant robot capable of manipulation tasks—opening doors, loading dishwashers, handling groceries, etc. During testing, the following incident occurs:

The robot is asked to "clear the table." On the table are:
- Dirty dishes (should be put in dishwasher)
- A newspaper (should be recycled or kept)
- Grandmother's antique teacup (extremely fragile, irreplaceable)
- A laptop computer (expensive, contains data)
- A half-eaten sandwich (trash or keep?)

The robot proceeds to grasp the antique teacup with force calibrated for normal ceramics, and it breaks. The family is devastated.

**Address the following:**

1. What safeguards might have prevented this incident? Consider both technical (sensing, force control) and process (user confirmation) approaches.

2. Who bears responsibility for the broken teacup? The robot company? The user who gave the command? The person who left the teacup on the table?

3. Should manipulation robots have "categories of caution"—object types that require human confirmation before handling? How would you define these categories?

4. How should the robot communicate uncertainty about object handling? The teacup might have looked similar to a regular cup. What signals should prompt extra caution?

### Rubric

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Technical Safeguards | Multiple specific solutions with tradeoffs | Several reasonable safeguards | One or two ideas | No concrete proposals |
| Responsibility Analysis | Nuanced consideration of all parties | Considers multiple parties | Single party blamed | Avoids the question |
| Category Design | Specific, practical categories with examples | General categories | Vague categorization | No categories |
| Uncertainty Communication | Multiple modalities, user-centered design | Basic communication approach | Generic suggestions | Ignores communication |
| Writing Quality | Clear, organized, persuasive | Clear with minor issues | Some clarity problems | Unclear or incomplete |

---

## Answer Key (Instructor Access Only)

### Quiz Answers

**Section A:**
1. b) Contact forces can resist any external wrench
2. b) Only with sufficient friction
3. c) Make the robot resist perturbations more strongly
4. b) Contact forces to object wrench
5. d) Force closure is impossible with 3 frictionless contacts
6. b) Which DOFs use force vs. position control
7. b) The largest perturbation wrench the grasp can resist
8. a) The object is never released
9. c) Contact forces are bounded by stiffness
10. b) Assumes static equilibrium

**Section B:**

**Q11:**
a) Grasp matrix for 2D (forces create wrench on object):

   Contact 1 at (-r, 0), normal = (1, 0):
   - Force contribution: (1, 0)
   - Torque contribution: (-r × 0) - (0 × 1) = 0

   Contact 2 at (r, 0), normal = (-1, 0):
   - Force contribution: (-1, 0)
   - Torque contribution: (r × 0) - (0 × -1) = 0

   G = | 1  0  -1  0 |
       | 0  1   0  1 |
       | 0  0   0  0 |

   (Note: This shows the grasp cannot control torque—needs friction)

b) Rank(G) = 2. Only translational forces controllable, not torque.

c) With μ = 0.3, each contact can apply tangential force up to 0.3×N.
   - For 1N horizontal force, need contact to provide 0.5N tangential force
   - This requires normal force N ≥ 0.5/0.3 = 1.67N per contact
   - **Yes**, achievable with sufficient grip force

**Q12:**
a) F = K(x_d - x) - D(ẋ) = 500(0.5 - 0.48) - 50(0.1) = 10 - 5 = **5N**

b) At steady state, x = 0.49m, ẋ = 0:
   F = 500(0.5 - 0.49) - 0 = **5N** (pushing against wall)

c) For max 20N: K(x_d - x_wall) ≤ 20
   K(0.5 - 0.49) ≤ 20
   K ≤ 20/0.01 = **2000 N/m**

**Q13:**
a) **No.** Position control would jam against hole edges due to 2mm misalignment (larger than 0.5mm clearance). The robot would apply excessive force trying to reach the commanded position.

b) Impedance control allows the peg to "feel" the hole edges and comply. Low XY stiffness lets misalignment self-correct as the peg slides along chamfers/edges.

c) Recommended parameters:
   - **Z-axis (insertion)**: Higher stiffness (500-1000 N/m) for positive insertion motion
   - **XY-axes (alignment)**: Low stiffness (50-100 N/m) to allow compliance with hole edges
   - **Rotation**: Low stiffness to allow angular correction

   This is sometimes called "Remote Center Compliance" (RCC) when mechanically implemented.

---

## Export Formats

This assessment package is available in:
- Markdown (this document)
- Canvas LMS import package
- PDF with answer key (instructor version)
- Gradescope autograder configuration (for lab submissions)
