---
module_id: "06"
title: "Assessment Package: Motion Planning"
---

# Assessment Package: Module 06 - Motion Planning

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice + algorithm analysis | 40 minutes |
| Lab Exercises | 35% | Python implementations | 3 labs |
| Simulation Project | 35% | Complete planner + analysis | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

---

## Theory Quiz

**Time Limit**: 40 minutes
**Passing Score**: 70%
**Attempts**: 2

### Section A: Multiple Choice (40 points)

**Q1.** RRT is classified as which type of motion planning algorithm?
- a) Graph search
- b) Potential field
- c) Sampling-based
- d) Optimization-based

**Q2.** For A* to guarantee optimal paths, the heuristic must be:
- a) Consistent
- b) Admissible
- c) Both admissible and consistent
- d) Neither (A* is always optimal)

**Q3.** The configuration space (C-space) of a robot arm with n revolute joints has dimension:
- a) 3 (x, y, z position)
- b) n (one per joint)
- c) 6 (position and orientation)
- d) 2n (position and velocity per joint)

**Q4.** In RRT, the "nearest" operation finds the tree node that:
- a) Has the lowest cost-to-come
- b) Is closest to the randomly sampled configuration
- c) Is closest to the goal
- d) Has the fewest children

**Q5.** Which heuristic is admissible for an 8-connected grid with diagonal movement cost √2?
- a) Manhattan distance
- b) Euclidean distance
- c) Chebyshev distance
- d) All of the above

**Q6.** Probabilistic completeness means an algorithm:
- a) Always finds a path if one exists
- b) Finds a path with probability approaching 1 as time increases
- c) Returns optimal paths with high probability
- d) Has bounded runtime

**Q7.** The primary advantage of RRT* over RRT is:
- a) Faster exploration
- b) Asymptotic optimality
- c) Simpler implementation
- d) Lower memory usage

**Q8.** In trajectory optimization, "direct collocation" refers to:
- a) Detecting collisions along the trajectory
- b) Discretizing the trajectory and optimizing all points
- c) Directly computing the optimal solution analytically
- d) Collecting data to train a neural network

**Q9.** A robot with 6-DOF moving among 10 spherical obstacles has a C-space that is:
- a) 6-dimensional with spherical obstacles
- b) 6-dimensional with complex-shaped C-obstacles
- c) 16-dimensional
- d) Impossible to compute

**Q10.** Goal bias in RRT helps to:
- a) Improve path quality
- b) Speed up goal discovery
- c) Reduce memory usage
- d) Guarantee completeness

### Section B: Algorithm Analysis (60 points)

**Q11.** (20 points) Consider A* on a graph with the following structure:

```
Start(S) ---2--- A ---3--- Goal(G)
    |           |
    4           1
    |           |
    B ----2---- C
```

With h(S)=4, h(A)=2, h(B)=3, h(C)=1, h(G)=0:

a) Is this heuristic admissible? Justify your answer.
b) List the order in which nodes are expanded by A*.
c) What is the optimal path and its cost?

**Q12.** (20 points) An RRT planner has the following parameters:
- Step size δ = 0.5 rad
- Goal bias = 0.1
- Workspace: 2-DOF arm with joint limits [-π, π]

a) If the random sample is q_rand = (1.5, 2.0) and the nearest tree node is q_near = (1.0, 1.5), what is q_new?
b) Approximately what fraction of samples will be the goal configuration?
c) If after 1000 iterations the tree has 800 nodes, what is the average collision rate?

**Q13.** (20 points) A trajectory optimizer minimizes:

$$J = T + 0.01 \int_0^T ||\ddot{q}||^2 dt$$

subject to dynamics and boundary conditions.

a) What does minimizing T encourage?
b) What does the integral term encourage?
c) If we increase the coefficient from 0.01 to 1.0, how would the optimal trajectory change?

---

## Lab Exercises

### Lab 06-01: RRT Path Planning (30% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Collision Check | Correct detection with proper resolution | Minor edge cases missed | Incomplete checking | Non-functional |
| RRT Implementation | Proper sampling, steering, tree growth | Minor issues with one component | Partial implementation | Non-functional |
| Path Finding | Reliably finds paths in test scenarios | Occasional failures | Inconsistent results | Cannot find paths |
| Visualization | Clear C-space and workspace views | Functional visualization | Basic plots | No visualization |
| Path Smoothing | Effective shortcutting maintains validity | Working but suboptimal | Partial smoothing | No smoothing |

### Lab 06-02: A* Path Planning (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Grid Environment | Proper obstacle representation and neighbors | Minor issues | Incomplete | Non-functional |
| Heuristics | Multiple correct admissible heuristics | Some heuristics incorrect | Single heuristic only | No heuristics |
| A* Implementation | Optimal paths with efficient data structures | Working but inefficient | Suboptimal paths | Non-functional |
| Comparison | Fair comparison with clear conclusions | Basic comparison | Limited analysis | No comparison |
| C-Space Extension | Working configuration space planner | Partial extension | Incomplete | Not attempted |

### Lab 06-03: Trajectory Optimization (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Problem Formulation | Correct variables, cost, constraints | Minor formulation errors | Incomplete formulation | Incorrect formulation |
| Dynamics Constraints | Proper collocation with tight residuals | Working with loose tolerances | Partially working | Non-functional |
| Optimization | Converges to valid trajectory | Converges occasionally | Rarely converges | Does not converge |
| Visualization | Comprehensive multi-panel plots | Basic visualization | Incomplete | No visualization |
| Analysis | Insightful discussion of trade-offs | Basic analysis | Incomplete | Missing |

---

## Simulation Project

### Project: Multi-Robot Path Planning

**Objective**: Plan collision-free paths for 3 mobile robots navigating a shared warehouse environment.

**Duration**: 1 week
**Deliverables**: Code repository + 4-page technical report

### Requirements

1. **Environment Setup** (15%)
   - Create 2D warehouse map with aisles and obstacles
   - Define start and goal positions for 3 robots
   - Implement collision checking for robot footprints

2. **Single-Robot Planning** (25%)
   - Implement either RRT or A* for individual path planning
   - Handle robot geometry (circular footprint, not point)
   - Achieve sub-second planning times

3. **Multi-Robot Coordination** (35%)
   - Implement one of: priority-based planning, velocity obstacles, or CBS
   - Ensure robots don't collide with each other
   - Handle cases where robots need to wait or yield

4. **Analysis and Evaluation** (25%)
   - Measure: planning time, path length, makespan
   - Test on 5 different scenarios
   - Compare single-robot vs. coordinated planning
   - Discuss scalability to more robots

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Environment | 15 | Correct map representation and collision checking |
| Single-Robot | 25 | Working planner meeting timing requirements |
| Coordination | 35 | Correct multi-robot collision avoidance |
| Analysis | 15 | Thorough evaluation on multiple scenarios |
| Report | 10 | Clear writing, proper figures |
| **Total** | **100** | |

### Test Scenarios

1. **Simple**: 3 robots, well-separated start/goals, few obstacles
2. **Crossing**: Robots must cross paths to reach goals
3. **Narrow**: Single-width corridors requiring sequencing
4. **Congested**: Multiple robots in tight space
5. **Custom**: Student-designed challenging scenario

---

## Ethics Discussion

### Prompt

*In a 500-word reflection, address the following scenario:*

A city is considering allowing autonomous delivery robots on public sidewalks. The robots would use motion planning algorithms to navigate around pedestrians, maintaining minimum clearance distances derived from collision safety studies.

Critics argue that even collision-free robot navigation changes the character of public sidewalks. They point out that:
- Pedestrians must constantly monitor for and yield to robots
- Robots move faster than pedestrians, creating "pressure" from behind
- The presence of many robots changes the sidewalk from a social space to a logistics corridor

The company argues their robots are safer than the alternatives (delivery trucks, cyclists) and provide valuable service to residents.

**Address the following:**
- Is collision-free motion planning sufficient for ethical sidewalk navigation? What else should planners optimize for?
- How should the burden of collision avoidance be distributed between robots and pedestrians?
- What constraints or requirements would you impose on robot motion planning in public spaces?
- Who should make these decisions—companies, cities, engineers, or the public?

### Rubric

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Beyond Safety | Recognizes need for social appropriateness | Acknowledges some non-safety concerns | Focuses mostly on collision avoidance | Only considers safety |
| Burden Distribution | Thoughtful analysis of yielding responsibility | Basic consideration | Oversimplified | Ignores the question |
| Concrete Proposals | Specific, implementable constraints | General recommendations | Vague suggestions | No proposals |
| Governance | Considers multiple stakeholders | Acknowledges decision complexity | Single-stakeholder view | Ignores governance |
| Writing Quality | Clear, well-organized | Clear with minor issues | Some clarity problems | Unclear |

---

## Answer Key (Instructor Access Only)

### Quiz Answers

**Section A:**
1. c) Sampling-based
2. b) Admissible (consistency ensures no reopening but admissibility suffices for optimality)
3. b) n (one per joint)
4. b) Is closest to the randomly sampled configuration
5. b) Euclidean distance (Manhattan overestimates for diagonal movement)
6. b) Finds a path with probability approaching 1 as time increases
7. b) Asymptotic optimality
8. b) Discretizing the trajectory and optimizing all points
9. b) 6-dimensional with complex-shaped C-obstacles
10. b) Speed up goal discovery

**Section B:**

**Q11:**
a) Admissibility check: h must not overestimate true cost.
   - True cost S→G: min(S-A-G=5, S-B-C-A-G=7) = 5
   - h(S)=4 ≤ 5 ✓
   - True cost A→G: 3, h(A)=2 ≤ 3 ✓
   - True cost B→G: B-C-A-G=6, h(B)=3 ≤ 6 ✓
   - True cost C→G: C-A-G=4, h(C)=1 ≤ 4 ✓
   - **Yes, admissible**

b) Expansion order:
   - Start: f(S)=0+4=4
   - Expand S: f(A)=2+2=4, f(B)=4+3=7
   - Expand A: f(G)=5+0=5
   - Expand G (goal reached)
   - **Order: S, A, G**

c) Optimal path: **S → A → G, cost = 5**

**Q12:**
a) Direction: (1.5-1.0, 2.0-1.5) = (0.5, 0.5), normalized: (0.707, 0.707)
   q_new = (1.0, 1.5) + 0.5 × (0.707, 0.707) = **(1.354, 1.854)**

b) Goal bias = 0.1 means **10% of samples** will be the goal.

c) 200 samples rejected out of 1000 → collision rate ≈ **20%**

**Q13:**
a) Minimizing T encourages **faster motions** (minimum time trajectory)

b) The integral term encourages **smooth acceleration** (jerk minimization equivalent)

c) With higher coefficient: **slower, smoother trajectory** - less aggressive acceleration/deceleration, longer total time

---

## Export Formats

This assessment package is available in:
- Markdown (this document)
- Canvas LMS import package
- PDF with answer key (instructor version)
- Gradescope autograder configuration (for lab submissions)
