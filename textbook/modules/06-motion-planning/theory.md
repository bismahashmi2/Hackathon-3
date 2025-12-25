---
id: "06"
title: "Motion Planning"
slug: "motion-planning"
week: 6
difficulty: intermediate
prerequisites: ["03", "04"]
learning_objectives:
  - "Implement RRT and RRT* algorithms for path planning in high-dimensional spaces"
  - "Apply A* and graph-based search for navigation problems"
  - "Generate smooth trajectories satisfying kinodynamic constraints"
  - "Design collision-free paths using occupancy grids and signed distance fields"
estimated_hours: 14
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 06: Motion Planning

## Introduction

Motion planning answers: "How do I get from here to there without hitting anything?" This module covers algorithms for computing collision-free paths and trajectories.

## Section 1: Sampling-Based Planning

### 1.1 RRT Algorithm

<definition id="def-rrt">
**Rapidly-exploring Random Tree (RRT)**: A sampling-based algorithm that incrementally builds a tree by randomly sampling the configuration space and extending toward samples.
</definition>

```python
def rrt(start, goal, obstacles, max_iter=1000):
    tree = Tree(start)
    for _ in range(max_iter):
        q_rand = random_config()
        q_near = tree.nearest(q_rand)
        q_new = extend(q_near, q_rand)
        if collision_free(q_near, q_new, obstacles):
            tree.add(q_new, parent=q_near)
            if distance(q_new, goal) < threshold:
                return tree.path_to(q_new)
    return None
```

### 1.2 RRT* Optimization

RRT* adds rewiring to find asymptotically optimal paths.

## Section 2: Graph Search

### 2.1 A* Algorithm

<equation id="eq-astar">
$$f(n) = g(n) + h(n)$$
</equation>

where $g(n)$ is cost from start, $h(n)$ is heuristic to goal.

### 2.2 Occupancy Grids

Discrete representation of free and occupied space.

## Section 3: Trajectory Optimization

### 3.1 Minimum Jerk Trajectories

Smooth trajectories minimize:

<equation id="eq-min-jerk">
$$J = \int_0^T \left(\frac{d^3x}{dt^3}\right)^2 dt$$
</equation>

### 3.2 Time-Optimal Trajectories

Subject to velocity, acceleration, and jerk limits.

<warning>
Motion planning computation time can vary dramatically. Always have fallback behaviors for planning failures.
</warning>

## Section 4: Collision Detection

### 4.1 Signed Distance Fields

Pre-computed distance to nearest obstacle surface.

### 4.2 Hierarchical Methods

Broad-phase (bounding volumes) then narrow-phase (detailed geometry).

## Summary

Key takeaways:
1. Sampling-based methods work in high-dimensional spaces
2. Graph search optimal for discretized problems
3. Trajectory optimization adds smoothness and constraints
4. Collision detection must be efficient for real-time use

## Key Concepts

- **RRT**: Sampling-based path planning
- **A***: Optimal graph search
- **Trajectory Optimization**: Generating smooth motions
- **SDF**: Signed distance field for collision

## Further Reading

1. LaValle, S. (2006). "Planning Algorithms"
2. Choset, H. et al. (2005). "Principles of Robot Motion"
