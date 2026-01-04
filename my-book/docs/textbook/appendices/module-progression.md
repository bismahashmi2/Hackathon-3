# Module Progression Map

This document visualizes the learning progression through all 14 modules of the Physical AI and Humanoid Robotics textbook.

## Overview

The curriculum is structured in three difficulty tiers spanning a 14-week semester:

| Tier | Weeks | Modules | Focus Area |
|------|-------|---------|------------|
| **Beginner** | 1-4 | 01-04 | Foundations: concepts, physics, kinematics, sensing |
| **Intermediate** | 5-10 | 05-10 | Core Skills: control, planning, manipulation, locomotion, ROS2, deployment |
| **Advanced** | 11-14 | 11-14 | Integration: learning, HRI, autonomy, capstone |

## Prerequisite Graph

```
                        ┌─────────────────────────────────────────────────┐
                        │           BEGINNER (Weeks 1-4)                  │
                        └─────────────────────────────────────────────────┘

                                    ┌─────────┐
                                    │   01    │
                                    │ Intro   │
                                    └────┬────┘
                                         │
                    ┌────────────────────┼────────────────────┐
                    │                    │                    │
                    ▼                    ▼                    ▼
              ┌─────────┐          ┌─────────┐          ┌─────────┐
              │   02    │          │   03    │          │   04    │
              │ Dynamics│          │Kinematics│         │ Sensors │
              └────┬────┘          └────┬────┘          └────┬────┘
                   │                    │                    │
                        └─────────────────────────────────────────────────┘
                        │         INTERMEDIATE (Weeks 5-10)               │
                        └─────────────────────────────────────────────────┘
                   │                    │                    │
                   ├────────────────────┤                    │
                   │                    │                    │
                   ▼                    ▼                    │
              ┌─────────┐          ┌─────────┐               │
              │   05    │          │   06    │               │
              │ Control │◄─────────│Planning │◄──────────────┤
              └────┬────┘          └────┬────┘               │
                   │                    │                    │
                   ├────────────────────┤                    │
                   │                    │                    │
                   ▼                    ▼                    ▼
              ┌─────────┐          ┌─────────┐          ┌─────────┐
              │   07    │          │   08    │          │   09    │
              │ Manip.  │          │ Loco.   │          │  ROS2   │
              └────┬────┘          └────┬────┘          └────┬────┘
                   │                    │                    │
                   └────────────────────┴────────────────────┘
                                        │
                                        ▼
                                   ┌─────────┐
                                   │   10    │
                                   │Sim2Real │
                                   └────┬────┘
                                        │
                        └─────────────────────────────────────────────────┘
                        │            ADVANCED (Weeks 11-14)               │
                        └─────────────────────────────────────────────────┘
                                        │
                   ┌────────────────────┼────────────────────┐
                   │                    │                    │
                   ▼                    ▼                    ▼
              ┌─────────┐          ┌─────────┐          ┌─────────┐
              │   11    │          │   12    │          │   13    │
              │Learning │          │  HRI    │          │Autonomy │
              └────┬────┘          └────┬────┘          └────┬────┘
                   │                    │                    │
                   └────────────────────┴────────────────────┘
                                        │
                                        ▼
                                   ┌─────────┐
                                   │   14    │
                                   │Capstone │
                                   └─────────┘
```

## Module Details

### Beginner Tier (Weeks 1-4)

| Module | Title | Prerequisites | Key Topics |
|--------|-------|---------------|------------|
| **01** | Introduction to Physical AI | None | Physical AI definition, embodiment, history, humanoids |
| **02** | Rigid Body Dynamics | 01 | Newton-Euler, forces, torques, inertia, momentum |
| **03** | Kinematics Fundamentals | 02 | Forward/inverse kinematics, DH parameters, workspaces |
| **04** | Sensors and Perception | 01 | IMU, cameras, LIDAR, sensor fusion, state estimation |

### Intermediate Tier (Weeks 5-10)

| Module | Title | Prerequisites | Key Topics |
|--------|-------|---------------|------------|
| **05** | Dynamics and Control | 02, 03 | PID, computed torque, impedance control, stability |
| **06** | Motion Planning | 03, 04 | RRT, A*, trajectory optimization, collision avoidance |
| **07** | Manipulation | 05, 06 | Grasping, force control, dexterous manipulation |
| **08** | Locomotion | 05 | ZMP, whole-body control, bipedal walking, balance |
| **09** | ROS2 Integration | 04, 06 | Nodes, topics, services, robot state, transforms |
| **10** | Simulation to Real | 07, 08, 09 | Domain randomization, reality gap, deployment |

### Advanced Tier (Weeks 11-14)

| Module | Title | Prerequisites | Key Topics |
|--------|-------|---------------|------------|
| **11** | Learning-Based Control | 05, 07 | RL, imitation learning, policy optimization |
| **12** | Human-Robot Interaction | 04, 07 | Safety, collaboration, natural interfaces |
| **13** | Full-Body Autonomy | 08, 10, 11 | Integrated systems, decision making, autonomy |
| **14** | Capstone Integration | All (01-13) | System design, deployment, future directions |

## Learning Paths

### Path A: Control Systems Focus
01 → 02 → 03 → 05 → 07/08 → 11 → 14

### Path B: Perception Focus
01 → 04 → 06 → 09 → 10 → 12 → 14

### Path C: Full Stack Roboticist (Recommended)
01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09 → 10 → 11 → 12 → 13 → 14

## Time Allocation

| Module | Estimated Hours | Theory | Labs | Assessment |
|--------|-----------------|--------|------|------------|
| 01 | 10 | 4 | 4 | 2 |
| 02 | 12 | 5 | 5 | 2 |
| 03 | 12 | 5 | 5 | 2 |
| 04 | 12 | 4 | 6 | 2 |
| 05 | 14 | 5 | 7 | 2 |
| 06 | 14 | 5 | 7 | 2 |
| 07 | 14 | 5 | 7 | 2 |
| 08 | 14 | 5 | 7 | 2 |
| 09 | 12 | 4 | 6 | 2 |
| 10 | 12 | 4 | 6 | 2 |
| 11 | 14 | 5 | 7 | 2 |
| 12 | 12 | 4 | 6 | 2 |
| 13 | 14 | 5 | 7 | 2 |
| 14 | 14 | 4 | 8 | 2 |
| **Total** | **170** | **64** | **88** | **28** |

## Prerequisite Validation Rules

1. **No circular dependencies**: The prerequisite graph is a directed acyclic graph (DAG)
2. **Difficulty progression**: Prerequisites must be same or lower difficulty tier
3. **Week ordering**: Prerequisites must have lower week numbers
4. **Completeness**: All modules except 01 have at least one prerequisite
5. **Reachability**: All modules are reachable from Module 01

### Validation Status

| Rule | Status |
|------|--------|
| DAG (no cycles) | ✅ PASS |
| Difficulty progression | ✅ PASS |
| Week ordering | ✅ PASS |
| Completeness | ✅ PASS |
| Reachability | ✅ PASS |
