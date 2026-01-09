---
module_id: "10"
title: "Assessment Package: Sim-to-Real Transfer"
---

# Assessment Package: Module 10 - Sim-to-Real Transfer

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice | 30 minutes |
| Lab Exercises | 35% | Implementation | 3 labs |
| Transfer Project | 35% | Real hardware deployment | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

## Theory Quiz (Sample Questions)

**Q1.** Domain randomization helps sim-to-real transfer by:
- a) Making simulation faster
- b) Training on distribution that includes reality
- c) Eliminating all sim-to-real gaps
- d) Removing need for real testing

**Answer**: b) Training on distribution that includes reality

**Q2.** System identification is used to:
- a) Name the robot
- b) Estimate real robot parameters
- c) Generate random parameters
- d) Simulate faster

**Answer**: b) Estimate real robot parameters

## Lab Exercise Rubrics

### Lab 10-01: Domain Randomization (30%)
- Physics randomization: 25 pts
- Visual randomization: 20 pts
- Curriculum learning: 20 pts
- Analysis: 20 pts
- Documentation: 15 pts

### Lab 10-02: System Identification (35%)
- Step response: 25 pts
- Friction identification: 25 pts
- Frequency response: 20 pts
- Validation: 20 pts
- Analysis: 10 pts

### Lab 10-03: Complete Transfer (35%)
- Training in sim: 20 pts
- Deployment to hardware: 40 pts
- Performance metrics: 20 pts
- Gap analysis: 20 pts

## Transfer Project

**Objective**: Train and deploy navigation controller

**Requirements**:
1. Train in randomized simulation
2. Validate with identified parameters
3. Deploy to TurtleBot 4
4. Achieve >70% success on test course
5. Document remaining sim-to-real gaps

**Grading**: Success rate (40%), analysis quality (30%), documentation (30%)

## Ethics Discussion

Reflect on simulation testing adequacy for safety-critical robotics applications.

**Rubric**: Analysis depth (40%), practical proposals (30%), ethical reasoning (30%)
