---
id: lab-XX-NN
module_id: "XX"
title: "Lab Title"
difficulty: guided  # guided | intermediate | challenge
tier: simulation  # simulation | low_cost_hardware | advanced_hardware
duration_minutes: 60
submission_format: notebook  # notebook | code | report | demo
auto_gradable: false
---

# Lab XX-NN: Lab Title

## Objectives

By completing this lab, you will:

- [ ] Objective 1
- [ ] Objective 2
- [ ] Objective 3

## Prerequisites

- Completed Module XX theory content
- Familiarity with Python and numpy
- MuJoCo environment configured (see Appendix: Software Setup)

## Materials

| Type | Name | Version | Tier | Alternatives |
|------|------|---------|------|--------------|
| software | MuJoCo | 3.0+ | required | - |
| software | Python | 3.10+ | required | - |
| simulation | model.xml | - | required | - |

## Background

Brief background on the concepts being applied in this lab.

## Instructions

### Step 1: Environment Setup

Description of the first step.

```python
import mujoco
import numpy as np

# Code for step 1
```

<checkpoint>
**Expected Result**: Description of what you should see/have at this point.

**Troubleshooting**:
- If you see error X, try solution Y
- If model doesn't load, check file path
</checkpoint>

### Step 2: Implementation

Description of the second step.

```python
# Code for step 2
```

<checkpoint>
**Expected Result**: Description of what you should see/have at this point.
</checkpoint>

### Step 3: Experimentation

Description of the third step.

```python
# Code for step 3
```

<checkpoint>
**Expected Result**: Description of what you should see/have at this point.
</checkpoint>

## Expected Outcomes

Upon successful completion:

1. **Artifact**: Description of deliverable
2. **Understanding**: What concept should now be clear
3. **Skill**: What practical skill has been developed

## Submission

Submit the following:

- [ ] Completed Jupyter notebook with all cells executed
- [ ] Written responses to reflection questions
- [ ] Screenshot of final simulation result

## Grading Rubric

| Criterion | Points | Excellent | Good | Satisfactory | Needs Improvement |
|-----------|--------|-----------|------|--------------|-------------------|
| Code Correctness | 40 | All code runs without errors, produces correct output | Minor issues, mostly correct | Some errors, partial functionality | Major errors, incomplete |
| Documentation | 20 | Well-commented, clear explanations | Adequate comments | Minimal comments | Missing comments |
| Results Analysis | 25 | Thorough analysis, insightful observations | Good analysis | Basic analysis | Little to no analysis |
| Reflection Questions | 15 | Thoughtful, comprehensive responses | Good responses | Basic responses | Incomplete responses |

**Total Points**: 100

## Hints

<details>
<summary>Hint 1 (Click to reveal)</summary>
First hint for students who are stuck.
</details>

<details>
<summary>Hint 2 (Click to reveal)</summary>
Second hint, more specific guidance.
</details>

<details>
<summary>Hint 3 (Click to reveal)</summary>
Most detailed hint, nearly gives the solution.
</details>

## Common Errors

| Error | Cause | Fix |
|-------|-------|-----|
| ImportError: No module named 'mujoco' | MuJoCo not installed | Run `pip install mujoco` |
| FileNotFoundError | Incorrect model path | Check file path and working directory |

## Extensions

For students who finish early or want additional challenge:

### Extension 1: Advanced Task

Description of more challenging variation.

### Extension 2: Research Question

Open-ended exploration topic.

## Reflection Questions

1. What was the most challenging part of this lab? How did you overcome it?
2. How does this lab connect to real-world humanoid robotics applications?
3. What would you do differently if you were to repeat this lab?
