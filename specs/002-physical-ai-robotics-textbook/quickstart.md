# Quickstart Guide: Physical AI and Humanoid Robotics Textbook

**Feature Branch**: `002-physical-ai-robotics-textbook`
**Date**: 2025-12-21

## Overview

This guide helps contributors get started with developing content for the Physical AI and Humanoid Robotics textbook. It covers environment setup, content structure, and development workflows.

---

## Prerequisites

### Required Knowledge
- Familiarity with Markdown and YAML
- Basic Python programming (for code examples)
- Understanding of robotics fundamentals (for content review)
- Git version control

### Required Tools
- Git 2.30+
- Python 3.10+
- VS Code or similar Markdown editor (recommended: with YAML and Python extensions)
- Docker Desktop (for simulation environments)

---

## Environment Setup

### 1. Clone Repository

```bash
git clone <repository-url>
cd constitution
git checkout 002-physical-ai-robotics-textbook
```

### 2. Install Python Dependencies

```bash
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

pip install -r requirements.txt
```

### 3. Install Content Validation Tools

```bash
pip install pyyaml jsonschema markdown-it-py pytest
```

### 4. Verify Setup

```bash
# Run content validation
python tests/content/validate-structure.py

# Run code example tests
pytest tests/code-examples/
```

---

## Content Structure

### Directory Layout

```
textbook/
├── modules/                    # Main content (14 modules)
│   ├── 01-introduction-physical-ai/
│   │   ├── theory.md          # Conceptual content
│   │   ├── labs/              # Hands-on exercises
│   │   │   ├── lab-01-01.md
│   │   │   ├── lab-01-02.md
│   │   │   └── lab-01-03.md
│   │   ├── simulations/       # Simulation configs
│   │   │   └── mujoco-config.yaml
│   │   ├── ethics.md          # Ethics section
│   │   └── assessment.md      # Quizzes and rubrics
│   └── ... (02-14)
├── case-studies/              # Industry examples
├── appendices/                # Reference materials
├── assets/                    # Images, diagrams, code
└── build/                     # Generated outputs
```

### Module Template

Each module directory must contain:

| File | Purpose | Required |
|------|---------|----------|
| `theory.md` | Main conceptual content | Yes |
| `labs/lab-XX-NN.md` | Hands-on exercises (3-5) | Yes |
| `simulations/*.yaml` | Simulation configurations | Yes |
| `ethics.md` | Ethics content for module | Yes |
| `assessment.md` | Quiz and rubric definitions | Yes |

---

## Creating Content

### Step 1: Start from Template

```bash
# Copy module template
cp -r textbook/modules/_template textbook/modules/XX-module-slug

# Rename placeholder files
cd textbook/modules/XX-module-slug
```

### Step 2: Fill Module Metadata

Edit the frontmatter in `theory.md`:

```yaml
---
id: "05"
title: "Dynamics and Control"
slug: "dynamics-control"
week: 5
difficulty: intermediate
prerequisites:
  - "02"
  - "03"
learning_objectives:
  - "Derive equations of motion for articulated robot systems"
  - "Implement PID and computed torque controllers"
  - "Analyze stability of closed-loop control systems"
  - "Tune controller gains using simulation"
estimated_hours: 12
status: draft
---
```

### Step 3: Write Theory Content

Follow the content block structure:

```markdown
## Section Title

<definition id="def-torque">
**Torque**: The rotational equivalent of force, causing angular acceleration
around a joint axis.
</definition>

The relationship between torque and motion is governed by:

<equation id="eq-newton-euler">
$$\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)$$
</equation>

where:
- $M(q)$ is the mass matrix
- $C(q, \dot{q})$ represents Coriolis and centrifugal terms
- $g(q)$ is the gravity vector

<example id="ex-2dof-arm">
### Two-DOF Arm Dynamics
Consider a planar two-link manipulator...
</example>

<warning>
Joint limits must be respected in all control implementations to prevent
hardware damage.
</warning>
```

### Step 4: Create Lab Exercises

Use the lab template in `labs/_template.md`:

```markdown
---
id: lab-05-01
module_id: "05"
title: "Implementing PID Control for a 2-DOF Arm"
difficulty: guided
tier: simulation
duration_minutes: 60
---

## Objectives

- [ ] Implement a PID controller in Python
- [ ] Tune gains using Ziegler-Nichols method
- [ ] Analyze step response characteristics

## Materials

| Type | Name | Tier |
|------|------|------|
| software | MuJoCo 3.0+ | required |
| software | Python 3.10+ | required |
| simulation | 2dof-arm.xml | required |

## Instructions

### Step 1: Load the Simulation Environment

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("assets/models/2dof-arm.xml")
data = mujoco.MjData(model)
```

<checkpoint>
**Expected**: Model loads without errors, `model.nq == 2`
</checkpoint>

### Step 2: Implement PID Controller

...
```

### Step 5: Add Ethics Content

In `ethics.md`:

```markdown
---
module_id: "05"
---

## Ethics Primer

Control systems in robotics raise important questions about autonomy,
safety, and accountability. When a robot's controller fails, determining
responsibility—whether in design, implementation, or operation—becomes
a critical ethical and legal question.

## Callouts

<ethics-callout id="ec-05-01" type="consideration" trigger="PID tuning">
Aggressive controller tuning may improve performance but increases risk
of instability. In safety-critical applications, conservative tuning with
adequate safety margins is ethically required.
</ethics-callout>

## Reflection

### Discussion Questions

1. Who bears responsibility when a robot controller fails and causes harm?
2. How should we balance performance optimization against safety margins?
3. What testing standards should be required before deploying controllers
   on physical robots?

### Stakeholders to Consider
- Robot operators
- People in proximity to the robot
- System integrators
- Control engineers

### Applicable Frameworks
- Safety engineering principles (ALARP)
- IEEE ethics guidelines for autonomous systems
- Professional engineering codes of conduct
```

---

## Code Examples

### Writing Testable Code

All code examples must be extractable and testable:

```python
# textbook/assets/code-examples/05/pid_controller.py

def pid_controller(error, error_integral, error_derivative, kp, ki, kd):
    """
    Compute PID control output.

    Args:
        error: Current error (setpoint - measured)
        error_integral: Accumulated error over time
        error_derivative: Rate of change of error
        kp, ki, kd: Controller gains

    Returns:
        Control output (torque command)

    Example:
        >>> pid_controller(1.0, 0.5, -0.1, 10, 1, 2)
        10.3
    """
    return kp * error + ki * error_integral + kd * error_derivative
```

### Test File

```python
# tests/code-examples/test_05_dynamics_control.py

import pytest
from textbook.assets.code_examples.module_05.pid_controller import pid_controller

def test_pid_proportional_only():
    """P-only control with zero integral and derivative."""
    result = pid_controller(1.0, 0.0, 0.0, 10.0, 0.0, 0.0)
    assert result == 10.0

def test_pid_full_controller():
    """Full PID with all terms active."""
    result = pid_controller(1.0, 0.5, -0.1, 10, 1, 2)
    assert result == pytest.approx(10.3)
```

---

## Validation

### Run Structure Validation

```bash
python tests/content/validate-structure.py --module 05
```

This checks:
- Required files present
- Frontmatter valid against schema
- Cross-references resolve
- Learning objectives count (3-5)
- Lab count (3-5)

### Run Code Tests

```bash
pytest tests/code-examples/test_05*.py -v
```

### Preview Content

```bash
# Generate HTML preview
python scripts/preview.py textbook/modules/05-dynamics-control

# Open in browser
open build/preview/05-dynamics-control/index.html
```

---

## Simulation Environments

### MuJoCo Setup (Primary)

```bash
# Install MuJoCo
pip install mujoco

# Verify installation
python -c "import mujoco; print(mujoco.__version__)"
```

### Running Simulations

```python
import mujoco
import mujoco.viewer

# Load model
model = mujoco.MjModel.from_xml_path("textbook/assets/models/humanoid.xml")
data = mujoco.MjData(model)

# Launch interactive viewer
mujoco.viewer.launch(model, data)
```

### Gazebo/ROS2 Setup (Secondary)

```bash
# Use Docker for consistent environment
docker pull osrf/ros:humble-desktop

# Run container with GUI support
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/textbook:/textbook \
  osrf/ros:humble-desktop
```

---

## Contributing Workflow

### 1. Create Feature Branch

```bash
git checkout 002-physical-ai-robotics-textbook
git pull origin 002-physical-ai-robotics-textbook
git checkout -b content/module-05-dynamics-control
```

### 2. Make Changes

Follow the content structure and style guides.

### 3. Validate

```bash
# Run all validations
./scripts/validate-all.sh

# Or specific module
python tests/content/validate-structure.py --module 05
pytest tests/code-examples/test_05*.py
```

### 4. Submit PR

```bash
git add .
git commit -m "Add Module 05: Dynamics and Control content"
git push origin content/module-05-dynamics-control
```

Create PR against `002-physical-ai-robotics-textbook` branch.

---

## Style Guide

### Markdown Conventions

- Use ATX headers (`#`, `##`, `###`)
- One sentence per line (for better diffs)
- Code blocks with language specifier
- Custom components in HTML-like tags

### Terminology

| Term | Usage |
|------|-------|
| Physical AI | The field (capitalize) |
| humanoid robot | Generic reference (lowercase) |
| MuJoCo | Simulator name (exact case) |
| ROS2 | Framework name (no space) |

### Code Style

- Python: Follow PEP 8, use type hints
- YAML: 2-space indentation
- Include docstrings for all functions

---

## Getting Help

- **Questions**: Open a GitHub Discussion
- **Bugs**: File a GitHub Issue
- **Content Review**: Request review from module maintainer

## Next Steps

1. Review `specs/002-physical-ai-robotics-textbook/spec.md` for requirements
2. Review `data-model.md` for entity definitions
3. Check assigned modules in `tasks.md` (when generated)
4. Start with the module template and build incrementally
