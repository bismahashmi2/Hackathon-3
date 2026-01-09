---
module_id: "01"
---

# Assessment Package: Introduction to Physical AI

## Overview

| Component | Weight | Format |
|-----------|--------|--------|
| Theory Quiz | 15% | Multiple choice + short answer |
| Lab Assessment | 35% | Jupyter notebooks with auto-grading |
| Simulation Project | 35% | Code submission with test harness |
| Ethics Component | 15% | Written reflection + peer review |

## Theory Quiz (15%)

**Time Limit**: 20 minutes
**Passing Score**: 70%
**Attempts Allowed**: 2

### Questions

#### Q1 (Multiple Choice, 2 points)
**Question**: What distinguishes Physical AI from traditional software AI?

**Options**:
a) Physical AI uses more computing power
b) Physical AI must interact with the real world through sensors and actuators
c) Physical AI is always humanoid in form
d) Physical AI does not use machine learning

**Correct Answer**: b
**Explanation**: Physical AI systems are defined by their need to perceive and act in the physical world, dealing with uncertainty, real-time constraints, and physical consequences that pure software AI does not face.
**Learning Objective**: Define Physical AI and distinguish it from traditional robotics and software AI

---

#### Q2 (Multiple Choice, 2 points)
**Question**: The Embodiment Hypothesis suggests that:

**Options**:
a) All intelligent robots must have human form
b) Intelligence emerges from the interaction between brain, body, and environment
c) Robots cannot be truly intelligent
d) Physical form has no effect on cognitive capabilities

**Correct Answer**: b
**Explanation**: The Embodiment Hypothesis is the principle that intelligent behavior cannot be fully separated from physical form—the body shapes cognition.
**Learning Objective**: Explain the role of embodiment in artificial intelligence

---

#### Q3 (Multiple Choice, 2 points)
**Question**: Which generation of robots introduced programmable automation?

**Options**:
a) First generation (1950s-1960s)
b) Second generation (1970s-1980s)
c) Third generation (1990s-2010s)
d) Fourth generation (2020s+)

**Correct Answer**: a
**Explanation**: First-generation industrial robots like Unimate introduced programmable automation to manufacturing, though they lacked sensors and operated in structured environments.
**Learning Objective**: Describe the historical evolution from industrial robots to intelligent humanoids

---

#### Q4 (Short Answer, 4 points)
**Question**: List three key advantages of the humanoid form factor for robots intended to operate in human environments. Explain one in detail.

**Rubric**:
- 1 point for each valid advantage listed (max 3)
- 1 point for detailed explanation

**Sample Answer**:
1. Compatible with human-designed environments (doors, stairs, tools)
2. Intuitive human-robot interaction through familiar body language
3. Versatile manipulation with human-like hands
Detailed: Human environments are designed for human bodies—standard door widths, stair heights, counter heights, and tool handles. A humanoid robot can navigate these spaces without environmental modification.
**Learning Objective**: Identify key components and capabilities of modern humanoid robot systems

---

#### Q5 (Multiple Choice, 2 points)
**Question**: In MuJoCo, what does `data.qpos` represent?

**Options**:
a) Joint velocities
b) Generalized positions (configuration)
c) Applied torques
d) Sensor readings

**Correct Answer**: b
**Explanation**: `qpos` stores the generalized position coordinates—joint angles for revolute joints and positions for free-floating bodies.
**Learning Objective**: Identify key components and capabilities of modern humanoid robot systems

---

#### Q6 (Short Answer, 4 points)
**Question**: Compare and contrast the challenges faced by Physical AI versus Software AI. Give a specific example for each.

**Rubric**:
- 2 points for correctly identifying differences
- 1 point for Physical AI example
- 1 point for Software AI example

**Sample Answer**: Physical AI must handle real-time constraints (can't pause to think longer), sensor noise, and physical consequences of failure. Software AI operates in deterministic digital environments where failures can be retried.
Example Physical AI: A robot grasping an egg must control force precisely—too much breaks it, too little drops it.
Example Software AI: A chess engine can take varying time per move and simply recalculate if interrupted.
**Learning Objective**: Define Physical AI and distinguish it from traditional robotics and software AI

---

#### Q7 (Multiple Choice, 2 points)
**Question**: Which of the following is NOT a current application domain for humanoid robots?

**Options**:
a) Manufacturing and logistics
b) Healthcare and elder care
c) Faster-than-light space travel
d) Research and education

**Correct Answer**: c
**Explanation**: While humanoids are used in manufacturing, healthcare, and research, faster-than-light travel is physically impossible and not a real application domain.
**Learning Objective**: Identify key components and capabilities of modern humanoid robot systems

---

#### Q8 (Short Answer, 2 points)
**Question**: What is the simulation timestep in the MuJoCo humanoid model, and why does timestep matter for physics simulation?

**Sample Answer**: The default timestep is typically 0.002 seconds (2ms). Timestep matters because smaller timesteps provide more accurate physics simulation (especially for contact and fast dynamics) but require more computation.
**Learning Objective**: Identify key components and capabilities of modern humanoid robot systems

---

**Total Quiz Points**: 20

## Lab Assessment (35%)

### Grading Rubric for Labs 01-01, 01-02, 01-03

| Criterion | Weight | Excellent (90-100%) | Good (70-89%) | Satisfactory (50-69%) | Needs Work (&lt;50%) |
|-----------|--------|---------------------|---------------|----------------------|-------------------|
| Code Functionality | 40% | All code runs without errors, produces correct output | Minor errors, mostly correct | Some functions work | Major errors |
| Understanding | 25% | Can explain code and concepts clearly | Explains most concepts | Partial understanding | Limited understanding |
| Documentation | 15% | Well-commented, clear notebook narrat`ive | Adequate comments | Minimal documentation | No documentation |
| Checkpoints | 20% | All checkpoints verified and documented | Most checkpoints complete | Some checkpoints done | Few checkpoints |

### Auto-Grading Tests

```python
# tests/lab_01_01_tests.py
def test_mujoco_installed():
    import mujoco
```python
```python
    assert mujoco.__version__ >= "3.0.0"
```
```

def test_model_loads():
    import mujoco
    model = mujoco.MjModel.from_xml_path(
        mujoco.util.get_resource_path("humanoid/humanoid.xml")
    )
```python
```python
    assert model.nbody > 0
```
```
```python
```python
    assert model.njnt > 0
```
```

def test_simulation_runs():
    import mujoco
    model = mujoco.MjModel.from_xml_path(
        mujoco.util.get_resource_path("humanoid/humanoid.xml")
    )
    data = mujoco.MjData(model)
    initial_time = data.time
    mujoco.mj_step(model, data)
```python
```python
    assert data.time > initial_time
```
```
```

## Simulation Project (35%)

### Project: Humanoid Model Analysis

**Description**: Create a comprehensive analysis of the MuJoCo humanoid model, documenting its structure, simulating various initial conditions, and visualizing the results.

### Deliverables

| Deliverable | Format | Points |
|-------------|--------|--------|
| Model documentation | Markdown | 20 |
| Simulation script | Python | 30 |
| Results visualization | PNG/PDF plots | 25 |
| Written analysis | Markdown | 25 |

### Requirements

1. **Model Documentation** (20 points)
   - Document all bodies, joints, and actuators
   - Create a kinematic tree diagram
   - List all sensors available

2. **Simulation Script** (30 points)
   - Implement at least 3 different initial conditions
   - Run each for 5 seconds of simulated time
   - Log position, velocity, and energy data

3. **Visualization** (25 points)
   - Plot joint trajectories over time
   - Plot total energy (kinetic + potential)
   - Create comparison plots across initial conditions

4. **Analysis** (25 points)
   - Explain observed behaviors
   - Discuss energy conservation
   - Identify any unexpected results

### Rubric

| Criterion | Excellent | Good | Satisfactory | Needs Work |
|-----------|-----------|------|--------------|------------|
| Completeness | All deliverables complete | Most complete | Partially complete | Missing items |
| Technical Accuracy | No errors | Minor errors | Some errors | Major errors |
| Insight | Deep analysis | Good observations | Basic analysis | Little analysis |
| Presentation | Professional quality | Good quality | Acceptable | Poor quality |

## Ethics Component (15%)

### Format: Written Reflection with Peer Review

**Length**: 500-750 words
**Peer Reviews**: 2 required

### Prompt

Reflect on the ethical implications of pursuing Physical AI and humanoid robotics as a career or research field. Consider:

1. What responsibilities do roboticists have to society?
2. How should potential negative impacts (job displacement, safety risks) influence research directions?
3. What ethical frameworks or principles will guide your own work in this field?

### Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Engagement with Material | 30 | References module content and case studies |
| Critical Thinking | 30 | Demonstrates nuanced analysis of trade-offs |
| Personal Reflection | 20 | Connects to own values and career goals |
| Writing Quality | 10 | Clear, well-organized writing |
| Peer Review Quality | 10 | Thoughtful feedback on others' reflections |

### Peer Review Guidelines

When reviewing peers' reflections:
- Identify one strength in their argument
- Ask one clarifying question
- Suggest one perspective they might not have considered

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
| Define Physical AI | Q1, Q6 | ✓ | ✓ | ✓ |
| Historical evolution | Q3 | | ✓ | |
| Key components | Q4, Q5, Q7, Q8 | ✓ | ✓ | |
| Role of embodiment | Q2 | ✓ | | ✓ |
