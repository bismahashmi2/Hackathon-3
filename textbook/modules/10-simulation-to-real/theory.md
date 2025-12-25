---
id: "10"
title: "Simulation to Real"
slug: "simulation-to-real"
week: 10
difficulty: intermediate
prerequisites: ["07", "08", "09"]
learning_objectives:
  - "Apply domain randomization techniques for robust policy transfer"
  - "Identify and mitigate the reality gap between simulation and physical systems"
  - "Validate simulation models against real robot behavior"
  - "Deploy trained policies from simulation to physical robots"
estimated_hours: 12
status: draft
last_updated: 2025-01-01
version: "1.0.0"
---

# Module 10: Simulation to Real

## Introduction

Training in simulation is faster, safer, and cheaper than real-world training. But the "reality gap" between simulation and reality can cause policies to fail. This module covers techniques for successful sim-to-real transfer.

## Section 1: The Reality Gap

### 1.1 Sources of Discrepancy

<definition id="def-reality-gap">
**Reality Gap**: The difference between simulated and real-world dynamics that causes policies trained in simulation to underperform or fail on physical systems.
</definition>

Sources include:
- Inaccurate physics parameters
- Unmodeled friction and contact
- Sensor noise differences
- Actuator delays and dynamics

### 1.2 Quantifying the Gap

System identification and validation experiments.

## Section 2: Domain Randomization

### 2.1 Randomizing Dynamics

```python
def randomize_simulation():
    # Randomize physical properties
    sim.mass = nominal_mass * np.random.uniform(0.8, 1.2)
    sim.friction = np.random.uniform(0.3, 1.0)
    sim.motor_delay = np.random.uniform(0.01, 0.05)

    # Randomize visual properties
    sim.lighting = random_lighting()
    sim.textures = random_textures()

    return sim
```

### 2.2 Curriculum Learning

Progressively increasing randomization difficulty.

<warning>
Too much randomization can make learning impossible. Start with narrow ranges and expand gradually.
</warning>

## Section 3: System Identification

### 3.1 Parameter Estimation

Fitting simulation parameters to real data:

<equation id="eq-sysid">
$$\boldsymbol{\theta}^* = \arg\min_{\boldsymbol{\theta}} \|\mathbf{y}_{real} - \mathbf{y}_{sim}(\boldsymbol{\theta})\|^2$$
</equation>

### 3.2 Model Validation

Cross-validation with held-out trajectories.

## Section 4: Deployment

### 4.1 Safety Considerations

- Gradual testing protocol
- Emergency stops
- Monitoring for anomalies

### 4.2 Online Adaptation

Fine-tuning with real-world data after initial deployment.

## Summary

Key takeaways:
1. Reality gap causes simulation-trained policies to fail
2. Domain randomization builds robustness
3. System identification improves simulation accuracy
4. Careful deployment protocols ensure safety

## Key Concepts

- **Reality Gap**: Sim-real discrepancy
- **Domain Randomization**: Training with varied parameters
- **System Identification**: Estimating model parameters
- **Transfer Learning**: Applying simulation knowledge to reality

## Further Reading

1. Tobin et al. (2017). "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World"
2. Zhao et al. (2020). "Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics"
