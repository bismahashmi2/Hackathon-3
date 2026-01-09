---
module_id: "05"
title: "Assessment Package: Dynamics and Control"
---

# Assessment Package: Module 05 - Dynamics and Control

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice + derivations | 45 minutes |
| Lab Exercises | 35% | Python implementations | 3 labs |
| Simulation Project | 35% | Controller design + analysis | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

---

## Theory Quiz

**Time Limit**: 45 minutes
**Passing Score**: 70%
**Attempts**: 2

### Section A: Multiple Choice (40 points)

**Q1.** In a PID controller, the integral term primarily serves to:
- a) Increase response speed
- b) Reduce overshoot
- c) Eliminate steady-state error
- d) Dampen oscillations

**Q2.** For a second-order system with natural frequency ωn, critical damping occurs when the damping ratio ζ equals:
- a) 0
- b) 0.5
- c) 1.0
- d) 2.0

**Q3.** The mass matrix M(q) in the robot dynamics equation M(q)q̈ + C(q,q̇)q̇ + g(q) = τ is:
- a) Always diagonal
- b) Always symmetric positive definite
- c) Constant for all configurations
- d) Independent of joint positions

**Q4.** Computed torque control achieves linearized dynamics by:
- a) Ignoring nonlinear terms
- b) Using high gains to dominate nonlinearities
- c) Canceling nonlinear dynamics with model-based feedforward
- d) Approximating the system as linear around an operating point

**Q5.** Anti-windup in PID control prevents:
- a) Integral term from growing unboundedly during saturation
- b) Derivative term from amplifying noise
- c) Proportional term from causing overshoot
- d) The controller from responding to step inputs

**Q6.** In the Ziegler-Nichols tuning method, Ku represents:
- a) The ultimate (critical) gain causing sustained oscillation
- b) The gain at which the system becomes unstable
- c) The optimal proportional gain
- d) The gain margin of the system

**Q7.** The Coriolis matrix C(q, q̇) in robot dynamics represents forces due to:
- a) Gravity alone
- b) Friction in joints
- c) Motion-dependent coupling between joints
- d) External disturbances

**Q8.** An overdamped system response:
- a) Oscillates before settling
- b) Returns to equilibrium without oscillation
- c) Takes infinite time to settle
- d) Becomes unstable

**Q9.** Model-based control requires:
- a) Perfect knowledge of system dynamics
- b) An approximate model of system dynamics
- c) Only input-output data
- d) Real-time system identification

**Q10.** The primary advantage of computed torque over PID for multi-joint robots is:
- a) Simpler implementation
- b) No need for sensor feedback
- c) Decoupled joint dynamics
- d) Lower computational cost

### Section B: Short Answer and Derivations (60 points)

**Q11.** (15 points) Given a 1-DOF system with transfer function G(s) = K/(s² + 2ζωₙs + ωₙ²), derive the relationship between settling time (2% criterion) and the damping ratio ζ and natural frequency ωₙ for an underdamped system.

**Q12.** (15 points) For a 2-DOF robot arm, the mass matrix is:
```
M(q) = [a + b*cos(q₂)    c + d*cos(q₂)]
       [c + d*cos(q₂)    e            ]
```
Show that M(q) is symmetric. Under what conditions is M(q) positive definite?

**Q13.** (15 points) A joint controlled by PID with Kp=100, Ki=10, Kd=20 has a steady-state position of 0.98 rad when the setpoint is 1.0 rad. A constant disturbance torque is causing the error. Calculate the magnitude of the disturbance torque and explain why the integral term hasn't eliminated the error.

**Q14.** (15 points) Explain why derivative gain Kd on the error signal can cause "derivative kick" when the setpoint changes suddenly. Describe an alternative implementation that avoids this problem.

---

## Lab Exercises

### Lab 05-01: PID Control (30% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (  &lt;50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| PID Implementation | Complete with anti-windup and proper discretization | Working without anti-windup | Partial implementation | Non-functional |
| Step Response Analysis | Accurate metrics with proper definitions | Minor calculation errors | Incomplete metrics | Missing analysis |
| Gain Tuning | Systematic exploration with clear conclusions | Working comparison | Limited exploration | No tuning study |
| Visualization | Publication-quality with annotations | Functional plots | Basic plots | No visualization |
| Code Quality | Well-documented, modular, tested | Working code | Partially working | Non-functional |

### Lab 05-02: Computed Torque Control (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (  &lt;50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Dynamics Extraction | Correctly extracts M and bias from MuJoCo | Minor issues | Partial extraction | Cannot extract |
| CTC Implementation | Proper inverse dynamics with feedforward | Working without feedforward | Partial implementation | Non-functional |
| Comparison Study | Fair comparison with PID, clear conclusions | Basic comparison | Limited comparison | No comparison |
| Trajectory Tracking | Excellent tracking with feedforward | Tracking without feedforward | Poor tracking | Non-functional |
| Analysis | Insightful interpretation of results | Basic analysis | Incomplete | Missing |

### Lab 05-03: Adaptive Control (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning  (Below 50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Model Perturbation | Correctly creates significant mismatch | Minor perturbation | Too small effect | Cannot perturb |
| Adaptation Law | Proper gradient update with bounds | Working update | Partial implementation | Non-functional |
| Integration | Smooth integration of adaptation with CTC | Working integration | Partial integration | Not integrated |
| Convergence | Parameter converges to correct value | Converges approximately | Slow/partial convergence | Does not converge |
| Analysis | Comprehensive stability and convergence analysis | Basic analysis | Incomplete | Missing |

---

## Simulation Project

### Project: Multi-Joint Robot Arm Controller Design

**Objective**: Design, implement, and compare control strategies for a 3-DOF robot arm performing a pick-and-place task.

**Duration**: 1 week
**Deliverables**: Code repository + 4-page technical report

### Requirements

1. **System Modeling** (20%)
   - Extract and analyze dynamics matrices for the 3-DOF arm
   - Characterize the system (natural frequencies, coupling)
   - Document model assumptions and limitations

2. **PID Controller** (20%)
   - Implement independent joint PID controllers
   - Tune using systematic method (Ziegler-Nichols or similar)
   - Achieve setpoint regulation within 5% error in 2 seconds

3. **Computed Torque Controller** (30%)
   - Implement CTC for the 3-DOF system
   - Demonstrate improved coupling rejection compared to PID
   - Achieve trajectory tracking with RMS error < 2 degrees

4. **Comparative Analysis** (30%)
   - Test both controllers on identical trajectories
   - Quantify: settling time, overshoot, tracking error, energy consumption
   - Analyze robustness to payload changes (±20% mass)
   - Provide recommendations for deployment

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| System Analysis | 20 | Correct dynamics extraction and characterization |
| PID Implementation | 20 | Working controller meeting specifications |
| CTC Implementation | 30 | Correct implementation with trajectory tracking |
| Comparison Quality | 20 | Fair, thorough comparison with clear metrics |
| Report Quality | 10 | Clear writing, proper figures, professional presentation |
| **Total** | **100** | |

### Test Trajectories

1. **Step Response**: Simultaneous step in all joints
2. **Sinusoidal Tracking**: Each joint follows sin wave at different frequency
3. **Pick-and-Place**: Move from home → pick position → place position → home
4. **Robustness Test**: Repeat with 20% increased payload mass

---

## Ethics Discussion

### Prompt

*In a 500-word reflection, address the following scenario:*

You are the control systems engineer for a collaborative robot deployed in a small manufacturing shop. The robot performs assembly tasks alongside human workers.

After six months of safe operation, a new production manager requests that you increase the robot's speed by 40% to meet a large order deadline. Your analysis shows this would:
- Increase peak velocities from 0.5 m/s to 0.7 m/s
- Reduce stopping distance from 15 cm to 25 cm
- Increase collision force from 80N to 140N (still below the 150N safety threshold)

The workers have not been consulted about this change.

**Address the following in your reflection:**
- What are your professional obligations as the control engineer in this situation?
- How should the trade-off between productivity and safety margin be decided, and by whom?
- What would you do if management insisted on the change over your objections?
- How might you approach the workers affected by this decision?

### Rubric

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning  ( &lt;50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Professional Ethics | Clear application of engineering ethics codes | Sound ethical reasoning | Basic ethical awareness | Ignores ethical dimension |
| Stakeholder Analysis | Considers all affected parties | Most stakeholders | Limited consideration | Single perspective |
| Technical Analysis | Correctly interprets safety implications | Basic technical understanding | Some technical errors | Misunderstands situation |
| Actionable Recommendations | Specific, feasible proposals | General recommendations | Vague suggestions | No recommendations |
| Writing Quality | Clear, well-organized, persuasive | Clear with minor issues | Some clarity problems | Unclear |

---

## Answer Key (Instructor Access Only)

### Quiz Answers

**Section A:**
1. c) Eliminate steady-state error
2. c) 1.0
3. b) Always symmetric positive definite
4. c) Canceling nonlinear dynamics with model-based feedforward
5. a) Integral term from growing unboundedly during saturation
6. a) The ultimate (critical) gain causing sustained oscillation
7. c) Motion-dependent coupling between joints
8. b) Returns to equilibrium without oscillation
9. b) An approximate model of system dynamics
10. c) Decoupled joint dynamics

**Section B:**

**Q11:** For underdamped second-order system (0 < ζ < 1):
- Envelope decays as exp(-ζωₙt)
- 2% settling when exp(-ζωₙtₛ) = 0.02
- -ζωₙtₛ = ln(0.02) ≈ -3.9
- **tₛ ≈ 4/(ζωₙ)** for 2% criterion

**Q12:** Symmetry: M₁₂ = c + d*cos(q₂) = M₂₁ ✓
Positive definite conditions:
- M₁₁ > 0: a + b*cos(q₂) > 0 → a > |b|
- det(M) > 0: (a + b*cos(q₂))*e - (c + d*cos(q₂))² > 0
- Typically satisfied by physical mass/inertia constraints

**Q13:** At steady state, Kp*e + Ki*∫e dt = τ_disturbance
With e = 0.02 rad steady-state, integral keeps growing (not saturating), so:
- If Ki*∫e dt has reached a limit due to anti-windup, τ_disturbance ≈ 0.02*100 = **2 Nm**
- Integral hasn't eliminated error because it's hitting anti-windup limits OR hasn't had enough time to accumulate

**Q14:** Derivative kick: When setpoint changes from r₁ to r₂ instantaneously:
- Error jumps from (r₁-y) to (r₂-y)
- de/dt → ∞ (impulse)
- Kd*de/dt produces large torque spike

**Solution**: Derivative on measurement (not error):
- Instead of Kd*d(r-y)/dt, use -Kd*dy/dt
- Setpoint changes don't affect derivative term
- Only actual system velocity is differentiated

---

## Export Formats

This assessment package is available in:
- Markdown (this document)
- Canvas LMS import package
- PDF with answer key (instructor version)
- Gradescope autograder configuration (for lab submissions)
