---
module_id: "08"
title: "Assessment Package: Locomotion"
---

# Assessment Package: Module 08 - Locomotion

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice + analysis | 45 minutes |
| Lab Exercises | 35% | Python implementations | 3 labs |
| Simulation Project | 35% | Complete locomotion system | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

---

## Theory Quiz

**Time Limit**: 45 minutes
**Passing Score**: 70%
**Attempts**: 2

### Section A: Multiple Choice (40 points)

**Q1.** The Zero Moment Point (ZMP) is defined as the point on the ground where:
- a) The center of mass projects vertically
- b) Net ground reaction force is applied
- c) Net moment of all forces is zero
- d) Robot is statically balanced

**Q2.** In the Linear Inverted Pendulum Model (LIPM), the assumption that makes the dynamics linear is:
- a) Small angles
- b) Constant angular momentum
- c) Constant CoM height
- d) Massless legs

**Q3.** During walking, the support polygon is:
- a) Always both feet
- b) The convex hull of all ground contacts
- c) A circle around the stance foot
- d) Determined by the CoM position

**Q4.** The capture point represents:
- a) Where the ZMP should be placed
- b) Where the foot must be placed to stop without falling
- c) The current CoM position
- d) The center of the support polygon

**Q5.** Froude number Fr = v²/(gL) helps predict gait transition. Walking is preferred when:
- a) Fr > 1
- b) Fr < 0.5
- c) Fr = 1
- d) Fr is minimized

**Q6.** In the Raibert hopping controller, forward speed is controlled by:
- a) Leg stiffness
- b) Hip torque during stance
- c) Touchdown leg angle
- d) Flight duration

**Q7.** The Spring-Loaded Inverted Pendulum (SLIP) model differs from LIPM by:
- a) Having constant CoM height
- b) Including leg spring elasticity
- c) Having no flight phase
- d) Being 3-dimensional

**Q8.** Double support phase in walking is important for:
- a) Speed increase
- b) Energy injection
- c) CoM transfer between feet
- d) Reducing foot forces

**Q9.** Cost of Transport (CoT) measures:
- a) Energy per unit mass per unit distance
- b) Total energy consumed
- c) Maximum speed achievable
- d) Time to complete task

**Q10.** For a biped to maintain balance, the ZMP must:
- a) Equal the CoM projection
- b) Stay within the support polygon
- c) Stay at the foot center
- d) Move opposite to the CoM

### Section B: Analysis Problems (60 points)

**Q11.** (20 points) A simplified biped has mass m = 70 kg and CoM height z_c = 0.9 m. Using the LIPM:

a) Calculate the natural frequency ω = √(g/z_c).
b) If the CoM is displaced 5 cm forward from the ZMP and has zero velocity, what is the initial CoM acceleration?
c) Design a state feedback controller u = ZMP = x_com + K₁·x_error + K₂·x_dot to place poles at s = -ω. What are K₁ and K₂?

**Q12.** (20 points) A walking robot takes steps of length L = 0.5 m with step duration T = 0.5 s.

a) What is the average walking speed?
b) If the robot has leg length 1.0 m, what is the Froude number?
c) Is this speed in the walking or running regime? Justify.
d) If the robot wants to double its speed, should it take longer steps or faster steps (or both)? Why?

**Q13.** (20 points) A SLIP model has mass 50 kg, leg length 0.8 m, and runs at 3 m/s with apex height 0.85 m.

a) Calculate the total mechanical energy at apex (KE + PE).
b) During stance, the leg compresses 0.1 m. What spring stiffness is needed to store the kinetic energy change? (Assume all vertical KE converts to spring PE at mid-stance)
c) If the robot loses 5% energy per stride, what is the Cost of Transport?

---

## Lab Exercises

### Lab 08-01: Balance and Standing (30% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning  (Below 50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| ZMP Computation | Correct from contact forces | Minor errors in force handling | Partially working | Non-functional |
| LIPM Controller | Stable balance, proper gains | Working but not well-tuned | Oscillatory | Unstable |
| Perturbation Response | Recovers from >50N push | Recovers from moderate push | Marginal recovery | Falls |
| Stability Analysis | Margin and capture point correct | Basic metrics computed | Incomplete | Missing |
| Documentation | Clear analysis of stability | Basic documentation | Minimal | None |

### Lab 08-02: Walking Gait (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (Below 50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Footstep Planning | Correct alternation, timing | Working with minor issues | Partial sequence | Non-functional |
| ZMP Trajectory | Smooth, follows footsteps | Working but discontinuous | Partial tracking | Non-functional |
| CoM Generation | Preview control stable | Basic tracking | Unstable | Non-functional |
| Swing Trajectories | Smooth with clearance | Working but jerky | Foot scraping | Non-functional |
| Walking Execution | 6+ steps without falling | 3-5 stable steps | 1-2 steps | Falls immediately |

### Lab 08-03: Dynamic Locomotion (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (Below 50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| SLIP Model | Correct stance/flight dynamics | Working with minor errors | Partial implementation | Non-functional |
| Raibert Controller | Speed regulation within 10% | Within 20% | Inconsistent | Cannot regulate |
| Energy Analysis | Correct CoT, energy tracking | Basic energy computation | Incomplete | Missing |
| Gait Comparison | Quantitative walk/run analysis | Basic comparison | Incomplete | Missing |
| Running Execution | 5+ stable strides | 3-4 strides | 1-2 strides | Cannot run |

---

## Simulation Project

### Project: Bipedal Walking Robot with Push Recovery

**Objective**: Develop a complete bipedal walking system that can walk forward and recover from lateral pushes.

**Duration**: 1 week
**Deliverables**: Code repository + 4-page technical report

### Requirements

1. **Balance Controller** (20%)
   - Implement ZMP-based standing balance
   - Demonstrate recovery from 30N lateral push
   - Compute and display stability margins in real-time

2. **Walking Controller** (30%)
   - Generate footsteps for commanded velocity
   - Produce smooth CoM trajectory with preview
   - Execute swing leg trajectories with ground clearance
   - Walk at least 10 steps without falling

3. **Push Recovery** (30%)
   - Detect pushes through CoM acceleration
   - Adjust footstep placement for capture
   - Demonstrate recovery from pushes during walking

4. **Analysis** (20%)
   - Measure CoT for different walking speeds
   - Compare push recovery success rate
   - Analyze stability margins during walking
   - Document controller tuning process

### Evaluation Scenarios

1. **Standing push**: 50N lateral push while standing
2. **Walking push (small)**: 30N push during walking
3. **Walking push (large)**: 60N push during walking
4. **Speed change**: Accelerate from 0.5 to 1.0 m/s while walking

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Standing Balance | 20 | Stable standing with push recovery |
| Walking | 30 | 10+ steps at commanded speed |
| Push Recovery | 30 | Recovery from walking pushes |
| Analysis | 10 | CoT, stability metrics |
| Report | 10 | Clear documentation |
| **Total** | **100** | |

### Bonus Challenges (+10 points each)

- Running gait implementation
- Turning while walking
- Step over obstacle

---

## Ethics Discussion

### Prompt

*In a 500-word reflection, address the following scenario:*

A company develops a fast-moving delivery robot that uses running gaits to achieve 8 m/s delivery speeds on sidewalks. The robot weighs 30 kg and can stop within 2 meters at full speed. Testing shows zero collisions in 10,000 km of operation, but pedestrians report feeling "intimidated" and "forced to move" when the robot approaches.

The company argues:
- The robot has never caused injury
- Faster delivery means fewer robots needed total
- The robot always stops if collision is imminent
- Pedestrians can hear it coming from 20m away

Critics argue:
- 8 m/s on sidewalks is unreasonably fast
- "Forced to move" is a form of harm even without contact
- The sidewalk should feel comfortable for people, not efficient for robots
- Past safety doesn't guarantee future safety

**Address the following:**

1. Is zero collisions sufficient evidence of safety? What else should be considered?

2. How should we weigh efficiency benefits against pedestrian comfort? Is "feeling forced to move" a legitimate concern?

3. What speed limit, if any, should apply to robots on sidewalks? Should it depend on robot weight, pedestrian density, or other factors?

4. Who should decide acceptable robot behavior in public spaces—companies, cities, or the affected public?

### Rubric

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (Below 50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Safety Analysis | Considers multiple dimensions beyond collision | Notes some limitations | Only considers collisions | Accepts company claim |
| Efficiency vs Comfort | Thoughtful weighing of tradeoffs | Acknowledges both | One-sided | Ignores tradeoff |
| Speed Regulation | Specific, justified proposal | General recommendation | Vague | No proposal |
| Governance | Multiple stakeholders considered | Some consideration | Single perspective | Ignores governance |
| Writing | Clear, well-organized | Minor issues | Some problems | Unclear |

---

## Answer Key (Instructor Access Only)

### Quiz Answers

**Section A:**
1. c) Net moment of all forces is zero
2. c) Constant CoM height
3. b) The convex hull of all ground contacts
4. b) Where the foot must be placed to stop without falling
5. b) Fr < 0.5
6. c) Touchdown leg angle
7. b) Including leg spring elasticity
8. c) CoM transfer between feet
9. a) Energy per unit mass per unit distance
10. b) Stay within the support polygon

**Section B:**

**Q11:**
a) ω = √(g/z_c) = √(9.81/0.9) = **3.30 rad/s**

b) From LIPM: ẍ_com = ω²(x_com - x_zmp)
   ẍ = (3.30)² × 0.05 = **0.54 m/s²** forward

c) State feedback: u = x_com + K₁x + K₂ẋ
   Closed loop: ẍ = ω²(x - u) = ω²x - ω²(x + K₁x + K₂ẋ)
   ẍ + ω²K₂ẋ + ω²K₁x = 0
   For poles at s = -ω: (s + ω)² = s² + 2ωs + ω²
   So: ω²K₂ = 2ω → **K₂ = 2/ω = 0.606 s**
       ω²K₁ = ω² → **K₁ = 1**

**Q12:**
a) Speed = L/T = 0.5/0.5 = **1.0 m/s**

b) Fr = v²/(gL) = 1²/(9.81×1.0) = **0.102**

c) Fr = 0.102 < 0.5, so this is in the **walking regime**. Walking is energetically preferred below Fr ≈ 0.5.

d) To double speed to 2 m/s while maintaining walking (Fr < 0.5):
   - New Fr = 4/(9.81×1.0) = 0.41 (still walking, marginally)
   - Could increase step length to 0.8m at same frequency: 0.8/0.5 = 1.6 m/s, Fr = 0.26
   - Or increase frequency: 0.5m at 4Hz = 2.0 m/s, Fr = 0.41
   - **Best**: Combination of both—longer steps are more efficient but frequency increase may be needed for high speeds

**Q13:**
a) At apex: z = 0.85m, v_horizontal = 3 m/s, v_vertical = 0
   KE = 0.5 × 50 × 3² = **225 J**
   PE = 50 × 9.81 × 0.85 = **417 J**
   Total = **642 J**

b) At mid-stance, assume all vertical KE converts to spring PE:
   From apex, falling 0.05m (0.85 - 0.8) before compression starts
   v_vertical at contact = √(2 × 9.81 × 0.05) = 0.99 m/s
   KE_vertical = 0.5 × 50 × 0.99² = 24.5 J
   Spring PE = 0.5 × k × 0.1² = 24.5 J
   k = 24.5 × 2 / 0.01 = **4900 N/m**
   (Note: This is simplified; actual stiffness depends on more detailed analysis)

c) Energy lost per stride: 0.05 × 642 = 32.1 J
   Stride length ≈ 2 × 3 × 0.3 = 1.8 m (assuming 0.3s per stride)
   Weight = 50 × 9.81 = 490.5 N
   CoT = 32.1 / (490.5 × 1.8) = **0.036** (dimensionless)
   (This is quite efficient; real running CoT is typically 0.1-0.3)

---

## Export Formats

This assessment package is available in:
- Markdown (this document)
- Canvas LMS import package
- PDF with answer key (instructor version)
- Gradescope autograder configuration
