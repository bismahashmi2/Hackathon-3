---
module_id: "04"
title: "Assessment Package: Sensors and Perception"
---

# Assessment Package: Module 04 - Sensors and Perception

## Assessment Overview

| Component | Weight | Format | Duration |
|-----------|--------|--------|----------|
| Theory Quiz | 15% | Multiple choice + short answer | 30 minutes |
| Lab Exercises | 35% | Jupyter notebooks | 3 labs |
| Simulation Project | 35% | Code + report | 1 week |
| Ethics Discussion | 15% | Written reflection | 500 words |
| **Total** | **100%** | | |

---

## Theory Quiz

**Time Limit**: 30 minutes
**Passing Score**: 70%
**Attempts**: 2

### Section A: Multiple Choice (40 points)

**Q1.** An accelerometer at rest on a horizontal surface measures:
- a) Zero acceleration in all axes
- b) Approximately 9.81 m/s² in the vertical axis
- c) The velocity of the surface
- d) Only dynamic accelerations, not gravity

**Q2.** A gyroscope measures:
- a) Linear acceleration
- b) Angular position
- c) Angular velocity
- d) Magnetic field direction

**Q3.** The primary advantage of sensor fusion compared to using individual sensors is:
- a) Lower cost
- b) Simpler implementation
- c) Improved accuracy and robustness
- d) Reduced power consumption

**Q4.** In a camera's pinhole model, the focal length determines:
- a) The image resolution
- b) The field of view
- c) The color accuracy
- d) The frame rate

**Q5.** Which sensor is most affected by magnetic interference from motors?
- a) Accelerometer
- b) Gyroscope
- c) Magnetometer
- d) LIDAR

**Q6.** Depth cameras using structured light work by:
- a) Measuring time-of-flight of laser pulses
- b) Projecting a known pattern and measuring distortion
- c) Using two cameras for stereo vision
- d) Detecting infrared radiation from objects

**Q7.** IMU drift primarily affects which sensor?
- a) Accelerometer
- b) Gyroscope
- c) Magnetometer
- d) All equally

**Q8.** The Kalman filter prediction step uses:
- a) Only sensor measurements
- b) Only the system model
- c) Both measurements and model
- d) Neither—it initializes the state

**Q9.** Which coordinate frame is typically fixed to the robot's body?
- a) World frame
- b) Body frame
- c) Camera frame
- d) Inertial frame

**Q10.** LIDAR measures distance using:
- a) Sound waves (echolocation)
- b) Radio waves (radar)
- c) Light pulses (time-of-flight)
- d) Infrared heat signatures

### Section B: Short Answer (60 points)

**Q11.** (15 points) Explain why accelerometers alone cannot determine heading (yaw angle) but can estimate roll and pitch. Include a diagram if helpful.

**Q12.** (15 points) A robot's camera has a 60° vertical field of view and produces 480-pixel tall images. Calculate the vertical focal length in pixels. Show your work.

**Q13.** (15 points) Describe the complementary filter approach to orientation estimation. What are its advantages over a pure gyroscope integration approach?

**Q14.** (15 points) A depth camera returns a value of 0.65 for a pixel when the near plane is 0.1m and far plane is 10m. Calculate the metric depth. If the pixel is at coordinates (320, 240) in a 640×480 image with focal length 500 pixels, what are the 3D coordinates?

---

## Lab Exercises

### Lab 04-01: IMU Data Reading (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Sensor Access | Correctly reads all IMU channels with proper indexing | Minor indexing issues | Reads some but not all channels | Cannot access sensor data |
| Data Collection | Efficient time-series collection with proper pre-allocation | Working but inefficient | Incomplete data collection | Non-functional |
| Visualization | Publication-quality plots with labels, legend, units | Functional plots missing some elements | Basic plots | No visualization |
| Noise Model | Realistic bias + variance model with justified parameters | Working noise model | Incomplete noise model | No noise modeling |
| Filtering | Effective filtering with tuned parameters | Working filter with default parameters | Attempted filtering | No filtering |

### Lab 04-02: Camera Processing (35% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Rendering | Correctly renders RGB and depth from arbitrary cameras | Minor issues with camera selection | Renders only default view | Cannot render |
| Intrinsics | Accurate matrix with correct FOV conversion | Minor calculation errors | Partially correct | Incorrect intrinsics |
| Depth Processing | Accurate metric conversion with proper handling of edge cases | Working conversion | Incomplete processing | Non-functional |
| Segmentation | Robust detection with morphological cleanup | Basic detection | Oversimplified approach | No detection |
| Point Cloud | Valid 3D reconstruction with proper coordinate handling | Minor geometric errors | Incomplete reconstruction | Non-functional |

### Lab 04-03: Sensor Fusion EKF (30% of lab grade)

**Grading Rubric:**

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| State Design | Complete state vector with proper covariance initialization | Minor design issues | Incomplete state | Incorrect state |
| Prediction | Correct nonlinear integration with accurate Jacobian | Minor Jacobian errors | Partially working | Non-functional |
| Update | Proper measurement model with innovation computation | Minor issues | Incomplete update | Non-functional |
| Integration | Working fusion with good tracking performance | Working but suboptimal | Partially integrated | Not integrated |
| Evaluation | Thorough RMSE analysis with insightful interpretation | Basic evaluation | Incomplete analysis | No evaluation |

---

## Simulation Project

### Project: Multi-Sensor Perception System

**Objective**: Build a perception system that fuses IMU and camera data to track a moving target while estimating the robot's own orientation.

**Duration**: 1 week
**Deliverables**: Code repository + 3-page technical report

### Requirements

1. **IMU-based Orientation Estimation** (25%)
   - Implement EKF or complementary filter
   - Demonstrate tracking accuracy < 5° RMSE for roll/pitch

2. **Visual Target Detection** (25%)
   - Detect colored marker in camera images
   - Track marker centroid across frames
   - Handle temporary occlusions gracefully

3. **3D Target Localization** (25%)
   - Combine depth data with detection to estimate 3D position
   - Transform to world coordinates using orientation estimate

4. **Integration and Testing** (25%)
   - Run on provided test sequences
   - Generate performance report with metrics
   - Document failure cases and limitations

### Grading Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Orientation Accuracy | 25 | RMSE for roll, pitch against ground truth |
| Detection Robustness | 25 | Success rate across lighting and occlusion |
| Localization Accuracy | 25 | 3D position error relative to ground truth |
| Code Quality | 15 | Modularity, documentation, reproducibility |
| Report Quality | 10 | Clear writing, proper figures, analysis depth |
| **Total** | **100** | |

### Test Sequences

1. **Static Scene**: Stationary robot, moving target
2. **Robot Motion**: Moving robot, stationary target
3. **Full Dynamic**: Both robot and target moving
4. **Challenging**: Low light, partial occlusions, fast motion

---

## Ethics Discussion

### Prompt

*In a 500-word reflection, address the following scenario:*

A university research lab develops an autonomous delivery robot for campus use. The robot is equipped with cameras, LIDAR, and microphones "for safety and navigation." After successful trials, the university administration requests access to the robot's sensor logs to:

1. Monitor mask compliance during a health emergency
2. Identify individuals involved in a campus protest
3. Track building occupancy for energy management

**Address the following in your reflection:**
- Which, if any, of these requests are ethically justifiable? Why or why not?
- What technical design choices could have prevented this situation?
- Who should have decision-making authority over sensor data use?
- How does the original stated purpose ("safety and navigation") relate to these secondary uses?

### Rubric

| Criterion | Excellent (90-100%) | Proficient (70-89%) | Developing (50-69%) | Beginning (<50%) |
|-----------|---------------------|---------------------|---------------------|------------------|
| Ethical Analysis | Nuanced evaluation of each request with clear reasoning | Sound analysis with minor gaps | Basic ethical reasoning | Superficial or missing analysis |
| Technical Solutions | Specific, feasible design recommendations | General technical suggestions | Vague technical mentions | No technical discussion |
| Stakeholder Consideration | Comprehensive stakeholder analysis | Most stakeholders considered | Limited stakeholder view | Ignores stakeholders |
| Writing Quality | Clear, well-organized, persuasive | Clear with minor issues | Some clarity problems | Unclear or disorganized |

---

## Answer Key (Instructor Access Only)

### Quiz Answers

**Section A:**
1. b) Approximately 9.81 m/s² in the vertical axis
2. c) Angular velocity
3. c) Improved accuracy and robustness
4. b) The field of view
5. c) Magnetometer
6. b) Projecting a known pattern and measuring distortion
7. b) Gyroscope
8. b) Only the system model
9. b) Body frame
10. c) Light pulses (time-of-flight)

**Section B:**

**Q11:** Accelerometers measure the gravity vector, which provides a reference for "down." Roll rotates around the forward axis, tilting the gravity measurement in the Y-Z plane. Pitch rotates around the lateral axis, tilting in the X-Z plane. However, yaw (rotation around the vertical axis) does not change the gravity vector's direction relative to the sensor—gravity still points straight down regardless of heading. Therefore, accelerometers cannot distinguish different yaw angles.

**Q12:**
- FOV_y = 60° = π/3 radians
- height = 480 pixels
- f_y = height / (2 × tan(FOV_y/2))
- f_y = 480 / (2 × tan(30°))
- f_y = 480 / (2 × 0.577)
- f_y = 480 / 1.155 = **415.7 pixels**

**Q13:** The complementary filter combines gyroscope integration (for fast dynamics) with accelerometer measurements (for drift correction) using a weighted blend:
```
angle = α × (angle + gyro × dt) + (1-α) × accel_angle
```
Advantages over pure gyro: eliminates drift over time while maintaining good short-term response. Simple to implement, computationally efficient, no complex state estimation required.

**Q14:**
- Metric depth: z = 0.1 + 0.65 × (10 - 0.1) = 0.1 + 6.435 = **6.535 m**
- Principal point: cx=320, cy=240; Pixel at (320, 240) is the center
- X = (320 - 320) × 6.535 / 500 = **0 m**
- Y = (240 - 240) × 6.535 / 500 = **0 m**
- Z = **6.535 m**
- 3D coordinates: **(0, 0, 6.535) meters**

---

## Export Formats

This assessment package is available in:
- Markdown (this document)
- Canvas LMS import package
- PDF with answer key (instructor version)
- Gradescope autograder configuration (for lab submissions)
