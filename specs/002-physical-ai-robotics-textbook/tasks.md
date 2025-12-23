# Tasks: Physical AI and Humanoid Robotics Textbook

**Input**: Design documents from `/specs/002-physical-ai-robotics-textbook/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/module-schema.yaml, quickstart.md

**Tests**: No test tasks included (not requested in specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Textbook content**: `textbook/modules/`, `textbook/case-studies/`, `textbook/appendices/`, `textbook/assets/`
- **Validation scripts**: `tests/content/`, `tests/code-examples/`
- **Build outputs**: `textbook/build/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create textbook directory structure (textbook/modules/, textbook/case-studies/, textbook/appendices/, textbook/assets/, textbook/build/)
- [ ] T002 Create tests directory structure (tests/content/, tests/code-examples/)
- [ ] T003 [P] Create Python requirements.txt with validation dependencies (pyyaml, jsonschema, markdown-it-py, pytest)
- [ ] T004 [P] Create module template directory at textbook/modules/_template/ with placeholder files
- [ ] T005 [P] Create lab template at textbook/modules/_template/labs/_template.md
- [ ] T006 [P] Create assets subdirectories (textbook/assets/images/, textbook/assets/diagrams/, textbook/assets/code-examples/, textbook/assets/robot-models/)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 Implement content structure validation script at tests/content/validate-structure.py (validates module frontmatter against schema)
- [ ] T008 [P] Create Python code example extraction utility at tests/content/extract-code.py (extracts code from markdown)
- [ ] T009 [P] Implement schema validation helper at tests/content/validate-schema.py (validates YAML against module-schema.yaml)
- [ ] T010 [P] Create module frontmatter validator at tests/content/validate-frontmatter.py (checks prerequisites, learning objectives count)
- [ ] T011 [P] Create lab structure validator at tests/content/validate-labs.py (ensures 3-5 labs per module, proper format)
- [ ] T012 Create validation runner script at scripts/validate-all.sh (runs all validation scripts)
- [ ] T013 [P] Create HTML preview generator at scripts/preview.py (converts markdown to HTML for preview)
- [ ] T014 [P] Setup MuJoCo simulation environment configuration at textbook/assets/robot-models/mujoco/
- [ ] T015 [P] Setup Gazebo simulation environment configuration at textbook/assets/robot-models/gazebo/
- [ ] T016 Create Docker configuration for ROS2 Humble environment at docker/ros2-humble/Dockerfile

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Comprehensive Course Structure (Priority: P1) 🎯 MVP

**Goal**: Create complete 14-module structure with logical progression from beginner to advanced topics

**Independent Test**: Can verify module progression by checking that all 14 modules exist with proper prerequisites, difficulty levels progress logically, and each module has 3-5 learning objectives

### Module Creation for User Story 1

**Beginner Modules (Weeks 1-4)**

- [ ] T017 [P] [US1] Create Module 01 directory structure at textbook/modules/01-introduction-physical-ai/
- [ ] T018 [P] [US1] Create Module 02 directory structure at textbook/modules/02-rigid-body-dynamics/
- [ ] T019 [P] [US1] Create Module 03 directory structure at textbook/modules/03-kinematics-fundamentals/
- [ ] T020 [P] [US1] Create Module 04 directory structure at textbook/modules/04-sensors-perception/
- [ ] T021 [P] [US1] Write Module 01 theory content in textbook/modules/01-introduction-physical-ai/theory.md (Physical AI overview, history, applications)
- [ ] T022 [P] [US1] Write Module 02 theory content in textbook/modules/02-rigid-body-dynamics/theory.md (Newton-Euler equations, forces, torques)
- [ ] T023 [P] [US1] Write Module 03 theory content in textbook/modules/03-kinematics-fundamentals/theory.md (Forward/inverse kinematics, DH parameters)
- [ ] T024 [P] [US1] Write Module 04 theory content in textbook/modules/04-sensors-perception/theory.md (IMU, cameras, LIDAR, sensor fusion)

**Intermediate Modules (Weeks 5-10)**

- [ ] T025 [P] [US1] Create Module 05 directory structure at textbook/modules/05-dynamics-control/
- [ ] T026 [P] [US1] Create Module 06 directory structure at textbook/modules/06-motion-planning/
- [ ] T027 [P] [US1] Create Module 07 directory structure at textbook/modules/07-manipulation/
- [ ] T028 [P] [US1] Create Module 08 directory structure at textbook/modules/08-locomotion/
- [ ] T029 [P] [US1] Create Module 09 directory structure at textbook/modules/09-ros2-integration/
- [ ] T030 [P] [US1] Create Module 10 directory structure at textbook/modules/10-simulation-to-real/
- [ ] T031 [P] [US1] Write Module 05 theory content in textbook/modules/05-dynamics-control/theory.md (PID, computed torque, adaptive control)
- [ ] T032 [P] [US1] Write Module 06 theory content in textbook/modules/06-motion-planning/theory.md (RRT, A*, trajectory optimization)
- [ ] T033 [P] [US1] Write Module 07 theory content in textbook/modules/07-manipulation/theory.md (Grasping, force control, dexterous manipulation)
- [ ] T034 [P] [US1] Write Module 08 theory content in textbook/modules/08-locomotion/theory.md (ZMP, whole-body control, bipedal walking)
- [ ] T035 [P] [US1] Write Module 09 theory content in textbook/modules/09-ros2-integration/theory.md (ROS2 architecture, nodes, topics, services)
- [ ] T036 [P] [US1] Write Module 10 theory content in textbook/modules/10-simulation-to-real/theory.md (Sim-to-real transfer, domain randomization)

**Advanced Modules (Weeks 11-14)**

- [ ] T037 [P] [US1] Create Module 11 directory structure at textbook/modules/11-learning-based-control/
- [ ] T038 [P] [US1] Create Module 12 directory structure at textbook/modules/12-human-robot-interaction/
- [ ] T039 [P] [US1] Create Module 13 directory structure at textbook/modules/13-full-body-autonomy/
- [ ] T040 [P] [US1] Create Module 14 directory structure at textbook/modules/14-capstone-integration/
- [ ] T041 [P] [US1] Write Module 11 theory content in textbook/modules/11-learning-based-control/theory.md (RL, imitation learning, policy optimization)
- [ ] T042 [P] [US1] Write Module 12 theory content in textbook/modules/12-human-robot-interaction/theory.md (Safety, collaboration, natural interfaces)
- [ ] T043 [P] [US1] Write Module 13 theory content in textbook/modules/13-full-body-autonomy/theory.md (Integrated perception-planning-control, decision making)
- [ ] T044 [P] [US1] Write Module 14 theory content in textbook/modules/14-capstone-integration/theory.md (System integration, project design, deployment)

**Module Metadata and Cross-References**

- [ ] T045 [US1] Add frontmatter to all module theory.md files with prerequisites, learning objectives, difficulty, week number
- [ ] T046 [US1] Validate module prerequisites are acyclic and logically consistent across all 14 modules
- [ ] T047 [US1] Create module progression diagram in textbook/appendices/module-progression.md

**Checkpoint**: At this point, User Story 1 should be fully functional - all 14 modules exist with complete theory content and proper progression structure

---

## Phase 4: User Story 2 - Integrated Learning Components (Priority: P2)

**Goal**: Add hands-on labs, simulations, ethics sections, and assessments to all modules for comprehensive integrated learning

**Independent Test**: Can verify by checking any single module (e.g., Module 05) contains theory, 3-5 labs, simulation configs, ethics section, and assessment package

### Labs for All Modules (3-5 per module)

**Beginner Module Labs**

- [ ] T048 [P] [US2] Create Lab 01-01 (guided, simulation) at textbook/modules/01-introduction-physical-ai/labs/lab-01-01.md (Setup MuJoCo environment)
- [ ] T049 [P] [US2] Create Lab 01-02 (guided, simulation) at textbook/modules/01-introduction-physical-ai/labs/lab-01-02.md (Explore humanoid model)
- [ ] T050 [P] [US2] Create Lab 01-03 (intermediate, simulation) at textbook/modules/01-introduction-physical-ai/labs/lab-01-03.md (Basic actuation)
- [ ] T051 [P] [US2] Create Lab 02-01 (guided, simulation) at textbook/modules/02-rigid-body-dynamics/labs/lab-02-01.md (Compute rigid body forces)
- [ ] T052 [P] [US2] Create Lab 02-02 (intermediate, simulation) at textbook/modules/02-rigid-body-dynamics/labs/lab-02-02.md (Simulate free fall)
- [ ] T053 [P] [US2] Create Lab 02-03 (intermediate, simulation) at textbook/modules/02-rigid-body-dynamics/labs/lab-02-03.md (Torque-motion relationship)
- [ ] T054 [P] [US2] Create Lab 03-01 (guided, simulation) at textbook/modules/03-kinematics-fundamentals/labs/lab-03-01.md (Forward kinematics 2-DOF arm)
- [ ] T055 [P] [US2] Create Lab 03-02 (intermediate, simulation) at textbook/modules/03-kinematics-fundamentals/labs/lab-03-02.md (Inverse kinematics solver)
- [ ] T056 [P] [US2] Create Lab 03-03 (challenge, simulation) at textbook/modules/03-kinematics-fundamentals/labs/lab-03-03.md (Full humanoid FK/IK)
- [ ] T057 [P] [US2] Create Lab 04-01 (guided, simulation) at textbook/modules/04-sensors-perception/labs/lab-04-01.md (Read IMU data)
- [ ] T058 [P] [US2] Create Lab 04-02 (intermediate, simulation) at textbook/modules/04-sensors-perception/labs/lab-04-02.md (Camera image processing)
- [ ] T059 [P] [US2] Create Lab 04-03 (intermediate, simulation) at textbook/modules/04-sensors-perception/labs/lab-04-03.md (Sensor fusion Kalman filter)

**Intermediate Module Labs**

- [ ] T060 [P] [US2] Create Lab 05-01 (guided, simulation) at textbook/modules/05-dynamics-control/labs/lab-05-01.md (Implement PID controller)
- [ ] T061 [P] [US2] Create Lab 05-02 (intermediate, simulation) at textbook/modules/05-dynamics-control/labs/lab-05-02.md (Computed torque control)
- [ ] T062 [P] [US2] Create Lab 05-03 (challenge, simulation) at textbook/modules/05-dynamics-control/labs/lab-05-03.md (Adaptive control for perturbations)
- [ ] T063 [P] [US2] Create Lab 06-01 (guided, simulation) at textbook/modules/06-motion-planning/labs/lab-06-01.md (RRT path planning)
- [ ] T064 [P] [US2] Create Lab 06-02 (intermediate, simulation) at textbook/modules/06-motion-planning/labs/lab-06-02.md (A* with heuristics)
- [ ] T065 [P] [US2] Create Lab 06-03 (challenge, simulation) at textbook/modules/06-motion-planning/labs/lab-06-03.md (Trajectory optimization)
- [ ] T066 [P] [US2] Create Lab 07-01 (guided, simulation) at textbook/modules/07-manipulation/labs/lab-07-01.md (Basic grasping)
- [ ] T067 [P] [US2] Create Lab 07-02 (intermediate, simulation) at textbook/modules/07-manipulation/labs/lab-07-02.md (Force control)
- [ ] T068 [P] [US2] Create Lab 07-03 (challenge, simulation) at textbook/modules/07-manipulation/labs/lab-07-03.md (Dexterous multi-finger grasp)
- [ ] T069 [P] [US2] Create Lab 08-01 (guided, simulation) at textbook/modules/08-locomotion/labs/lab-08-01.md (ZMP calculation)
- [ ] T070 [P] [US2] Create Lab 08-02 (intermediate, simulation) at textbook/modules/08-locomotion/labs/lab-08-02.md (Bipedal walking controller)
- [ ] T071 [P] [US2] Create Lab 08-03 (challenge, simulation) at textbook/modules/08-locomotion/labs/lab-08-03.md (Whole-body balance)
- [ ] T072 [P] [US2] Create Lab 09-01 (guided, simulation) at textbook/modules/09-ros2-integration/labs/lab-09-01.md (ROS2 nodes and topics)
- [ ] T073 [P] [US2] Create Lab 09-02 (intermediate, simulation) at textbook/modules/09-ros2-integration/labs/lab-09-02.md (Service communication)
- [ ] T074 [P] [US2] Create Lab 09-03 (intermediate, simulation) at textbook/modules/09-ros2-integration/labs/lab-09-03.md (Robot state publisher)
- [ ] T075 [P] [US2] Create Lab 10-01 (intermediate, simulation) at textbook/modules/10-simulation-to-real/labs/lab-10-01.md (Domain randomization)
- [ ] T076 [P] [US2] Create Lab 10-02 (intermediate, low_cost_hardware) at textbook/modules/10-simulation-to-real/labs/lab-10-02.md (Deploy to TurtleBot 4)
- [ ] T077 [P] [US2] Create Lab 10-03 (challenge, low_cost_hardware) at textbook/modules/10-simulation-to-real/labs/lab-10-03.md (Sim-to-real transfer validation)

**Advanced Module Labs**

- [ ] T078 [P] [US2] Create Lab 11-01 (intermediate, simulation) at textbook/modules/11-learning-based-control/labs/lab-11-01.md (RL policy training)
- [ ] T079 [P] [US2] Create Lab 11-02 (challenge, simulation) at textbook/modules/11-learning-based-control/labs/lab-11-02.md (Imitation learning from demos)
- [ ] T080 [P] [US2] Create Lab 11-03 (challenge, simulation) at textbook/modules/11-learning-based-control/labs/lab-11-03.md (Policy optimization PPO)
- [ ] T081 [P] [US2] Create Lab 12-01 (guided, simulation) at textbook/modules/12-human-robot-interaction/labs/lab-12-01.md (Safety zones and constraints)
- [ ] T082 [P] [US2] Create Lab 12-02 (intermediate, simulation) at textbook/modules/12-human-robot-interaction/labs/lab-12-02.md (Collaborative task planning)
- [ ] T083 [P] [US2] Create Lab 12-03 (challenge, advanced_hardware) at textbook/modules/12-human-robot-interaction/labs/lab-12-03.md (Natural language interface)
- [ ] T084 [P] [US2] Create Lab 13-01 (challenge, simulation) at textbook/modules/13-full-body-autonomy/labs/lab-13-01.md (Integrated perception-planning-control)
- [ ] T085 [P] [US2] Create Lab 13-02 (challenge, simulation) at textbook/modules/13-full-body-autonomy/labs/lab-13-02.md (Decision making under uncertainty)
- [ ] T086 [P] [US2] Create Lab 13-03 (challenge, advanced_hardware) at textbook/modules/13-full-body-autonomy/labs/lab-13-03.md (Full autonomy demo)
- [ ] T087 [P] [US2] Create Lab 14-01 (challenge, simulation) at textbook/modules/14-capstone-integration/labs/lab-14-01.md (Design capstone project)
- [ ] T088 [P] [US2] Create Lab 14-02 (challenge, simulation) at textbook/modules/14-capstone-integration/labs/lab-14-02.md (Implement and test system)
- [ ] T089 [P] [US2] Create Lab 14-03 (challenge, advanced_hardware) at textbook/modules/14-capstone-integration/labs/lab-14-03.md (Deploy and demonstrate)

### Simulation Configurations

- [ ] T090 [P] [US2] Create MuJoCo simulation config for Module 01 at textbook/modules/01-introduction-physical-ai/simulations/mujoco-config.yaml
- [ ] T091 [P] [US2] Create MuJoCo simulation config for Module 02 at textbook/modules/02-rigid-body-dynamics/simulations/mujoco-config.yaml
- [ ] T092 [P] [US2] Create MuJoCo simulation config for Module 03 at textbook/modules/03-kinematics-fundamentals/simulations/mujoco-config.yaml
- [ ] T093 [P] [US2] Create MuJoCo simulation config for Module 04 at textbook/modules/04-sensors-perception/simulations/mujoco-config.yaml
- [ ] T094 [P] [US2] Create MuJoCo simulation config for Module 05 at textbook/modules/05-dynamics-control/simulations/mujoco-config.yaml
- [ ] T095 [P] [US2] Create MuJoCo simulation config for Module 06 at textbook/modules/06-motion-planning/simulations/mujoco-config.yaml
- [ ] T096 [P] [US2] Create MuJoCo simulation config for Module 07 at textbook/modules/07-manipulation/simulations/mujoco-config.yaml
- [ ] T097 [P] [US2] Create MuJoCo simulation config for Module 08 at textbook/modules/08-locomotion/simulations/mujoco-config.yaml
- [ ] T098 [P] [US2] Create Gazebo simulation config for Module 09 at textbook/modules/09-ros2-integration/simulations/gazebo-config.yaml
- [ ] T099 [P] [US2] Create Gazebo simulation config for Module 10 at textbook/modules/10-simulation-to-real/simulations/gazebo-config.yaml
- [ ] T100 [P] [US2] Create MuJoCo simulation config for Module 11 at textbook/modules/11-learning-based-control/simulations/mujoco-config.yaml
- [ ] T101 [P] [US2] Create Gazebo simulation config for Module 12 at textbook/modules/12-human-robot-interaction/simulations/gazebo-config.yaml
- [ ] T102 [P] [US2] Create MuJoCo simulation config for Module 13 at textbook/modules/13-full-body-autonomy/simulations/mujoco-config.yaml
- [ ] T103 [P] [US2] Create Gazebo simulation config for Module 14 at textbook/modules/14-capstone-integration/simulations/gazebo-config.yaml

### Ethics Sections

- [ ] T104 [P] [US2] Write ethics content for Module 01 at textbook/modules/01-introduction-physical-ai/ethics.md (AI ethics foundations, responsible development)
- [ ] T105 [P] [US2] Write ethics content for Module 02 at textbook/modules/02-rigid-body-dynamics/ethics.md (Safety margins, physical harm prevention)
- [ ] T106 [P] [US2] Write ethics content for Module 03 at textbook/modules/03-kinematics-fundamentals/ethics.md (Workspace safety, collision avoidance)
- [ ] T107 [P] [US2] Write ethics content for Module 04 at textbook/modules/04-sensors-perception/ethics.md (Privacy, surveillance, data ethics)
- [ ] T108 [P] [US2] Write ethics content for Module 05 at textbook/modules/05-dynamics-control/ethics.md (Control failure responsibility, conservative tuning)
- [ ] T109 [P] [US2] Write ethics content for Module 06 at textbook/modules/06-motion-planning/ethics.md (Predictable behavior, transparency)
- [ ] T110 [P] [US2] Write ethics content for Module 07 at textbook/modules/07-manipulation/ethics.md (Force limits, safe interaction)
- [ ] T111 [P] [US2] Write ethics content for Module 08 at textbook/modules/08-locomotion/ethics.md (Public space navigation, accessibility)
- [ ] T112 [P] [US2] Write ethics content for Module 09 at textbook/modules/09-ros2-integration/ethics.md (Software quality, testing standards)
- [ ] T113 [P] [US2] Write ethics content for Module 10 at textbook/modules/10-simulation-to-real/ethics.md (Validation requirements, deployment readiness)
- [ ] T114 [P] [US2] Write ethics content for Module 11 at textbook/modules/11-learning-based-control/ethics.md (Black box systems, interpretability, alignment)
- [ ] T115 [P] [US2] Write ethics content for Module 12 at textbook/modules/12-human-robot-interaction/ethics.md (Autonomy, dignity, informed consent)
- [ ] T116 [P] [US2] Write ethics content for Module 13 at textbook/modules/13-full-body-autonomy/ethics.md (Autonomous decision ethics, accountability)
- [ ] T117 [P] [US2] Write ethics content for Module 14 at textbook/modules/14-capstone-integration/ethics.md (Professional ethics, societal impact)

### Assessment Packages

- [ ] T118 [P] [US2] Create assessment package for Module 01 at textbook/modules/01-introduction-physical-ai/assessment.md (quiz, rubrics, project)
- [ ] T119 [P] [US2] Create assessment package for Module 02 at textbook/modules/02-rigid-body-dynamics/assessment.md
- [ ] T120 [P] [US2] Create assessment package for Module 03 at textbook/modules/03-kinematics-fundamentals/assessment.md
- [ ] T121 [P] [US2] Create assessment package for Module 04 at textbook/modules/04-sensors-perception/assessment.md
- [ ] T122 [P] [US2] Create assessment package for Module 05 at textbook/modules/05-dynamics-control/assessment.md
- [ ] T123 [P] [US2] Create assessment package for Module 06 at textbook/modules/06-motion-planning/assessment.md
- [ ] T124 [P] [US2] Create assessment package for Module 07 at textbook/modules/07-manipulation/assessment.md
- [ ] T125 [P] [US2] Create assessment package for Module 08 at textbook/modules/08-locomotion/assessment.md
- [ ] T126 [P] [US2] Create assessment package for Module 09 at textbook/modules/09-ros2-integration/assessment.md
- [ ] T127 [P] [US2] Create assessment package for Module 10 at textbook/modules/10-simulation-to-real/assessment.md
- [ ] T128 [P] [US2] Create assessment package for Module 11 at textbook/modules/11-learning-based-control/assessment.md
- [ ] T129 [P] [US2] Create assessment package for Module 12 at textbook/modules/12-human-robot-interaction/assessment.md
- [ ] T130 [P] [US2] Create assessment package for Module 13 at textbook/modules/13-full-body-autonomy/assessment.md
- [ ] T131 [P] [US2] Create assessment package for Module 14 at textbook/modules/14-capstone-integration/assessment.md

### Code Examples with Tests

- [ ] T132 [P] [US2] Create Python code examples for Module 01 in textbook/assets/code-examples/01/ (basic MuJoCo usage)
- [ ] T133 [P] [US2] Create Python code examples for Module 02 in textbook/assets/code-examples/02/ (dynamics calculations)
- [ ] T134 [P] [US2] Create Python code examples for Module 03 in textbook/assets/code-examples/03/ (FK/IK solvers)
- [ ] T135 [P] [US2] Create Python code examples for Module 04 in textbook/assets/code-examples/04/ (sensor processing)
- [ ] T136 [P] [US2] Create Python code examples for Module 05 in textbook/assets/code-examples/05/ (PID, computed torque)
- [ ] T137 [P] [US2] Create Python code examples for Module 06 in textbook/assets/code-examples/06/ (RRT, A* planners)
- [ ] T138 [P] [US2] Create Python code examples for Module 07 in textbook/assets/code-examples/07/ (grasping, force control)
- [ ] T139 [P] [US2] Create Python code examples for Module 08 in textbook/assets/code-examples/08/ (ZMP, walking)
- [ ] T140 [P] [US2] Create Python code examples for Module 09 in textbook/assets/code-examples/09/ (ROS2 nodes)
- [ ] T141 [P] [US2] Create Python code examples for Module 10 in textbook/assets/code-examples/10/ (sim-to-real transfer)
- [ ] T142 [P] [US2] Create Python code examples for Module 11 in textbook/assets/code-examples/11/ (RL policies)
- [ ] T143 [P] [US2] Create Python code examples for Module 12 in textbook/assets/code-examples/12/ (HRI interfaces)
- [ ] T144 [P] [US2] Create Python code examples for Module 13 in textbook/assets/code-examples/13/ (integrated systems)
- [ ] T145 [P] [US2] Create pytest test file at tests/code-examples/test_01_intro.py
- [ ] T146 [P] [US2] Create pytest test file at tests/code-examples/test_02_dynamics.py
- [ ] T147 [P] [US2] Create pytest test file at tests/code-examples/test_03_kinematics.py
- [ ] T148 [P] [US2] Create pytest test file at tests/code-examples/test_04_sensors.py
- [ ] T149 [P] [US2] Create pytest test file at tests/code-examples/test_05_control.py
- [ ] T150 [P] [US2] Create pytest test file at tests/code-examples/test_06_planning.py
- [ ] T151 [P] [US2] Create pytest test file at tests/code-examples/test_07_manipulation.py
- [ ] T152 [P] [US2] Create pytest test file at tests/code-examples/test_08_locomotion.py
- [ ] T153 [P] [US2] Create pytest test file at tests/code-examples/test_09_ros2.py
- [ ] T154 [P] [US2] Create pytest test file at tests/code-examples/test_10_sim2real.py
- [ ] T155 [P] [US2] Create pytest test file at tests/code-examples/test_11_learning.py
- [ ] T156 [P] [US2] Create pytest test file at tests/code-examples/test_12_hri.py
- [ ] T157 [P] [US2] Create pytest test file at tests/code-examples/test_13_autonomy.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - modules have complete integrated learning components

---

## Phase 5: User Story 3 - Real-world Applications Context (Priority: P3)

**Goal**: Add current industry case studies (2023-2025) representing diverse application domains to provide real-world context

**Independent Test**: Can verify by reviewing case studies directory to confirm presence of recent examples from manufacturing, healthcare, disaster response, research, and consumer domains with proper distribution

### Case Study Creation by Domain

**Manufacturing & Logistics (35% - 5 case studies)**

- [ ] T158 [P] [US3] Create case study at textbook/case-studies/manufacturing/2024-figure-bmw.md (Figure AI + BMW Spartanburg humanoid deployment)
- [ ] T159 [P] [US3] Create case study at textbook/case-studies/manufacturing/2024-tesla-optimus.md (Tesla Optimus factory deployment)
- [ ] T160 [P] [US3] Create case study at textbook/case-studies/manufacturing/2024-amazon-digit.md (Amazon + Agility Digit fulfillment centers)
- [ ] T161 [P] [US3] Create case study at textbook/case-studies/manufacturing/2024-apptronik-mercedes.md (Apptronik Apollo + Mercedes-Benz)
- [ ] T162 [P] [US3] Create case study at textbook/case-studies/manufacturing/2023-warehouse-automation.md (Warehouse robotics trends)

**Healthcare & Rehabilitation (25% - 3 case studies)**

- [ ] T163 [P] [US3] Create case study at textbook/case-studies/healthcare/2024-diligent-moxi.md (Diligent Robotics Moxi hospital logistics)
- [ ] T164 [P] [US3] Create case study at textbook/case-studies/healthcare/2023-toyota-hsr.md (Toyota HSR elder care research)
- [ ] T165 [P] [US3] Create case study at textbook/case-studies/healthcare/2023-exoskeleton-rehab.md (ReWalk/Ekso rehabilitation)

**Research & Academia (20% - 3 case studies)**

- [ ] T166 [P] [US3] Create case study at textbook/case-studies/research/2024-mit-humanoid-lab.md (MIT dynamic locomotion research)
- [ ] T167 [P] [US3] Create case study at textbook/case-studies/research/2024-berkeley-bair.md (UC Berkeley manipulation and learning)
- [ ] T168 [P] [US3] Create case study at textbook/case-studies/research/2023-stanford-iris.md (Stanford HRI research)

**Disaster Response & Hazardous (12% - 2 case studies)**

- [ ] T169 [P] [US3] Create case study at textbook/case-studies/disaster/2024-boston-dynamics-atlas.md (Boston Dynamics Atlas DARPA/construction)
- [ ] T170 [P] [US3] Create case study at textbook/case-studies/disaster/2023-anymal-inspection.md (ANYbotics ANYmal hazardous inspection)

**Consumer & Service (8% - 1 case study)**

- [ ] T171 [P] [US3] Create case study at textbook/case-studies/consumer/2025-1x-neo.md (1X EVE/NEO home assistant development)

### Case Study Integration

- [ ] T172 [US3] Link case studies to related modules by adding references in module theory.md files
- [ ] T173 [US3] Create case study index at textbook/case-studies/index.md with domain distribution summary
- [ ] T174 [US3] Add case study discussion questions to relevant module ethics sections
- [ ] T175 [US3] Create case study validation script at tests/content/validate-case-studies.py (checks year, domain distribution, module links)

**Checkpoint**: All user stories should now be independently functional - complete textbook with structure, integrated components, and real-world context

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T176 [P] Create appendices at textbook/appendices/hardware-requirements.md (detailed hardware tier descriptions)
- [ ] T177 [P] Create appendices at textbook/appendices/software-setup.md (installation guides for MuJoCo, ROS2, Docker)
- [ ] T178 [P] Create simulation guides at textbook/appendices/simulation-guides/mujoco-guide.md
- [ ] T179 [P] Create simulation guides at textbook/appendices/simulation-guides/gazebo-guide.md
- [ ] T180 [P] Create simulation guides at textbook/appendices/simulation-guides/docker-setup.md
- [ ] T181 [P] Create glossary at textbook/appendices/glossary.md (key terms from all modules)
- [ ] T182 [P] Create bibliography at textbook/appendices/bibliography.md (academic references)
- [ ] T183 [P] Create index of learning objectives at textbook/appendices/learning-objectives-index.md
- [ ] T184 [P] Generate asset diagrams for module progression at textbook/assets/diagrams/module-progression.svg
- [ ] T185 [P] Generate asset diagrams for prerequisite graph at textbook/assets/diagrams/prerequisites-graph.svg
- [ ] T186 Create build script for PDF generation at scripts/build-pdf.py
- [ ] T187 [P] Create build script for HTML generation at scripts/build-html.py
- [ ] T188 [P] Create build script for EPUB generation at scripts/build-epub.py
- [ ] T189 Run complete content validation across all modules using scripts/validate-all.sh
- [ ] T190 Run all code example tests using pytest tests/code-examples/
- [ ] T191 Generate preview builds for all modules to verify rendering
- [ ] T192 Review and validate quickstart.md against actual project structure

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - Requires US1 module structure but can work in parallel with US1 content writing
  - User Story 3 (P3): Can start after Foundational - Independent of US1/US2 (case studies are separate)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Creates module structure and theory content - foundational for US2
- **User Story 2 (P2)**: Adds labs/simulations/ethics/assessments - requires module directories from US1 but can start once directories exist
- **User Story 3 (P3)**: Adds case studies - completely independent, can run in parallel with US1/US2

### Within Each User Story

**US1 (Module Structure)**:
- Module directory creation tasks can all run in parallel [P]
- Theory writing tasks can all run in parallel [P]
- Frontmatter addition must come after theory writing
- Validation comes last

**US2 (Integrated Components)**:
- All lab creation tasks marked [P] within each section
- All simulation config tasks marked [P]
- All ethics content tasks marked [P]
- All assessment tasks marked [P]
- All code example tasks marked [P]
- All test file tasks marked [P]

**US3 (Case Studies)**:
- All case study creation tasks marked [P]
- Integration tasks come after case study creation

### Parallel Opportunities

- **Phase 1**: Tasks T003-T006 can run in parallel
- **Phase 2**: Tasks T008-T011, T013-T015 can run in parallel
- **Phase 3**: Tasks T017-T020 (module directories) in parallel, T021-T024 (theory) in parallel, and so on
- **Phase 4**: Massive parallelization - all lab creation, simulation configs, ethics, assessments, code examples can proceed simultaneously
- **Phase 5**: All case study creation tasks (T158-T171) can run in parallel
- **Phase 6**: Tasks T176-T183, T187-T188 can run in parallel

---

## Parallel Example: User Story 2 Module 05 Labs

```bash
# Launch all lab creation for Module 05 together:
Task: "Create Lab 05-01 (guided, simulation) at textbook/modules/05-dynamics-control/labs/lab-05-01.md (Implement PID controller)"
Task: "Create Lab 05-02 (intermediate, simulation) at textbook/modules/05-dynamics-control/labs/lab-05-02.md (Computed torque control)"
Task: "Create Lab 05-03 (challenge, simulation) at textbook/modules/05-dynamics-control/labs/lab-05-03.md (Adaptive control for perturbations)"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Module Structure)

1. Complete Phase 1: Setup (6 tasks)
2. Complete Phase 2: Foundational (10 tasks, CRITICAL)
3. Complete Phase 3: User Story 1 (31 tasks - module structure and theory)
4. **STOP and VALIDATE**: Run validation scripts, verify all 14 modules exist with proper progression
5. Review sample module content for quality

**MVP Delivers**: Complete 14-module structure with theory content, proper prerequisites, and logical progression

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (16 tasks)
2. Add User Story 1 → Test independently → Review sample modules (47 tasks total, MVP ready!)
3. Add User Story 2 → Test independently → Review integrated components (157 tasks total, full learning experience)
4. Add User Story 3 → Test independently → Review case studies (175 tasks total, real-world context)
5. Polish phase → Final quality pass (192 tasks total, publication ready)

### Parallel Team Strategy

With multiple developers after Foundational phase completes:

1. **Team A (3-4 people)**: User Story 1 (Module Structure)
   - Person 1: Modules 01-04
   - Person 2: Modules 05-08
   - Person 3: Modules 09-12
   - Person 4: Modules 13-14 + validation

2. **Team B (4-5 people)**: User Story 2 (Integrated Components) - can start once directories exist
   - Person 1: Labs for Modules 01-03
   - Person 2: Labs for Modules 04-07
   - Person 3: Labs for Modules 08-11
   - Person 4: Labs for Modules 12-14
   - Person 5: Simulations, ethics, assessments across all modules

3. **Team C (1-2 people)**: User Story 3 (Case Studies) - fully independent
   - Person 1: Manufacturing, healthcare, research cases
   - Person 2: Disaster, consumer cases + integration

---

## Summary Statistics

- **Total Tasks**: 192
- **Tasks per User Story**:
  - US1 (P1 - Course Structure): 31 tasks (T017-T047)
  - US2 (P2 - Integrated Components): 110 tasks (T048-T157)
  - US3 (P3 - Real-world Applications): 18 tasks (T158-T175)
- **Setup + Foundational**: 16 tasks (T001-T016)
- **Polish**: 17 tasks (T176-T192)
- **Parallel Tasks**: 160+ tasks marked [P] (83% parallelizable)
- **MVP Scope**: Setup + Foundational + US1 = 47 tasks

**Independent Test Criteria**:
- **US1**: All 14 modules exist with theory.md, proper frontmatter, acyclic prerequisites, 3-5 learning objectives each
- **US2**: Any sample module (e.g., Module 05) contains 3-5 labs, simulation config, ethics.md, assessment.md, working code examples with tests
- **US3**: Case studies directory contains 14 recent case studies (2023-2025), proper domain distribution (35% manufacturing, 25% healthcare, 20% research, 12% disaster, 8% consumer), all linked to relevant modules

---

## Notes

- [P] tasks = different files, no dependencies (can execute in parallel)
- [US1], [US2], [US3] labels map tasks to specific user stories for traceability
- Each user story is independently completable and testable
- Tests are NOT included (not requested in specification)
- This is a content creation project - no application code, only educational materials
- Validation scripts ensure content quality and consistency
- Commit after completing each module or logical group of tasks
- Stop at any checkpoint to validate story independently
