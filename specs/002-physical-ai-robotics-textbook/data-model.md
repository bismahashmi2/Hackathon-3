# Data Model: Physical AI and Humanoid Robotics Textbook

**Feature Branch**: `002-physical-ai-robotics-textbook`
**Date**: 2025-12-21
**Status**: Complete

## Overview

This document defines the content structure entities, their relationships, and validation rules for the Physical AI and Humanoid Robotics textbook.

---

## Core Entities

### 1. Module

The primary organizational unit representing one week of instruction.

```yaml
Module:
  id: string           # Format: "MM" (01-14)
  title: string        # Human-readable title
  slug: string         # URL-safe identifier
  week: integer        # 1-14
  difficulty: enum     # beginner | intermediate | advanced
  prerequisites: [ModuleRef]  # List of required prior modules
  learning_objectives: [string]  # 3-5 measurable outcomes
  estimated_hours: integer      # Total student time including labs

  # Nested components
  theory: TheorySection
  labs: [LabExercise]          # 3-5 per module
  simulations: [SimulationConfig]
  ethics: EthicsSection
  assessment: AssessmentPackage

  # Metadata
  last_updated: date
  version: semver
  status: enum         # draft | review | published
```

**Validation Rules**:
- `id` must be unique across all modules
- `prerequisites` cannot create circular dependencies
- `learning_objectives` must contain 3-5 items
- Each module must have at least 3 labs

---

### 2. TheorySection

Conceptual content within a module.

```yaml
TheorySection:
  sections: [ContentBlock]
  key_concepts: [Concept]
  equations: [Equation]        # LaTeX format
  figures: [FigureRef]
  code_examples: [CodeExample]
  reading_time_minutes: integer

ContentBlock:
  id: string
  type: enum           # text | definition | theorem | example | warning
  title: string?
  content: markdown

Concept:
  term: string
  definition: markdown
  related_terms: [string]
  first_introduced_in: ModuleRef
```

**Validation Rules**:
- Each `TheorySection` must define at least 3 `key_concepts`
- All `Equation` references must have LaTeX that compiles
- `figures` must reference existing files in `assets/`

---

### 3. LabExercise

Hands-on practical exercise.

```yaml
LabExercise:
  id: string           # Format: "lab-MM-NN"
  module_id: ModuleRef
  title: string
  difficulty: enum     # guided | intermediate | challenge
  tier: enum           # simulation | low_cost_hardware | advanced_hardware
  duration_minutes: integer

  # Content
  objectives: [string]
  prerequisites: [LabRef | ModuleRef]
  materials: [MaterialRequirement]
  instructions: [InstructionStep]
  expected_outcomes: [Outcome]

  # Assessment
  rubric: GradingRubric
  submission_format: enum  # notebook | code | report | demo
  auto_gradable: boolean

  # Support
  hints: [Hint]
  common_errors: [ErrorPattern]
  extensions: [ExtensionTask]  # For advanced students

MaterialRequirement:
  type: enum           # software | hardware | data | simulation
  name: string
  version: string?
  tier: enum           # required | recommended | optional
  alternatives: [string]

InstructionStep:
  step_number: integer
  title: string
  content: markdown
  code_snippet: CodeExample?
  checkpoint: Checkpoint?    # Verification point

Checkpoint:
  description: string
  expected_result: string
  troubleshooting: [TroubleshootingTip]
```

**Validation Rules**:
- `duration_minutes` must be between 30 and 180
- Each lab must have at least one `Checkpoint`
- `tier: simulation` labs must not require physical hardware
- All labs must have a `rubric`

---

### 4. SimulationConfig

Configuration for simulation-based exercises.

```yaml
SimulationConfig:
  id: string
  platform: enum       # mujoco | gazebo | isaac_sim
  robot_model: string  # URDF/MJCF reference
  environment: string  # Scene configuration

  # Parameters
  physics_settings: PhysicsParams
  visualization: VisualizationParams
  recording: RecordingParams?

  # Entry points
  launch_script: string        # Path to launch file
  notebook: string?            # Jupyter notebook path
  docker_image: string?        # Pre-configured container

PhysicsParams:
  timestep: float              # Simulation timestep in seconds
  gravity: [float, float, float]
  solver_iterations: integer
  contact_model: string

VisualizationParams:
  camera_position: [float, float, float]
  render_mode: enum    # headless | window | offscreen
  record_video: boolean
```

**Validation Rules**:
- `platform` must be one of the supported platforms from research.md
- `robot_model` file must exist in the assets directory
- `timestep` must be between 0.0001 and 0.01

---

### 5. EthicsSection

Ethics content integrated within each module.

```yaml
EthicsSection:
  primer: markdown           # Opening context (100-200 words)
  callouts: [EthicsCallout]  # Inline during technical content
  reflection: EthicsReflection
  case_study: CaseStudy?

EthicsCallout:
  id: string
  trigger_concept: string    # Technical concept that triggers callout
  content: markdown
  type: enum                 # consideration | principle | question

EthicsReflection:
  discussion_questions: [string]  # 3-5 questions
  stakeholders: [string]          # Affected parties to consider
  frameworks: [string]            # Applicable ethical frameworks

CaseStudy:
  title: string
  domain: enum               # healthcare | manufacturing | etc.
  scenario: markdown
  dilemma: string
  perspectives: [Perspective]
  discussion_guide: markdown

Perspective:
  stakeholder: string
  position: markdown
  concerns: [string]
```

**Validation Rules**:
- Each module must have at least one `EthicsCallout`
- `reflection.discussion_questions` must have 3-5 items
- Case studies must include at least 2 different perspectives

---

### 6. AssessmentPackage

Complete assessment materials for a module.

```yaml
AssessmentPackage:
  module_id: ModuleRef

  # Components (weights must sum to 100)
  theory_quiz: Quiz          # 15%
  lab_assessment: [LabGrade] # 35%
  simulation_project: Project # 35%
  ethics_component: EthicsAssessment # 15%

  # Export formats
  exports: [ExportFormat]

Quiz:
  questions: [QuizQuestion]
  time_limit_minutes: integer
  passing_score: float       # Percentage (0-100)
  attempts_allowed: integer

QuizQuestion:
  id: string
  type: enum                 # multiple_choice | short_answer | code
  question: markdown
  options: [string]?         # For multiple choice
  correct_answer: string | [string]
  points: integer
  explanation: markdown      # Shown after submission
  learning_objective: string # Maps to module objective

Project:
  title: string
  description: markdown
  deliverables: [Deliverable]
  rubric: GradingRubric
  due_week: integer          # Relative to module start

ExportFormat:
  format: enum               # qti | canvas | blackboard | moodle | pdf
  file_path: string
```

**Validation Rules**:
- Assessment weights must sum to 100%
- Each `QuizQuestion` must map to a `learning_objective`
- `passing_score` must be between 60 and 100

---

### 7. CaseStudyEntry

Industry case studies for real-world context.

```yaml
CaseStudyEntry:
  id: string
  title: string
  domain: enum               # manufacturing | healthcare | disaster | research | consumer
  company: string
  robot_platform: string
  year: integer              # 2023-2025

  # Content
  summary: markdown          # 200-300 words
  technical_details: markdown
  challenges: [Challenge]
  outcomes: [Outcome]
  lessons_learned: [string]

  # Links
  related_modules: [ModuleRef]
  external_references: [URL]

  # Metadata
  last_verified: date
  status: enum               # current | dated | archived

Challenge:
  category: enum             # technical | ethical | regulatory | economic
  description: markdown
  resolution: markdown?
```

**Validation Rules**:
- `year` must be between 2023 and current year
- Each case study must link to at least one module
- `domain` distribution should match research.md percentages

---

## Entity Relationships

```
┌─────────────┐
│   Module    │
└──────┬──────┘
       │
       ├──────────────┬──────────────┬──────────────┬──────────────┐
       │              │              │              │              │
       ▼              ▼              ▼              ▼              ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│   Theory    │ │    Labs     │ │ Simulations │ │   Ethics    │ │ Assessment  │
│   Section   │ │  [3-5 per]  │ │  Configs    │ │  Section    │ │  Package    │
└─────────────┘ └──────┬──────┘ └──────┬──────┘ └──────┬──────┘ └─────────────┘
                       │              │              │
                       │              │              │
                       ▼              │              ▼
                ┌─────────────┐       │       ┌─────────────┐
                │  Material   │       │       │ Case Study  │◄────────────────┐
                │Requirements │       │       │   Entry     │                 │
                └─────────────┘       │       └─────────────┘                 │
                                      │                                       │
                                      ▼                                       │
                               ┌─────────────┐                                │
                               │   Robot     │                                │
                               │   Models    │────────────────────────────────┘
                               └─────────────┘
```

---

## Module Progression Map

```yaml
# Difficulty and dependency progression across 14 modules

beginner_modules:      # Weeks 1-4
  - 01-introduction-physical-ai:
      prerequisites: []
      difficulty: beginner
  - 02-rigid-body-dynamics:
      prerequisites: [01]
      difficulty: beginner
  - 03-kinematics-fundamentals:
      prerequisites: [02]
      difficulty: beginner
  - 04-sensors-perception:
      prerequisites: [01]
      difficulty: beginner

intermediate_modules:  # Weeks 5-10
  - 05-dynamics-control:
      prerequisites: [02, 03]
      difficulty: intermediate
  - 06-motion-planning:
      prerequisites: [03, 04]
      difficulty: intermediate
  - 07-manipulation:
      prerequisites: [05, 06]
      difficulty: intermediate
  - 08-locomotion:
      prerequisites: [05]
      difficulty: intermediate
  - 09-ros2-integration:
      prerequisites: [04, 06]
      difficulty: intermediate
  - 10-simulation-to-real:
      prerequisites: [07, 08, 09]
      difficulty: intermediate

advanced_modules:      # Weeks 11-14
  - 11-learning-based-control:
      prerequisites: [05, 07]
      difficulty: advanced
  - 12-human-robot-interaction:
      prerequisites: [04, 07]
      difficulty: advanced
  - 13-full-body-autonomy:
      prerequisites: [08, 10, 11]
      difficulty: advanced
  - 14-capstone-integration:
      prerequisites: [all]
      difficulty: advanced
```

---

## State Transitions

### Module Status Lifecycle

```
         ┌──────────────────────────────────────┐
         │                                      │
         ▼                                      │
    ┌─────────┐    review     ┌──────────┐     │
    │  draft  │──────────────►│  review  │     │
    └─────────┘               └────┬─────┘     │
         ▲                         │           │
         │                         │ approve   │
         │ revisions               │           │
         │ requested               ▼           │
         │                   ┌───────────┐     │
         └───────────────────│ published │     │
                             └─────┬─────┘     │
                                   │           │
                                   │ update    │
                                   └───────────┘
```

### Case Study Status

```
    ┌──────────┐
    │ current  │ ◄─── new case study
    └────┬─────┘
         │
         │ 18+ months old
         ▼
    ┌──────────┐
    │  dated   │ ◄─── add "Current as of" warning
    └────┬─────┘
         │
         │ no longer relevant
         ▼
    ┌──────────┐
    │ archived │ ◄─── move to archive/, keep for reference
    └──────────┘
```

---

## File Naming Conventions

```yaml
modules:
  directory: "textbook/modules/{id}-{slug}/"
  theory: "theory.md"
  lab: "labs/lab-{id}-{nn}.md"
  simulation: "simulations/{platform}-config.yaml"
  ethics: "ethics.md"
  assessment: "assessment.md"

case_studies:
  directory: "textbook/case-studies/{domain}/"
  file: "{year}-{slug}.md"

assets:
  images: "textbook/assets/images/{module-id}/"
  diagrams: "textbook/assets/diagrams/{module-id}/"
  code: "textbook/assets/code-examples/{module-id}/"
  models: "textbook/assets/robot-models/{platform}/"
```
