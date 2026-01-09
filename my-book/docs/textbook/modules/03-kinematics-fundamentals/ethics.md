---
module_id: "03"
---

# Ethics: Workspace Safety and Collision Avoidance

## Ethics Primer

Kinematics defines where a robot can reach—its workspace. This seemingly abstract mathematical concept has immediate ethical implications: the robot's workspace overlaps with human space. Every point a robot can reach is a point where it could potentially contact, assist, or injure a person. Understanding kinematics is therefore not just about solving equations; it's about mapping the boundaries of robot-human interaction and designing systems that respect human safety within shared spaces.

The inverse kinematics problem—finding how to position joints to reach a target—embodies a fundamental design choice: reaching capability versus safety. A robot with larger workspace can do more but also poses risks to more of the environment.

## Callouts

<ethics-callout id="ec-03-01" type="principle" trigger="workspace calculation">
**Workspace-as-Risk-Zone Principle**: Every reachable point in a robot's workspace is a potential contact point with humans or obstacles. Safety planning must begin with workspace analysis—understanding not just where the robot can reach, but where it might inadvertently reach during motion or failure.
</ethics-callout>

<ethics-callout id="ec-03-02" type="consideration" trigger="inverse kinematics">
**Multiple Solutions, Multiple Risks**: IK typically has multiple solutions (e.g., elbow-up vs. elbow-down). Different solutions traverse different paths through space. Safety-conscious design must consider which solution minimizes risk, not just which is computationally convenient.
</ethics-callout>

<ethics-callout id="ec-03-03" type="question" trigger="singularity">
**Singularity Hazards**: Near kinematic singularities, small Cartesian movements require large joint velocities. This creates unpredictable behavior risks. How should robots be programmed to handle singularity approach—slow down, avoid, or stop?
</ethics-callout>

<ethics-callout id="ec-03-04" type="principle" trigger="collision avoidance">
**Proactive Collision Prevention**: Collision avoidance must be designed in, not added on. The kinematic structure itself should be chosen with collision safety in mind—link lengths, joint limits, and workspace shape all affect how safely the robot can operate near humans.
</ethics-callout>

## Reflection

### Discussion Questions

1. **Workspace Segregation vs. Collaboration**: Traditional industrial safety uses fences to separate robot and human workspaces. Collaborative robots work alongside humans. What kinematic design choices enable safer collaboration? Should workspace overlap be minimized or carefully managed?

2. **IK Solution Selection**: When multiple IK solutions exist, should robots always choose the "safest" solution (e.g., one that keeps elbows away from humans), even if it's less efficient? Who decides what "safe" means in different contexts?

3. **Reach vs. Restriction Trade-offs**: A robot with limited reach is inherently safer but less capable. How should designers balance the desire for capability against the risks of larger workspaces? Should this decision involve end-users?

4. **Transparent Motion**: Humans predict robot motion based on intuition. Some kinematic configurations lead to unintuitive motions (e.g., elbow reversals). Should robots be constrained to only "readable" configurations that humans can predict?

5. **Failure Mode Workspace**: If a joint fails (locks or goes limp), the effective workspace changes unpredictably. How much engineering effort should be dedicated to analyzing failure-mode kinematics?

### Stakeholders to Consider

- **Coworkers in shared spaces**: Factory workers, warehouse employees
- **Patients in healthcare**: Those receiving robotic assistance or surgery
- **Children and pets**: Who may unpredictably enter robot workspaces
- **Maintenance workers**: Who work on robots in unusual configurations
- **Disabled individuals**: Who may have reduced ability to avoid robots

### Applicable Ethical Frameworks

- **Safety Zone Standards**: ISO 13855 (safety distance calculation), ISO 13857 (safety distances to prevent reach)
- **Collaborative Robot Requirements**: ISO 10218-2, ISO/TS 15066 (speed and separation monitoring)
- **Risk Assessment**: ISO 12100 (safety of machinery principles)
- **Defense in Depth**: Multiple independent safety barriers rather than single-point protection
- **Fail-Safe Design**: System should fail to a safe state

## Case Study Connection

See the case study on **Figure AI + BMW Spartanburg** (textbook/case-studies/manufacturing/2024-figure-bmw.md) for insights on how humanoid robots with human-like kinematics are deployed in manufacturing environments where workspace management is critical.

**Discussion**: Figure's humanoid robots have human-proportioned limbs and therefore human-like workspaces. Does this make them inherently safer (predictable) or more dangerous (larger reach) compared to traditional industrial arms? How does their deployment strategy address workspace overlap with human workers?

## Practical Exercise

**Workspace Safety Audit**: For a given robot model (e.g., the humanoid in this module's labs):

1. Compute the maximum reach envelope
2. Identify zones where unexpected motion could occur (near singularities)
3. Propose workspace restrictions for safe human collaboration
4. Design a speed-limiting scheme based on proximity to workspace boundaries

Document your reasoning and the trade-offs involved in each decision.
