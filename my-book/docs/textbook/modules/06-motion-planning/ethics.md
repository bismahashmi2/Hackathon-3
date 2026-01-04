---
module_id: "06"
title: "Ethics in Motion Planning"
---

# Ethics in Motion Planning

## Ethics Primer

Motion planning determines how robots move through the world—which paths they take, how fast they travel, and how they avoid obstacles. These seemingly technical decisions carry ethical weight: a robot's trajectory affects the safety of nearby humans, the predictability of its behavior, and the fairness of shared space allocation.

When a robot plans a path that cuts close to a person to save time, or chooses a route that blocks a hallway, it makes implicit value judgments about whose convenience matters more. Motion planners that prioritize efficiency above all else may create systems that are aggressive, unpredictable, or inequitable. Ethical motion planning requires balancing multiple objectives: task completion, human safety, social norms, and transparent behavior.

## Technical Ethics Callouts

<ethics-callout id="ec-06-01" type="principle" trigger="path optimization">
**Predictable Motion**: Humans need to anticipate robot behavior to safely share space. Optimal paths that surprise people—sudden turns, unexpected accelerations, unconventional routes—create safety hazards even when technically collision-free. Predictability can be more important than optimality.
</ethics-callout>

<ethics-callout id="ec-06-02" type="consideration" trigger="cost functions">
**Implicit Values in Objectives**: Every cost function encodes priorities. Minimizing time values efficiency; minimizing energy values sustainability; minimizing proximity to humans values safety. The choice of objective function is a values decision that should be made explicitly, not buried in code.
</ethics-callout>

<ethics-callout id="ec-06-03" type="question" trigger="collision avoidance">
**Who Yields?**: When a robot and human approach a narrow passage, who should yield? Always making humans accommodate robots imposes burden; always yielding may make robots ineffective. How should responsibility for collision avoidance be distributed between robots and the humans they work alongside?
</ethics-callout>

<ethics-callout id="ec-06-04" type="consideration" trigger="replanning">
**Commitment and Trust**: Frequent replanning optimizes for current conditions but makes robot behavior unpredictable. If a robot commits to a path and then abandons it, humans who planned around the original intention may be disrupted. There's value in following through on communicated intentions.
</ethics-callout>

<ethics-callout id="ec-06-05" type="principle" trigger="safety margins">
**Conservative Under Uncertainty**: When sensor data is uncertain or predictions are unreliable, motion planners should increase safety margins. The appropriate response to uncertainty is caution, not optimistic assumptions.
</ethics-callout>

## Reflection

### Discussion Questions

1. **Efficiency vs. Social Norms**: A robot determines that cutting through a park would save 30 seconds compared to using the sidewalk. The park path is technically legal and collision-free. Should the robot take it? What if it's a busy park with children playing? What social norms should robots respect even when not legally required?

2. **Allocation of Shared Space**: In a warehouse with human workers and mobile robots, paths must be allocated. A purely efficient allocation might give robots priority on main corridors (they're faster and more predictable), relegating humans to secondary routes. Is this fair? Who should decide how shared space is allocated?

3. **Risk Distribution**: A motion planner can choose between a route that passes one person closely (higher individual risk) versus a route passing ten people at moderate distance (lower individual risk but more people exposed). How should planners reason about distributing risk across individuals?

4. **Transparency Requirements**: Should robots be required to signal their intended path before executing it? This would give humans time to react but might slow operations. How much efficiency loss is acceptable for improved transparency?

5. **Learning from Behavior**: Motion planners increasingly learn from human demonstrations or feedback. If they learn aggressive driving styles or space-hogging behaviors from human teachers, is that acceptable? What obligations exist to audit learned behaviors?

### Stakeholders to Consider

- **People sharing space**: Pedestrians, workers, patients who must coexist with mobile robots
- **Robot operators**: Organizations deploying robots who bear responsibility for behavior
- **Vulnerable populations**: Children, elderly, disabled individuals who may have limited ability to avoid robots
- **Other road users**: Vehicles, cyclists, other robots sharing transportation infrastructure
- **Property owners**: Those whose spaces robots traverse
- **Urban planners**: Officials designing spaces that must accommodate both robots and humans
- **Insurance and liability**: Parties who bear costs when motion planning fails

### Applicable Ethical Frameworks

- **Duty of Care**: Robots operating in shared spaces have an obligation not to harm those around them through negligent path planning

- **Proportionality**: Safety margins and speed limits should be proportionate to the risk of harm in each context

- **Transparency**: Robot behavior should be understandable and predictable to those affected

- **Fairness in Allocation**: Access to shared spaces should be allocated fairly, not systematically advantaging robots over humans

- **Precautionary Principle**: When consequences of collision are severe, conservative paths are required even at efficiency cost

## Case Study: The Autonomous Delivery Robot

### The Scenario

A company deploys sidewalk delivery robots in a dense urban neighborhood. The robots use motion planning to navigate from restaurants to customers, sharing sidewalks with pedestrians.

The planning algorithm optimizes for delivery time while maintaining minimum clearance from obstacles. After extensive testing showing no collisions, the robots are deployed at scale—hundreds operating simultaneously during peak meal times.

Complaints begin arriving:
- Elderly residents report feeling "chased" by robots that approach quickly from behind
- Parents say robots cut through groups of children waiting at school bus stops
- Wheelchair users find robots blocking curb cuts while waiting to cross
- A local advocacy group argues the robots have "colonized" the sidewalk, forcing pedestrians to constantly yield

The company's data shows the robots maintain legal clearance distances and have caused no injuries. From a pure safety standpoint, the system is working.

### The Dilemma

The current planning algorithm is collision-free and efficient. However, it optimizes for the robot's objectives without considering the subjective experience of pedestrians or the accumulated burden of constantly yielding to robots. The robots are technically safe but arguably not socially appropriate.

Modifying the planner to be more deferential would reduce delivery speed by 20-30%, potentially making the business uneconomical.

### Perspectives

**Company Engineer**:
> "Our robots meet all safety standards. We maintain required clearances and have zero collision incidents. The complaints are about feelings, not safety. We can't optimize for everyone's subjective comfort."

**Elderly Resident**:
> "I used to enjoy walking to the corner store. Now I'm constantly checking over my shoulder for these robots. They come up so fast and quiet. I don't feel unsafe exactly, but I don't feel comfortable either. The sidewalk doesn't feel like mine anymore."

**City Planner**:
> "Sidewalks are public infrastructure designed for pedestrians. When we permitted these robots, we assumed they would fit into existing pedestrian flow, not that pedestrians would need to constantly accommodate them. The cumulative impact is more than we anticipated."

**Company CEO**:
> "We're providing an affordable delivery service that reduces car trips. If we slow down for every complaint, the economics don't work. People will get used to the robots, just like they got used to bicycles on shared paths."

**Disability Advocate**:
> "These robots treat accessibility features—curb cuts, wide paths, smooth surfaces—as convenient shortcuts. They're co-opting infrastructure we fought for decades to require. And when they block a curb cut 'just for a moment,' that moment means someone in a wheelchair is stuck."

### Discussion Guide

1. **Beyond Collision-Free**: Is collision avoidance sufficient for ethical motion planning, or should robots also optimize for pedestrian comfort? If so, how is comfort measured and weighted?

2. **Cumulative Impact**: Individual robots maintaining legal clearance may collectively create an environment where pedestrians feel dominated. How should regulators think about aggregate effects versus individual robot behavior?

3. **Yielding Protocol**: Design a set of social rules for robot-pedestrian interactions. When should robots wait? Slow down? Take alternate routes? How should these rules be encoded in planning algorithms?

4. **Accessibility Priority**: Should robots be prohibited from using accessibility infrastructure (curb cuts, elevators, wide paths) unless genuinely necessary? How do we balance robot access needs against the populations these features were designed for?

5. **Consent and Notification**: Should neighborhoods have the right to opt out of robot delivery? Should pedestrians receive notification when robots are operating nearby? What level of community consent should be required?

6. **Business Model Implications**: If socially appropriate motion planning makes the business unprofitable, does that mean the business shouldn't exist, or that society should subsidize socially appropriate robot behavior, or that standards should be relaxed?

---

## Key Takeaways

1. **Collision-free is necessary but not sufficient**—ethical motion planning must also consider predictability, social norms, and fairness.

2. **Cost functions encode values**—the choice of optimization objective should be a conscious ethical decision, not a default technical choice.

3. **Cumulative impacts matter**—many individually acceptable behaviors can create collectively unacceptable environments.

4. **Yielding must be negotiated**—the distribution of collision avoidance responsibility between robots and humans requires explicit policy, not implicit technical defaults.

5. **Transparency enables trust**—predictable, signaled behavior allows humans to confidently share space with robots.
