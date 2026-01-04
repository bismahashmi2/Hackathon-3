---
id: cs-mfg-003
title: "Amazon and Agility Robotics: Digit in Fulfillment Centers"
domain: manufacturing
company: Amazon / Agility Robotics
robot_platform: Digit
year: 2024
related_modules: ["06", "08", "10"]
status: current
last_verified: 2024-12-01
---

# Case Study: Amazon's Deployment of Agility Digit Robots

## Summary

In October 2023, Amazon announced a pilot program to test Agility Robotics' Digit humanoid robot in its fulfillment centers. By 2024, the program expanded to multiple facilities, focusing on tote handling and recycling tasks. This represents one of the largest commercial deployments of bipedal robots in logistics.

## Background

### Agility Robotics and Digit

Agility Robotics, a spin-off from Oregon State University's ATRIAS project, developed Digit specifically for logistics and warehouse applications.

**Digit Specifications:**
- Height: 5'9" (175 cm)
- Weight: 141 lbs (64 kg)
- Payload: 35 lbs (16 kg)
- Walking speed: Up to 1.5 m/s
- Battery life: 8+ hours
- Manipulation: 4-DOF arms with parallel gripper

### Amazon's Robotics Strategy

Amazon has invested heavily in warehouse automation:
- 750,000+ robots deployed (as of 2024)
- Acquisition of Kiva Systems (2012)
- Development of Proteus autonomous mobile robot
- Investment in Agility Robotics ($150M+)

## Technical Implementation

### Primary Use Case: Tote Recycling

Digit robots handle empty plastic totes in the fulfillment workflow:

1. **Detection**: Identify empty totes on conveyors
2. **Pick**: Grasp totes from conveyor or storage
3. **Transport**: Walk to destination area
4. **Place**: Stack or organize totes for reuse

### System Architecture

```python
class DigitWarehouseTask:
    """
    Digit integration with Amazon fulfillment systems
    """
    def __init__(self, robot_id, facility_zone):
        self.robot_id = robot_id
        self.zone = facility_zone
        self.wms_client = WarehouseManagementClient()

    async def execute_tote_task(self, task):
        # Receive task from WMS
        pickup = await self.wms_client.get_pickup_location(task)
        dropoff = await self.wms_client.get_dropoff_location(task)

        # Navigate to pickup
        await self.navigate_to(pickup.location)

        # Perception and grasp
        tote_pose = self.perceive_tote()
        await self.pick_tote(tote_pose)

        # Transport
        await self.navigate_to(dropoff.location)

        # Place
        await self.place_tote(dropoff.slot)

        # Report completion
        await self.wms_client.report_complete(task)
```

### Integration Challenges

| Challenge | Solution |
|-----------|----------|
| Floor variability | Robust locomotion control with terrain adaptation |
| Dynamic obstacles | Real-time path replanning, human detection |
| Tote variations | Adaptive grasping with force sensing |
| System integration | REST APIs connecting to Amazon WMS |

## Locomotion on Warehouse Floors

### Challenges Specific to Warehouses

- **Surface transitions**: Concrete to anti-fatigue mats
- **Debris**: Small items on floor
- **Slopes**: Loading dock ramps
- **Wet areas**: Near wash stations

### Control Approach

Digit uses a hierarchical control architecture:

1. **High-Level Planner**: Path planning with A* or RRT
2. **Mid-Level Footstep Planner**: Foot placement optimization
3. **Low-Level Controller**: Real-time balance and locomotion

```python
# Simplified locomotion hierarchy
class DigitLocomotion:
    def navigate_to(self, goal):
        # High-level path
        path = self.path_planner.plan(self.position, goal)

        for waypoint in path:
            # Generate footstep plan
            footsteps = self.footstep_planner.plan(
                current=self.foot_positions,
                target=waypoint,
                terrain=self.terrain_map
            )

            # Execute with balance control
            for step in footsteps:
                self.step_controller.execute(step)
                self.balance_controller.maintain_stability()
```

## Outcomes

### Pilot Program Results

- Successful handling of thousands of totes daily
- Reduced ergonomic strain on human workers
- Integration with existing fulfillment workflows
- Data collection for continued improvement

### Scalability Considerations

- Agility building dedicated manufacturing facility
- Target: 10,000+ units annual production capacity
- Multi-facility deployment planned

## Safety Architecture

### Human-Robot Collaboration

Digit operates in shared spaces with human workers:

1. **Detection**: 360° awareness of human presence
2. **Speed Reduction**: Automatic slowdown near humans
3. **Stop**: Emergency stop capability
4. **Communication**: Visual indicators of robot state

### Compliance

- OSHA guidelines for material handling
- ANSI/RIA R15.06 industrial robot safety
- Amazon internal safety standards

## Economic Analysis

### Cost-Benefit Considerations

| Factor | Impact |
|--------|--------|
| Unit Cost | ~$250,000 per Digit robot |
| Labor Savings | Repetitive task automation |
| Injury Reduction | Ergonomic benefit quantification |
| Flexibility | Adaptable to multiple tasks |
| Scalability | Incremental deployment possible |

### ROI Factors

- Task throughput comparison
- Maintenance and support costs
- Training and integration expenses
- Productivity gains from human workers

## Discussion Questions

1. Why did Amazon choose bipedal robots over wheeled alternatives for this application?
2. How does the warehouse environment differ from factory settings for robot deployment?
3. What are the implications of large retailers driving humanoid robot development?
4. How should safety standards evolve for human-robot collaboration in logistics?

## Related Modules

- **Module 06: Motion Planning** - Path planning in dynamic environments
- **Module 08: Locomotion** - Bipedal walking control
- **Module 10: Simulation to Real** - Deployment in unstructured environments

## External References

- [Agility Robotics](https://agilityrobotics.com/)
- [Amazon Robotics](https://www.aboutamazon.com/news/tag/robotics)
- [IEEE Robotics & Automation Magazine](https://www.ieee-ras.org/)

---

*Current as of: December 2024*
