---
id: cs-hlth-001
title: "Diligent Robotics Moxi: Hospital Logistics Automation"
domain: healthcare
company: Diligent Robotics
robot_platform: Moxi
year: 2024
related_modules: ["06", "09", "12"]
status: current
last_verified: 2024-12-01
---

# Case Study: Moxi Hospital Logistics Robot

## Summary

Diligent Robotics' Moxi robot represents one of the most successful deployments of autonomous mobile manipulation in healthcare settings. Operating in over 500 hospital facilities by 2024, Moxi handles logistics tasks such as delivering supplies, lab samples, and medications, demonstrating how robots can support healthcare workers.

## Background

### Diligent Robotics

Founded in 2017 by Andrea Thomaz and Vivian Chu, Diligent Robotics emerged from research in human-robot interaction at Georgia Tech and UT Austin.

**Company Focus:**
- Hospital logistics automation
- Human-centered design
- Nurse workflow optimization
- Safe autonomous operation

### Moxi Platform

**Physical Specifications:**
- Height: 4'7" (140 cm)
- Weight: 330 lbs (150 kg)
- Manipulation: Single 7-DOF arm
- Navigation: 2D LIDAR + depth cameras
- Social cues: LED eyes, head movements

**Capabilities:**
- Autonomous navigation in hospitals
- Door operation (handles and buttons)
- Elevator usage
- Pickup and delivery of supplies
- Basic manipulation tasks

## Technical Implementation

### Navigation System

Moxi uses a sophisticated navigation stack for hospital environments:

```python
class MoxiNavigation:
    """
    Hospital-optimized navigation system
    """
    def __init__(self):
        self.map = SemanticHospitalMap()
        self.planner = HybridPlanner()
        self.social_nav = SocialNavigationLayer()

    async def navigate_to_room(self, destination):
        # Plan path considering hospital semantics
        path = self.planner.plan(
            start=self.current_location,
            goal=destination,
            constraints={
                "avoid_patient_rooms_during_quiet_hours": True,
                "prefer_service_corridors": True,
                "avoid_crowded_areas": True,
            }
        )

        # Execute with social awareness
        for waypoint in path:
            await self.social_nav.navigate(
                waypoint,
                yield_to_humans=True,
                maintain_distance=1.5  # meters
            )

    async def traverse_door(self, door):
        if door.requires_credential:
            await self.request_door_access(door)
        else:
            await self.open_door(door)
```

### Task Management

Moxi integrates with hospital systems for task assignment:

```yaml
moxi_integration:
  hospital_systems:
    - name: "Nurse Call System"
      integration: "Webhook notifications"
      tasks: "Supply delivery requests"

    - name: "Pharmacy System"
      integration: "HL7 FHIR API"
      tasks: "Medication delivery"

    - name: "Laboratory Information System"
      integration: "REST API"
      tasks: "Specimen transport"

  task_priorities:
    stat: 1  # Urgent, immediate response
    routine: 2  # Standard delivery
    scheduled: 3  # Pre-planned rounds
```

## Hospital Workflow Integration

### Nurse Time Savings

Studies show nurses spend significant time on logistics:
- Walking: ~3 miles per shift
- Supply gathering: 30+ minutes per shift
- Documentation fetching: Variable

Moxi handles repetitive logistics, returning time to patient care.

### Typical Daily Tasks

| Task Type | Volume | Time Saved |
|-----------|--------|------------|
| Supply delivery | 50-100/day | 2-3 hours |
| Lab sample pickup | 20-40/day | 1-2 hours |
| Linen delivery | 10-20/day | 30-60 min |
| Pharmacy runs | 10-30/day | 1-2 hours |

## Safety and Compliance

### Healthcare-Specific Requirements

1. **Infection Control**
   - Wipeable surfaces
   - UV-C disinfection compatible
   - Hand sanitizer dispenser integration

2. **Patient Privacy**
   - Limited data collection
   - HIPAA compliance
   - No patient imagery storage

3. **Clinical Safety**
   - No direct patient contact
   - Clear of crash cart paths
   - Emergency stop accessible

### Regulatory Considerations

```
Healthcare Robot Compliance Matrix:
├── FDA: Not a medical device (logistics only)
├── HIPAA: Limited PHI exposure
├── Joint Commission: Facility safety standards
└── State regulations: Variable by jurisdiction
```

## Human-Robot Interaction Design

### Social Navigation

Moxi exhibits socially appropriate behavior:

1. **Yielding**: Steps aside for humans in corridors
2. **Speed adjustment**: Slows near patients/visitors
3. **Gaze direction**: "Looks" at people appropriately
4. **Audio cues**: Pleasant sounds to indicate presence

### Staff Acceptance

Key factors in successful adoption:

| Factor | Implementation |
|--------|----------------|
| Introduction | Staff orientation sessions |
| Naming | Hospitals often name their Moxi units |
| Feedback | Easy reporting of issues |
| Recognition | Robots "thanked" by staff |

## Outcomes

### Quantified Results

- 500+ hospital facilities deployed
- Millions of deliveries completed
- High staff satisfaction scores
- Reduced nurse walking distance

### Qualitative Benefits

1. Nurses report feeling "supported"
2. Reduced physical strain from carrying
3. More predictable supply availability
4. Improved workflow timing

## Challenges and Limitations

### Technical Challenges

1. **Hospital complexity**: Maze-like layouts
2. **Dynamic environments**: Constant changes
3. **Connectivity**: WiFi reliability varies
4. **Manipulation limits**: Single arm restricts payload variety

### Operational Challenges

1. **Elevator sharing**: Human patience varies
2. **Door access**: Credential management
3. **Charging logistics**: Uptime requirements
4. **Maintenance**: 24/7 hospital operation

## Business Model

### Deployment Approach

- Robot-as-a-Service (RaaS) model
- Monthly subscription pricing
- Includes maintenance and updates
- Scalable with hospital needs

### Economic Justification

The ROI case typically includes:
- Nursing time savings (primary)
- Reduced travel injuries
- Improved supply tracking
- Staff satisfaction/retention

## Discussion Questions

1. How does Moxi's design reflect human-centered robotics principles?
2. What makes healthcare environments uniquely challenging for robots?
3. How should hospitals measure the value of logistics robots?
4. What tasks should remain human-only in healthcare settings?

## Related Modules

- **Module 06: Motion Planning** - Navigation in complex environments
- **Module 09: ROS2 Integration** - System integration
- **Module 12: Human-Robot Interaction** - Social navigation

## External References

- [Diligent Robotics](https://www.diligentrobots.com/)
- [American Hospital Association](https://www.aha.org/)
- [Healthcare Robotics Research](https://ieeexplore.ieee.org/)

---

*Current as of: December 2024*
