# MuJoCo Robot Models

This directory contains MJCF (MuJoCo XML) robot model files for use in the textbook simulations.

## Available Models

| Model | File | Description | Modules |
|-------|------|-------------|---------|
| 2-DOF Arm | `2dof-arm.xml` | Simple planar two-link arm | 03, 05 |
| 6-DOF Arm | `6dof-arm.xml` | Industrial-style manipulator | 06, 07 |
| Simple Humanoid | `humanoid-simple.xml` | Reduced-DOF humanoid for learning | 01, 02, 08 |
| Full Humanoid | `humanoid.xml` | Complete humanoid model | 10-14 |
| Mobile Base | `mobile-base.xml` | Wheeled platform | 04, 09 |

## Model Conventions

- All models use SI units (meters, kilograms, seconds)
- Joint limits are defined to prevent self-collision
- Actuator ranges match typical servo specifications
- Default colors: blue (links), red (joints), green (end-effectors)

## Adding New Models

1. Create the MJCF XML file
2. Add mesh files to `meshes/` subdirectory
3. Update this README with model documentation
4. Test with `mujoco.viewer.launch()`

## References

- [MuJoCo MJCF Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [MuJoCo Model Gallery](https://mujoco.readthedocs.io/en/stable/models.html)
