"""
MuJoCo Basics - Fundamental operations for Physical AI simulation.

This module provides functions for loading models, running simulations,
and extracting information from MuJoCo physics simulations.

Examples:
    >>> model, data = load_humanoid()
    >>> info = get_model_info(model)
    >>> print(f"Model has {info['num_bodies']} bodies")
    Model has 14 bodies
"""

from typing import Tuple, Dict, Any, Optional
import numpy as np

try:
    import mujoco
except ImportError:
    raise ImportError(
        "MuJoCo is required. Install with: pip install mujoco"
    )


def load_humanoid() -> Tuple['mujoco.MjModel', 'mujoco.MjData']:
    """
    Load the standard MuJoCo humanoid model.

    Returns:
        Tuple containing:
            - model: MjModel object with robot specification
            - data: MjData object for simulation state

    Example:
        >>> model, data = load_humanoid()
        >>> print(model.nv)  # Degrees of freedom
        27
    """
    model_path = mujoco.util.get_resource_path("humanoid/humanoid.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    return model, data


def get_model_info(model: 'mujoco.MjModel') -> Dict[str, Any]:
    """
    Extract key information from a MuJoCo model.

    Args:
        model: MuJoCo model object

    Returns:
        Dictionary containing model properties

    Example:
        >>> model, _ = load_humanoid()
        >>> info = get_model_info(model)
        >>> info['num_joints']
        18
    """
    return {
        'num_bodies': model.nbody,
        'num_joints': model.njnt,
        'num_actuators': model.nu,
        'degrees_of_freedom': model.nv,
        'timestep': model.opt.timestep,
        'gravity': model.opt.gravity.tolist(),
    }


def run_simulation(
    model: 'mujoco.MjModel',
    data: 'mujoco.MjData',
    duration: float = 1.0,
    controller: Optional[callable] = None,
    record_interval: int = 10
) -> Dict[str, np.ndarray]:
    """
    Run a physics simulation for a specified duration.

    Args:
        model: MuJoCo model
        data: MuJoCo data (will be modified in place)
        duration: Simulation duration in seconds
        controller: Optional callback function(model, data) called each step
        record_interval: Record state every N steps

    Returns:
        Dictionary with recorded trajectories:
            - 'time': timestamps
            - 'qpos': generalized positions
            - 'qvel': generalized velocities
            - 'energy': total mechanical energy

    Example:
        >>> model, data = load_humanoid()
        >>> traj = run_simulation(model, data, duration=2.0)
        >>> print(f"Recorded {len(traj['time'])} frames")
        Recorded 100 frames
    """
    # Reset simulation
    mujoco.mj_resetData(model, data)

    # Calculate number of steps
    n_steps = int(duration / model.opt.timestep)

    # Initialize recording arrays
    n_records = n_steps // record_interval + 1
    trajectories = {
        'time': np.zeros(n_records),
        'qpos': np.zeros((n_records, model.nq)),
        'qvel': np.zeros((n_records, model.nv)),
        'energy': np.zeros(n_records),
    }

    record_idx = 0

    for step in range(n_steps):
        # Apply controller if provided
        if controller is not None:
            controller(model, data)

        # Step simulation
        mujoco.mj_step(model, data)

        # Record at intervals
        if step % record_interval == 0 and record_idx < n_records:
            trajectories['time'][record_idx] = data.time
            trajectories['qpos'][record_idx] = data.qpos.copy()
            trajectories['qvel'][record_idx] = data.qvel.copy()
            trajectories['energy'][record_idx] = _compute_energy(model, data)
            record_idx += 1

    return trajectories


def _compute_energy(model: 'mujoco.MjModel', data: 'mujoco.MjData') -> float:
    """
    Compute total mechanical energy (kinetic + potential).

    Args:
        model: MuJoCo model
        data: MuJoCo data

    Returns:
        Total energy in Joules
    """
    # Kinetic energy: 0.5 * v^T * M * v
    mass_matrix = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mass_matrix, data.qM)
    kinetic = 0.5 * data.qvel @ mass_matrix @ data.qvel

    # Potential energy: sum of m * g * h for each body
    potential = 0.0
    for i in range(model.nbody):
        mass = model.body_mass[i]
        height = data.xpos[i, 2]  # z-coordinate
        potential += mass * 9.81 * height

    return kinetic + potential


def reset_to_pose(
    model: 'mujoco.MjModel',
    data: 'mujoco.MjData',
    qpos: Optional[np.ndarray] = None
) -> None:
    """
    Reset simulation to a specific pose.

    Args:
        model: MuJoCo model
        data: MuJoCo data (modified in place)
        qpos: Target generalized positions (None for default)

    Example:
        >>> model, data = load_humanoid()
        >>> custom_pose = np.zeros(model.nq)
        >>> reset_to_pose(model, data, custom_pose)
    """
    mujoco.mj_resetData(model, data)

    if qpos is not None:
        if len(qpos) != model.nq:
            raise ValueError(
                f"qpos length {len(qpos)} != model.nq {model.nq}"
            )
        data.qpos[:] = qpos

    # Forward kinematics to update body positions
    mujoco.mj_forward(model, data)


if __name__ == "__main__":
    # Demo usage
    print("Loading humanoid model...")
    model, data = load_humanoid()

    print("\nModel information:")
    info = get_model_info(model)
    for key, value in info.items():
        print(f"  {key}: {value}")

    print("\nRunning simulation for 2 seconds...")
    traj = run_simulation(model, data, duration=2.0)
    print(f"Recorded {len(traj['time'])} frames")
    print(f"Final height: {traj['qpos'][-1, 2]:.3f} m")
    print(f"Energy change: {traj['energy'][-1] - traj['energy'][0]:.2f} J")
