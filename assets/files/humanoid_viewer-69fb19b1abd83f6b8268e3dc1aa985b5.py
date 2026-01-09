"""
Humanoid Viewer - Visualization utilities for MuJoCo simulations.

This module provides functions for interactive visualization and
video recording of humanoid robot simulations.

Examples:
    >>> from mujoco_basics import load_humanoid
    >>> model, data = load_humanoid()
    >>> launch_viewer(model, data, duration=5.0)
"""

from typing import Optional, Callable
import numpy as np
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    raise ImportError(
        "MuJoCo is required. Install with: pip install mujoco"
    )


def launch_viewer(
    model: 'mujoco.MjModel',
    data: 'mujoco.MjData',
    duration: Optional[float] = None,
    controller: Optional[Callable] = None,
    speed: float = 1.0
) -> None:
    """
    Launch interactive MuJoCo viewer for simulation visualization.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        duration: Optional max duration in seconds (None for unlimited)
        controller: Optional callback function(model, data) called each step
        speed: Playback speed multiplier (1.0 = realtime)

    Controls:
        - Mouse drag: Rotate view
        - Scroll: Zoom
        - Double-click: Center on body
        - Space: Pause/Resume
        - Backspace: Reset
        - Escape: Close

    Example:
        >>> model, data = load_humanoid()
        >>> launch_viewer(model, data, duration=10.0)
    """
    mujoco.mj_resetData(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Check duration limit
            if duration is not None and data.time >= duration:
                break

            # Apply controller if provided
            if controller is not None:
                controller(model, data)

            # Step simulation
            mujoco.mj_step(model, data)

            # Sync viewer at appropriate rate
            viewer.sync()


def create_video(
    model: 'mujoco.MjModel',
    data: 'mujoco.MjData',
    output_path: str,
    duration: float = 5.0,
    fps: int = 30,
    resolution: tuple = (640, 480),
    controller: Optional[Callable] = None
) -> str:
    """
    Create a video recording of a simulation.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        output_path: Path for output video file
        duration: Recording duration in seconds
        fps: Frames per second
        resolution: Video resolution (width, height)
        controller: Optional callback function

    Returns:
        Path to created video file

    Example:
        >>> model, data = load_humanoid()
        >>> path = create_video(model, data, "humanoid_fall.mp4")
        >>> print(f"Video saved to {path}")
    """
    # Ensure output directory exists
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Reset simulation
    mujoco.mj_resetData(model, data)

    # Create renderer
    width, height = resolution
    renderer = mujoco.Renderer(model, height, width)

    # Calculate frame timing
    frame_dt = 1.0 / fps
    steps_per_frame = max(1, int(frame_dt / model.opt.timestep))

    frames = []
    n_frames = int(duration * fps)

    for frame_idx in range(n_frames):
        # Step simulation
        for _ in range(steps_per_frame):
            if controller is not None:
                controller(model, data)
            mujoco.mj_step(model, data)

        # Render frame
        renderer.update_scene(data)
        frame = renderer.render()
        frames.append(frame.copy())

    # Save video (requires imageio)
    try:
        import imageio
        imageio.mimsave(str(output_path), frames, fps=fps)
        return str(output_path)
    except ImportError:
        # Fallback: save as image sequence
        img_dir = output_path.parent / f"{output_path.stem}_frames"
        img_dir.mkdir(exist_ok=True)
        for i, frame in enumerate(frames):
            import PIL.Image
            img = PIL.Image.fromarray(frame)
            img.save(img_dir / f"frame_{i:04d}.png")
        return str(img_dir)


def render_frame(
    model: 'mujoco.MjModel',
    data: 'mujoco.MjData',
    resolution: tuple = (640, 480),
    camera: Optional[str] = None
) -> np.ndarray:
    """
    Render a single frame from current simulation state.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        resolution: Image resolution (width, height)
        camera: Optional camera name (None for default)

    Returns:
        RGB image as numpy array (height, width, 3)

    Example:
        >>> model, data = load_humanoid()
        >>> frame = render_frame(model, data)
        >>> print(frame.shape)
        (480, 640, 3)
    """
    width, height = resolution
    renderer = mujoco.Renderer(model, height, width)

    if camera is not None:
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        renderer.update_scene(data, camera_id)
    else:
        renderer.update_scene(data)

    return renderer.render()


def visualize_joints(
    model: 'mujoco.MjModel',
    data: 'mujoco.MjData'
) -> None:
    """
    Print visualization of joint states.

    Args:
        model: MuJoCo model
        data: MuJoCo data

    Example:
        >>> model, data = load_humanoid()
        >>> visualize_joints(model, data)
        Joint States:
        ...
    """
    print("Joint States:")
    print("-" * 50)
    print(f"{'Joint Name':<20} {'Position':>10} {'Velocity':>10}")
    print("-" * 50)

    qpos_idx = 0
    qvel_idx = 0

    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]

        # Get position and velocity based on joint type
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            pos = data.qpos[qpos_idx:qpos_idx+7]
            vel = data.qvel[qvel_idx:qvel_idx+6]
            pos_str = f"[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]"
            vel_str = f"[{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}]"
            qpos_idx += 7
            qvel_idx += 6
        elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
            pos = data.qpos[qpos_idx:qpos_idx+4]
            vel = data.qvel[qvel_idx:qvel_idx+3]
            pos_str = f"{np.linalg.norm(pos):.3f}"
            vel_str = f"{np.linalg.norm(vel):.3f}"
            qpos_idx += 4
            qvel_idx += 3
        else:  # Hinge or slide
            pos = data.qpos[qpos_idx]
            vel = data.qvel[qvel_idx]
            pos_str = f"{pos:.3f}"
            vel_str = f"{vel:.3f}"
            qpos_idx += 1
            qvel_idx += 1

        print(f"{joint_name:<20} {pos_str:>10} {vel_str:>10}")


if __name__ == "__main__":
    from mujoco_basics import load_humanoid

    print("Loading humanoid model...")
    model, data = load_humanoid()

    print("\nInitial joint states:")
    visualize_joints(model, data)

    print("\nLaunching viewer (close window to exit)...")
    launch_viewer(model, data, duration=10.0)
