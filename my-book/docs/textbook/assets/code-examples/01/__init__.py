"""
Module 01: Introduction to Physical AI - Code Examples

This module provides basic examples for getting started with MuJoCo
simulation and understanding Physical AI fundamentals.
"""

from .mujoco_basics import load_humanoid, run_simulation, get_model_info
from .humanoid_viewer import launch_viewer, create_video

__all__ = [
    'load_humanoid',
    'run_simulation',
    'get_model_info',
    'launch_viewer',
    'create_video',
]
