"""CA-LSC-TD3 research package."""

from .physics.capabilities import (
    aerodynamic_control_authority,
    physics_prior,
    smoothstep,
    wing_vertical_support_capability,
)

__all__ = [
    "aerodynamic_control_authority",
    "physics_prior",
    "smoothstep",
    "wing_vertical_support_capability",
]

__version__ = "0.1.0"

