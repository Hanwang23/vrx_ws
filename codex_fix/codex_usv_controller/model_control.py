"""Safety gate and plant model interface for a future NMPC backend."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class FirstOrderAxis:
    state_coefficient: float
    input_gain: float
    bias: float = 0.0

    def derivative(self, state: float, control: float) -> float:
        return self.state_coefficient * state + self.input_gain * control + self.bias


@dataclass(frozen=True)
class PlanarThreeDofModel:
    surge: FirstOrderAxis
    yaw: FirstOrderAxis
    sway_damping: float = 0.5

    def step(
        self,
        state: Tuple[float, float, float, float, float, float],
        average_thrust: float,
        differential_thrust: float,
        dt: float,
    ) -> Tuple[float, float, float, float, float, float]:
        east, north, yaw, surge, sway, yaw_rate = state
        dt = min(0.5, max(0.0, dt))
        surge += dt * self.surge.derivative(surge, average_thrust)
        sway += dt * (-abs(self.sway_damping) * sway)
        yaw_rate += dt * self.yaw.derivative(yaw_rate, differential_thrust)
        yaw += dt * yaw_rate
        east += dt * (surge * math.cos(yaw) - sway * math.sin(yaw))
        north += dt * (surge * math.sin(yaw) + sway * math.cos(yaw))
        return east, north, yaw, surge, sway, yaw_rate


@dataclass(frozen=True)
class ModelControlStatus:
    requested_backend: str
    active_backend: str
    nmpc_ready: bool
    reason: str


def gate_model_control(
    requested_backend: str,
    fitted_model: Optional[dict],
) -> ModelControlStatus:
    requested = str(requested_backend).lower()
    if requested != 'nmpc':
        return ModelControlStatus(requested, 'ilos_pid', False, 'baseline selected')
    if not fitted_model or not fitted_model.get('nmpc_ready'):
        return ModelControlStatus(
            requested, 'ilos_pid', False,
            'fitted 3-DOF model is absent or failed readiness gates')
    return ModelControlStatus(
        requested, 'ilos_pid', True,
        'model passed readiness gates; NMPC solver is not permitted to actuate yet')
