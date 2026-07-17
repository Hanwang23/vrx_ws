"""Small dependency-free planar EKF for GNSS and IMU fusion."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple

from .core import enu_to_geodetic, geodetic_delta_m, normalize_angle


@dataclass(frozen=True)
class EstimatorConfig:
    position_process_noise: float = 0.15
    velocity_process_noise: float = 0.8
    yaw_process_noise: float = math.radians(2.0)
    yaw_rate_process_noise: float = math.radians(8.0)
    default_gps_std: float = 1.5
    default_yaw_std: float = math.radians(3.0)
    default_yaw_rate_std: float = math.radians(2.0)
    velocity_measurement_std: float = 0.8
    innovation_gate_sigma: float = 6.0
    max_speed: float = 15.0
    max_position_std: float = 8.0


@dataclass(frozen=True)
class StateEstimate:
    latitude: float
    longitude: float
    east: float
    north: float
    velocity_east: float
    velocity_north: float
    yaw: float
    yaw_rate: float
    position_std: float
    velocity_std: float
    yaw_std: float
    healthy: bool
    rejected_measurements: int

    @property
    def forward_speed(self) -> float:
        return (
            self.velocity_east * math.cos(self.yaw)
            + self.velocity_north * math.sin(self.yaw)
        )


def _identity(size: int) -> List[List[float]]:
    return [
        [1.0 if row == column else 0.0 for column in range(size)]
        for row in range(size)
    ]


def _multiply(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]):
    return [
        [
            sum(left[row][inner] * right[inner][column]
                for inner in range(len(right)))
            for column in range(len(right[0]))
        ]
        for row in range(len(left))
    ]


def _transpose(matrix: Sequence[Sequence[float]]):
    return [list(values) for values in zip(*matrix)]


class PlanarEKF:
    """Constant-velocity EKF with scalar Joseph-form measurement updates."""

    STATE_SIZE = 6
    EAST = 0
    NORTH = 1
    VELOCITY_EAST = 2
    VELOCITY_NORTH = 3
    YAW = 4
    YAW_RATE = 5

    def __init__(self, config: EstimatorConfig = EstimatorConfig()) -> None:
        self.config = config
        self.state = [0.0] * self.STATE_SIZE
        self.covariance = _identity(self.STATE_SIZE)
        self.origin: Optional[Tuple[float, float]] = None
        self.last_timestamp: Optional[float] = None
        self.last_gps: Optional[Tuple[float, float, float]] = None
        self.position_initialized = False
        self.yaw_initialized = False
        self.rejected_measurements = 0

    @property
    def initialized(self) -> bool:
        return self.position_initialized and self.yaw_initialized

    def reset(self) -> None:
        self.__init__(self.config)

    def predict(self, timestamp: float) -> None:
        if not math.isfinite(timestamp):
            return
        if self.last_timestamp is None:
            self.last_timestamp = timestamp
            return
        dt = timestamp - self.last_timestamp
        if dt <= 0.0:
            return
        dt = min(dt, 1.0)
        self.last_timestamp = timestamp
        self.state[self.EAST] += self.state[self.VELOCITY_EAST] * dt
        self.state[self.NORTH] += self.state[self.VELOCITY_NORTH] * dt
        self.state[self.YAW] = normalize_angle(
            self.state[self.YAW] + self.state[self.YAW_RATE] * dt)

        transition = _identity(self.STATE_SIZE)
        transition[self.EAST][self.VELOCITY_EAST] = dt
        transition[self.NORTH][self.VELOCITY_NORTH] = dt
        transition[self.YAW][self.YAW_RATE] = dt
        predicted = _multiply(
            _multiply(transition, self.covariance), _transpose(transition))
        noises = (
            self.config.position_process_noise ** 2,
            self.config.position_process_noise ** 2,
            self.config.velocity_process_noise ** 2,
            self.config.velocity_process_noise ** 2,
            self.config.yaw_process_noise ** 2,
            self.config.yaw_rate_process_noise ** 2,
        )
        for index, noise in enumerate(noises):
            predicted[index][index] += noise * dt
        self.covariance = predicted

    def _update_scalar(
        self,
        index: int,
        measurement: float,
        variance: float,
        wrap_angle: bool = False,
        gate: bool = True,
    ) -> bool:
        variance = max(1e-9, float(variance))
        innovation = measurement - self.state[index]
        if wrap_angle:
            innovation = normalize_angle(innovation)
        innovation_variance = self.covariance[index][index] + variance
        if (
            gate
            and innovation * innovation
            > self.config.innovation_gate_sigma ** 2 * innovation_variance
        ):
            self.rejected_measurements += 1
            return False
        gain = [
            self.covariance[row][index] / innovation_variance
            for row in range(self.STATE_SIZE)
        ]
        for row in range(self.STATE_SIZE):
            self.state[row] += gain[row] * innovation
        self.state[self.YAW] = normalize_angle(self.state[self.YAW])

        identity_minus_kh = _identity(self.STATE_SIZE)
        for row in range(self.STATE_SIZE):
            identity_minus_kh[row][index] -= gain[row]
        joseph = _multiply(
            _multiply(identity_minus_kh, self.covariance),
            _transpose(identity_minus_kh),
        )
        for row in range(self.STATE_SIZE):
            for column in range(self.STATE_SIZE):
                joseph[row][column] += gain[row] * variance * gain[column]
        self.covariance = joseph
        return True

    def update_gps(
        self,
        latitude: float,
        longitude: float,
        timestamp: float,
        position_variances: Optional[Tuple[float, float]] = None,
    ) -> bool:
        if not all(math.isfinite(value) for value in (
            latitude, longitude, timestamp,
        )):
            return False
        if self.origin is None:
            self.origin = (latitude, longitude)
        east, north = geodetic_delta_m(
            self.origin[0], self.origin[1], latitude, longitude)
        self.predict(timestamp)
        default_variance = self.config.default_gps_std ** 2
        east_variance, north_variance = position_variances or (
            default_variance, default_variance)
        if not self.position_initialized:
            self.state[self.EAST] = east
            self.state[self.NORTH] = north
            self.covariance[self.EAST][self.EAST] = max(east_variance, 1e-6)
            self.covariance[self.NORTH][self.NORTH] = max(north_variance, 1e-6)
            self.position_initialized = True
            accepted = True
        else:
            residual_east = east - self.state[self.EAST]
            residual_north = north - self.state[self.NORTH]
            residual_variance = (
                self.covariance[self.EAST][self.EAST] + east_variance
                + self.covariance[self.NORTH][self.NORTH] + north_variance
            )
            if (
                residual_east ** 2 + residual_north ** 2
                > self.config.innovation_gate_sigma ** 2 * residual_variance
            ):
                self.rejected_measurements += 1
                return False
            accepted = self._update_scalar(
                self.EAST, east, east_variance, gate=False)
            accepted = self._update_scalar(
                self.NORTH, north, north_variance, gate=False) and accepted

        if self.last_gps is not None:
            previous_east, previous_north, previous_time = self.last_gps
            dt = timestamp - previous_time
            if 0.05 <= dt <= 3.0:
                velocity_east = (east - previous_east) / dt
                velocity_north = (north - previous_north) / dt
                if math.hypot(velocity_east, velocity_north) <= self.config.max_speed:
                    variance = self.config.velocity_measurement_std ** 2
                    self._update_scalar(
                        self.VELOCITY_EAST, velocity_east, variance)
                    self._update_scalar(
                        self.VELOCITY_NORTH, velocity_north, variance)
        if accepted:
            self.last_gps = (east, north, timestamp)
        return accepted

    def update_imu(
        self,
        yaw: float,
        yaw_rate: Optional[float],
        timestamp: float,
        yaw_variance: Optional[float] = None,
        yaw_rate_variance: Optional[float] = None,
    ) -> bool:
        if not math.isfinite(yaw) or not math.isfinite(timestamp):
            return False
        self.predict(timestamp)
        yaw_variance = (
            self.config.default_yaw_std ** 2
            if yaw_variance is None or not math.isfinite(yaw_variance)
            else max(yaw_variance, 1e-9)
        )
        if not self.yaw_initialized:
            self.state[self.YAW] = normalize_angle(yaw)
            self.covariance[self.YAW][self.YAW] = yaw_variance
            self.yaw_initialized = True
            accepted = True
        else:
            accepted = self._update_scalar(
                self.YAW, yaw, yaw_variance, wrap_angle=True)
        if yaw_rate is not None and math.isfinite(yaw_rate):
            variance = (
                self.config.default_yaw_rate_std ** 2
                if yaw_rate_variance is None or not math.isfinite(yaw_rate_variance)
                else max(yaw_rate_variance, 1e-9)
            )
            self._update_scalar(self.YAW_RATE, yaw_rate, variance)
        return accepted

    def estimate(self) -> Optional[StateEstimate]:
        if not self.initialized or self.origin is None:
            return None
        latitude, longitude = enu_to_geodetic(
            self.origin[0], self.origin[1],
            self.state[self.EAST], self.state[self.NORTH])
        position_std = math.sqrt(max(
            0.0,
            self.covariance[self.EAST][self.EAST]
            + self.covariance[self.NORTH][self.NORTH],
        ))
        velocity_std = math.sqrt(max(
            0.0,
            self.covariance[self.VELOCITY_EAST][self.VELOCITY_EAST]
            + self.covariance[self.VELOCITY_NORTH][self.VELOCITY_NORTH],
        ))
        yaw_std = math.sqrt(max(0.0, self.covariance[self.YAW][self.YAW]))
        finite = all(math.isfinite(value) for value in self.state)
        return StateEstimate(
            latitude=latitude,
            longitude=longitude,
            east=self.state[self.EAST],
            north=self.state[self.NORTH],
            velocity_east=self.state[self.VELOCITY_EAST],
            velocity_north=self.state[self.VELOCITY_NORTH],
            yaw=self.state[self.YAW],
            yaw_rate=self.state[self.YAW_RATE],
            position_std=position_std,
            velocity_std=velocity_std,
            yaw_std=yaw_std,
            healthy=finite and position_std <= self.config.max_position_std,
            rejected_measurements=self.rejected_measurements,
        )
