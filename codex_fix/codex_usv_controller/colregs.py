"""Lightweight moving-target tracking and COLREGs encounter supervision."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .core import normalize_angle


@dataclass
class DynamicTrack:
    track_id: int
    east: float
    north: float
    velocity_east: float = 0.0
    velocity_north: float = 0.0
    covariance_m2: float = 4.0
    hits: int = 1
    timestamp: float = 0.0
    first_east: Optional[float] = None
    first_north: Optional[float] = None
    first_timestamp: Optional[float] = None
    motion_consistency: int = 0
    last_motion_heading: Optional[float] = None

    @property
    def speed(self) -> float:
        return math.hypot(self.velocity_east, self.velocity_north)


@dataclass(frozen=True)
class EncounterAssessment:
    track_id: int
    encounter: str
    action: str
    range_m: float
    relative_bearing: float
    tcpa_s: Optional[float]
    dcpa_m: Optional[float]
    risk: bool
    heading_bias: float = 0.0
    speed_scale: float = 1.0


class DynamicTargetTracker:
    """Nearest-neighbor alpha-beta tracker in a fixed local ENU frame."""

    def __init__(
        self,
        match_distance: float = 4.0,
        timeout: float = 4.0,
        position_gain: float = 0.65,
        velocity_gain: float = 0.35,
        max_speed: float = 15.0,
    ) -> None:
        self.match_distance = max(0.1, float(match_distance))
        self.timeout = max(0.1, float(timeout))
        self.position_gain = min(1.0, max(0.0, position_gain))
        self.velocity_gain = min(1.0, max(0.0, velocity_gain))
        self.max_speed = max(0.1, float(max_speed))
        self.tracks: Dict[int, DynamicTrack] = {}
        self.next_track_id = 1

    def reset(self) -> None:
        self.tracks.clear()
        self.next_track_id = 1

    def update(
        self,
        detections: Iterable[Tuple[float, float]],
        timestamp: float,
    ) -> Tuple[DynamicTrack, ...]:
        if not math.isfinite(timestamp):
            return self.active_tracks(0.0)
        self.tracks = {
            track_id: track for track_id, track in self.tracks.items()
            if timestamp - track.timestamp <= self.timeout
        }
        available = set(self.tracks)
        for east, north in detections:
            if not math.isfinite(east) or not math.isfinite(north):
                continue
            best_id = None
            best_distance = self.match_distance
            for track_id in available:
                track = self.tracks[track_id]
                dt = max(0.0, timestamp - track.timestamp)
                predicted_east = track.east + track.velocity_east * dt
                predicted_north = track.north + track.velocity_north * dt
                distance = math.hypot(
                    east - predicted_east, north - predicted_north)
                if distance <= best_distance:
                    best_distance = distance
                    best_id = track_id
            if best_id is None:
                track_id = self.next_track_id
                self.next_track_id += 1
                self.tracks[track_id] = DynamicTrack(
                    track_id=track_id, east=east, north=north,
                    timestamp=timestamp,
                    first_east=east,
                    first_north=north,
                    first_timestamp=timestamp,
                )
                continue

            available.remove(best_id)
            track = self.tracks[best_id]
            dt = timestamp - track.timestamp
            if dt <= 1e-3:
                continue
            measured_velocity_east = (east - track.east) / dt
            measured_velocity_north = (north - track.north) / dt
            measured_speed = math.hypot(
                measured_velocity_east, measured_velocity_north)
            if measured_speed <= self.max_speed:
                if measured_speed >= 0.2:
                    motion_heading = math.atan2(
                        measured_velocity_north, measured_velocity_east)
                    if track.last_motion_heading is None:
                        track.motion_consistency = 1
                    elif abs(normalize_angle(
                        motion_heading - track.last_motion_heading
                    )) <= math.radians(35.0):
                        track.motion_consistency = min(
                            1000, track.motion_consistency + 1)
                    else:
                        track.motion_consistency = max(
                            0, track.motion_consistency - 2)
                    track.last_motion_heading = motion_heading
                track.velocity_east += self.velocity_gain * (
                    measured_velocity_east - track.velocity_east)
                track.velocity_north += self.velocity_gain * (
                    measured_velocity_north - track.velocity_north)
            predicted_east = track.east + track.velocity_east * dt
            predicted_north = track.north + track.velocity_north * dt
            residual_east = east - predicted_east
            residual_north = north - predicted_north
            track.east = predicted_east + self.position_gain * residual_east
            track.north = predicted_north + self.position_gain * residual_north
            residual_squared = residual_east ** 2 + residual_north ** 2
            track.covariance_m2 = 0.8 * track.covariance_m2 + 0.2 * residual_squared
            track.hits = min(1000, track.hits + 1)
            track.timestamp = timestamp
        return self.active_tracks(timestamp)

    def active_tracks(self, timestamp: float) -> Tuple[DynamicTrack, ...]:
        return tuple(
            track for track in self.tracks.values()
            if timestamp <= 0.0 or timestamp - track.timestamp <= self.timeout
        )


def is_confirmed_moving(
    target: DynamicTrack,
    minimum_target_speed: float = 0.35,
    minimum_hits: int = 4,
    maximum_covariance_m2: float = 2.5,
) -> bool:
    track_age = (
        target.timestamp - target.first_timestamp
        if target.first_timestamp is not None else 0.0)
    displacement = (
        math.hypot(
            target.east - target.first_east,
            target.north - target.first_north,
        )
        if target.first_east is not None and target.first_north is not None
        else 0.0
    )
    return bool(
        target.hits >= minimum_hits
        and target.speed >= minimum_target_speed
        and track_age >= 2.0
        and displacement >= max(2.0, 0.5 * minimum_target_speed * track_age)
        and target.motion_consistency >= max(3, minimum_hits // 2)
        and target.covariance_m2 <= maximum_covariance_m2
    )


def assess_encounter(
    own_position: Tuple[float, float],
    own_velocity: Tuple[float, float],
    own_yaw: float,
    target: DynamicTrack,
    safety_radius: float = 15.0,
    time_horizon: float = 120.0,
    minimum_target_speed: float = 0.35,
    minimum_hits: int = 4,
    maximum_covariance_m2: float = 2.5,
) -> EncounterAssessment:
    relative_east = target.east - own_position[0]
    relative_north = target.north - own_position[1]
    range_m = math.hypot(relative_east, relative_north)
    forward = (
        relative_east * math.cos(own_yaw)
        + relative_north * math.sin(own_yaw))
    left = (
        -relative_east * math.sin(own_yaw)
        + relative_north * math.cos(own_yaw))
    bearing = math.atan2(left, forward)
    relative_velocity = (
        target.velocity_east - own_velocity[0],
        target.velocity_north - own_velocity[1],
    )
    relative_speed_squared = (
        relative_velocity[0] ** 2 + relative_velocity[1] ** 2)
    tcpa = None
    dcpa = None
    if relative_speed_squared > 1e-4:
        tcpa = -(
            relative_east * relative_velocity[0]
            + relative_north * relative_velocity[1]
        ) / relative_speed_squared
        closest = (
            relative_east + relative_velocity[0] * max(0.0, tcpa),
            relative_north + relative_velocity[1] * max(0.0, tcpa),
        )
        dcpa = math.hypot(*closest)

    confirmed_moving = is_confirmed_moving(
        target,
        minimum_target_speed=minimum_target_speed,
        minimum_hits=minimum_hits,
        maximum_covariance_m2=maximum_covariance_m2,
    )
    risk = bool(
        confirmed_moving
        and tcpa is not None and dcpa is not None
        and 0.0 < tcpa <= time_horizon
        and dcpa < safety_radius
    )
    encounter = 'none'
    action = 'none'
    heading_bias = 0.0
    speed_scale = 1.0
    if risk:
        own_speed = math.hypot(*own_velocity)
        own_course = (
            math.atan2(own_velocity[1], own_velocity[0])
            if own_speed >= 0.2 else own_yaw)
        target_course = math.atan2(
            target.velocity_north, target.velocity_east)
        reciprocal = abs(normalize_angle(target_course - own_course))
        target_forward = (
            -relative_east * math.cos(target_course)
            - relative_north * math.sin(target_course))
        target_left = (
            relative_east * math.sin(target_course)
            - relative_north * math.cos(target_course))
        own_bearing_from_target = math.atan2(target_left, target_forward)
        similar_course = reciprocal <= math.radians(67.5)
        own_is_overtaking = bool(
            similar_course
            and abs(own_bearing_from_target) > math.radians(112.5)
            and own_speed > target.speed + 0.1
        )
        own_is_being_overtaken = bool(
            similar_course
            and abs(bearing) > math.radians(112.5)
            and target.speed > own_speed + 0.1
        )
        if abs(bearing) <= math.radians(15.0) and reciprocal >= math.radians(150.0):
            encounter = 'head_on'
            action = 'give_way_starboard'
            heading_bias = math.radians(-25.0)
            speed_scale = 0.60
        elif own_is_overtaking:
            encounter = 'overtaking'
            action = 'keep_clear_starboard'
            heading_bias = math.radians(-15.0)
            speed_scale = 0.75
        elif math.radians(-112.5) <= bearing < math.radians(-15.0):
            encounter = 'crossing_starboard'
            action = 'give_way_starboard'
            heading_bias = math.radians(-20.0)
            speed_scale = 0.65
        elif math.radians(15.0) < bearing <= math.radians(112.5):
            encounter = 'crossing_port'
            action = 'stand_on_monitor'
            speed_scale = 0.90
        elif own_is_being_overtaken:
            encounter = 'being_overtaken'
            action = 'stand_on_monitor'
            speed_scale = 0.90
        else:
            encounter = 'close_quarters'
            action = 'give_way_starboard'
            heading_bias = math.radians(-20.0)
            speed_scale = 0.60
    return EncounterAssessment(
        track_id=target.track_id,
        encounter=encounter,
        action=action,
        range_m=range_m,
        relative_bearing=bearing,
        tcpa_s=tcpa,
        dcpa_m=dcpa,
        risk=risk,
        heading_bias=heading_bias,
        speed_scale=speed_scale,
    )


def select_most_urgent(
    assessments: Sequence[EncounterAssessment],
) -> Optional[EncounterAssessment]:
    risky = [assessment for assessment in assessments if assessment.risk]
    if not risky:
        return None
    return min(
        risky,
        key=lambda assessment: (
            0 if assessment.action.startswith(('give_way', 'keep_clear')) else 1,
            assessment.tcpa_s if assessment.tcpa_s is not None else math.inf,
            assessment.dcpa_m if assessment.dcpa_m is not None else math.inf,
        ),
    )
