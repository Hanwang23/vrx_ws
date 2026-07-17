"""Dubins path planning and integral line-of-sight path following."""

from dataclasses import dataclass
import math
from typing import Callable, List, Optional, Sequence, Tuple


def _mod2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class DubinsPath:
    modes: Tuple[str, str, str]
    segment_lengths: Tuple[float, float, float]
    points: Tuple[Tuple[float, float, float], ...]
    curvatures: Tuple[float, ...]

    @property
    def total_length(self) -> float:
        return sum(self.segment_lengths)


@dataclass(frozen=True)
class ILOSOutput:
    course: float
    path_course: float
    path_curvature: float
    upcoming_curvature: float
    cross_track_error: float
    segment_index: int
    along_track: float
    remaining_distance: float
    integral_bias: float
    projection: Tuple[float, float]


DubinsSolver = Callable[
    [float, float, float], Optional[Tuple[float, float, float]]]


def _dubins_candidates(alpha: float, beta: float, distance: float):
    sin_alpha = math.sin(alpha)
    sin_beta = math.sin(beta)
    cos_alpha = math.cos(alpha)
    cos_beta = math.cos(beta)
    cos_delta = math.cos(alpha - beta)

    def lsl(_a: float, _b: float, d: float):
        p_squared = (
            2.0 + d * d - 2.0 * cos_delta
            + 2.0 * d * (sin_alpha - sin_beta)
        )
        if p_squared < -1e-12:
            return None
        temporary = math.atan2(
            cos_beta - cos_alpha,
            d + sin_alpha - sin_beta,
        )
        return (
            _mod2pi(-alpha + temporary),
            math.sqrt(max(0.0, p_squared)),
            _mod2pi(beta - temporary),
        )

    def rsr(_a: float, _b: float, d: float):
        p_squared = (
            2.0 + d * d - 2.0 * cos_delta
            + 2.0 * d * (sin_beta - sin_alpha)
        )
        if p_squared < -1e-12:
            return None
        temporary = math.atan2(
            cos_alpha - cos_beta,
            d - sin_alpha + sin_beta,
        )
        return (
            _mod2pi(alpha - temporary),
            math.sqrt(max(0.0, p_squared)),
            _mod2pi(-beta + temporary),
        )

    def lsr(_a: float, _b: float, d: float):
        p_squared = (
            -2.0 + d * d + 2.0 * cos_delta
            + 2.0 * d * (sin_alpha + sin_beta)
        )
        if p_squared < -1e-12:
            return None
        p = math.sqrt(max(0.0, p_squared))
        temporary = (
            math.atan2(
                -cos_alpha - cos_beta,
                d + sin_alpha + sin_beta,
            )
            - math.atan2(-2.0, p)
        )
        return (
            _mod2pi(-alpha + temporary),
            p,
            _mod2pi(-beta + temporary),
        )

    def rsl(_a: float, _b: float, d: float):
        p_squared = (
            d * d - 2.0 + 2.0 * cos_delta
            - 2.0 * d * (sin_alpha + sin_beta)
        )
        if p_squared < -1e-12:
            return None
        p = math.sqrt(max(0.0, p_squared))
        temporary = (
            math.atan2(
                cos_alpha + cos_beta,
                d - sin_alpha - sin_beta,
            )
            - math.atan2(2.0, p)
        )
        return (
            _mod2pi(alpha - temporary),
            p,
            _mod2pi(beta - temporary),
        )

    def rlr(_a: float, _b: float, d: float):
        temporary = (
            6.0 - d * d + 2.0 * cos_delta
            + 2.0 * d * (sin_alpha - sin_beta)
        ) / 8.0
        if abs(temporary) > 1.0 + 1e-12:
            return None
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, temporary))))
        t = _mod2pi(
            alpha
            - math.atan2(
                cos_alpha - cos_beta,
                d - sin_alpha + sin_beta,
            )
            + 0.5 * p
        )
        return t, p, _mod2pi(alpha - beta - t + p)

    def lrl(_a: float, _b: float, d: float):
        temporary = (
            6.0 - d * d + 2.0 * cos_delta
            + 2.0 * d * (-sin_alpha + sin_beta)
        ) / 8.0
        if abs(temporary) > 1.0 + 1e-12:
            return None
        p = _mod2pi(2.0 * math.pi - math.acos(max(-1.0, min(1.0, temporary))))
        t = _mod2pi(
            -alpha
            - math.atan2(
                cos_alpha - cos_beta,
                d + sin_alpha - sin_beta,
            )
            + 0.5 * p
        )
        return t, p, _mod2pi(beta - alpha - t + p)

    return (
        (('L', 'S', 'L'), lsl),
        (('R', 'S', 'R'), rsr),
        (('L', 'S', 'R'), lsr),
        (('R', 'S', 'L'), rsl),
        (('R', 'L', 'R'), rlr),
        (('L', 'R', 'L'), lrl),
    )


def _advance_pose(
    x: float,
    y: float,
    yaw: float,
    mode: str,
    distance: float,
    radius: float,
) -> Tuple[float, float, float]:
    if mode == 'S':
        return (
            x + distance * math.cos(yaw),
            y + distance * math.sin(yaw),
            yaw,
        )
    direction = 1.0 if mode == 'L' else -1.0
    next_yaw = yaw + direction * distance / radius
    if mode == 'L':
        next_x = x + radius * (math.sin(next_yaw) - math.sin(yaw))
        next_y = y + radius * (math.cos(yaw) - math.cos(next_yaw))
    else:
        next_x = x + radius * (math.sin(yaw) - math.sin(next_yaw))
        next_y = y + radius * (math.cos(next_yaw) - math.cos(yaw))
    return next_x, next_y, _normalize_angle(next_yaw)


def plan_dubins_path(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    turn_radius: float,
    sample_step: float = 1.0,
    allow_three_turn_paths: bool = True,
) -> DubinsPath:
    """Return the shortest forward-only bounded-curvature Dubins path."""
    radius = float(turn_radius)
    step = float(sample_step)
    start_x, start_y, start_yaw = map(float, start)
    goal_x, goal_y, goal_yaw = map(float, goal)
    if not all(math.isfinite(value) for value in (
        start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, radius, step,
    )):
        raise ValueError('Dubins inputs must be finite')
    if radius <= 0.0 or step <= 0.0:
        raise ValueError('Dubins turn radius and sample step must be positive')
    if (
        math.hypot(goal_x - start_x, goal_y - start_y) <= 1e-12
        and abs(_normalize_angle(goal_yaw - start_yaw)) <= 1e-12
    ):
        pose = (start_x, start_y, _normalize_angle(start_yaw))
        return DubinsPath(
            ('S', 'S', 'S'), (0.0, 0.0, 0.0), (pose,), ())
    delta_x = goal_x - start_x
    delta_y = goal_y - start_y
    normalized_distance = math.hypot(delta_x, delta_y) / radius
    direction = math.atan2(delta_y, delta_x)
    alpha = _mod2pi(start_yaw - direction)
    beta = _mod2pi(goal_yaw - direction)

    best_modes: Optional[Tuple[str, str, str]] = None
    best_parameters: Optional[Tuple[float, float, float]] = None
    best_cost = math.inf
    for modes, solver in _dubins_candidates(alpha, beta, normalized_distance):
        if not allow_three_turn_paths and modes in (
            ('R', 'L', 'R'), ('L', 'R', 'L')
        ):
            continue
        parameters = solver(alpha, beta, normalized_distance)
        if parameters is None:
            continue
        cost = sum(parameters)
        if cost < best_cost:
            best_cost = cost
            best_modes = modes
            best_parameters = parameters
    if best_modes is None or best_parameters is None:
        raise ValueError('No finite Dubins path found')

    lengths = tuple(parameter * radius for parameter in best_parameters)
    x, y, yaw = start_x, start_y, _normalize_angle(start_yaw)
    points: List[Tuple[float, float, float]] = [(x, y, yaw)]
    curvatures: List[float] = []
    for mode, segment_length in zip(best_modes, lengths):
        remaining = segment_length
        while remaining > 1e-9:
            distance = min(step, remaining)
            x, y, yaw = _advance_pose(x, y, yaw, mode, distance, radius)
            points.append((x, y, yaw))
            curvatures.append(
                1.0 / radius
                if mode == 'L'
                else -1.0 / radius if mode == 'R' else 0.0
            )
            remaining -= distance
    if math.hypot(points[-1][0] - goal_x, points[-1][1] - goal_y) < 1e-5:
        points[-1] = (goal_x, goal_y, _normalize_angle(goal_yaw))
    return DubinsPath(best_modes, lengths, tuple(points), tuple(curvatures))


class ILOSPathFollower:
    """Integral LOS guidance over a sampled continuous path."""

    def __init__(
        self,
        lookahead: float = 8.0,
        integral_gain: float = 0.015,
        integral_limit: float = 3.0,
        correction_limit: float = math.radians(60.0),
    ) -> None:
        self.lookahead = max(0.5, float(lookahead))
        self.integral_gain = max(0.0, float(integral_gain))
        self.integral_limit = max(0.0, float(integral_limit))
        self.correction_limit = max(0.0, abs(float(correction_limit)))
        self.points: Tuple[Tuple[float, float], ...] = ()
        self.curvatures: Tuple[float, ...] = ()
        self.segment_index = 0
        self.integral_bias = 0.0

    def reset(
        self,
        points: Sequence[Tuple[float, float]] = (),
        curvatures: Sequence[float] = (),
    ) -> None:
        self.points = tuple((float(x), float(y)) for x, y in points)
        values = tuple(float(value) for value in curvatures)
        self.curvatures = (
            values if len(values) == max(0, len(self.points) - 1) else ())
        self.segment_index = 0
        self.integral_bias = 0.0

    def preview(
        self,
        x: float,
        y: float,
        return_to_endpoint: bool = True,
    ) -> ILOSOutput:
        if len(self.points) < 2:
            raise ValueError('ILOS requires at least two path points')
        last_segment = len(self.points) - 2
        search_start = max(0, self.segment_index - 2)
        # Bounded forward search prevents a self-near Dubins arc from being
        # mistaken for a much later path section after a local avoidance move.
        search_end = min(last_segment, self.segment_index + 10)
        best = None
        for index in range(search_start, search_end + 1):
            start_x, start_y = self.points[index]
            end_x, end_y = self.points[index + 1]
            delta_x = end_x - start_x
            delta_y = end_y - start_y
            length_squared = delta_x * delta_x + delta_y * delta_y
            if length_squared <= 1e-12:
                continue
            projection = max(0.0, min(1.0, (
                (x - start_x) * delta_x + (y - start_y) * delta_y
            ) / length_squared))
            closest_x = start_x + projection * delta_x
            closest_y = start_y + projection * delta_y
            distance_squared = (x - closest_x) ** 2 + (y - closest_y) ** 2
            candidate = (distance_squared, index, projection)
            if best is None or candidate < best:
                best = candidate
        if best is None:
            raise ValueError('ILOS path contains no valid segment')

        _, index, projection = best
        self.segment_index = max(self.segment_index, index)
        index = self.segment_index
        start_x, start_y = self.points[index]
        end_x, end_y = self.points[index + 1]
        delta_x = end_x - start_x
        delta_y = end_y - start_y
        selected_length_squared = delta_x * delta_x + delta_y * delta_y
        projection = max(0.0, min(1.0, (
            (x - start_x) * delta_x + (y - start_y) * delta_y
        ) / selected_length_squared))
        segment_length = math.hypot(delta_x, delta_y)
        path_course = math.atan2(delta_y, delta_x)
        path_curvature = (
            self.curvatures[index] if self.curvatures else 0.0)
        upcoming_curvature = abs(path_curvature)
        if self.curvatures:
            preview_distance = 0.0
            for preview_index in range(index, len(self.curvatures)):
                preview_start = self.points[preview_index]
                preview_end = self.points[preview_index + 1]
                upcoming_curvature = max(
                    upcoming_curvature,
                    abs(self.curvatures[preview_index]),
                )
                preview_distance += math.hypot(
                    preview_end[0] - preview_start[0],
                    preview_end[1] - preview_start[1],
                )
                if preview_distance > self.lookahead:
                    break
        if not self.curvatures and index < last_segment:
            for next_index in range(index + 1, last_segment + 1):
                next_start_x, next_start_y = self.points[next_index]
                next_end_x, next_end_y = self.points[next_index + 1]
                next_delta_x = next_end_x - next_start_x
                next_delta_y = next_end_y - next_start_y
                next_length = math.hypot(next_delta_x, next_delta_y)
                if next_length <= 1e-9:
                    continue
                next_course = math.atan2(next_delta_y, next_delta_x)
                transition_length = 0.5 * (segment_length + next_length)
                path_curvature = _normalize_angle(
                    next_course - path_course) / max(1e-9, transition_length)
                break
        elif not self.curvatures and index > 0:
            for previous_index in range(index - 1, -1, -1):
                previous_start_x, previous_start_y = self.points[previous_index]
                previous_end_x, previous_end_y = self.points[previous_index + 1]
                previous_delta_x = previous_end_x - previous_start_x
                previous_delta_y = previous_end_y - previous_start_y
                previous_length = math.hypot(previous_delta_x, previous_delta_y)
                if previous_length <= 1e-9:
                    continue
                previous_course = math.atan2(previous_delta_y, previous_delta_x)
                transition_length = 0.5 * (previous_length + segment_length)
                path_curvature = _normalize_angle(
                    path_course - previous_course) / max(1e-9, transition_length)
                break
        cross_track = (
            -math.sin(path_course) * (x - start_x)
            + math.cos(path_course) * (y - start_y)
        )
        correction = max(
            -self.correction_limit,
            min(
                self.correction_limit,
                math.atan2(
                    cross_track + self.integral_bias,
                    self.lookahead,
                ),
            ),
        )
        course = _normalize_angle(path_course - correction)
        projection_x = start_x + projection * delta_x
        projection_y = start_y + projection * delta_y
        remaining_distance = (1.0 - projection) * segment_length
        remaining_distance += sum(
            math.hypot(
                self.points[next_index + 1][0] - self.points[next_index][0],
                self.points[next_index + 1][1] - self.points[next_index][1],
            )
            for next_index in range(index + 1, len(self.points) - 1)
        )
        if (
            return_to_endpoint
            and index == last_segment
            and projection >= 0.95
        ):
            goal_delta_x = self.points[-1][0] - x
            goal_delta_y = self.points[-1][1] - y
            if math.hypot(goal_delta_x, goal_delta_y) > 1e-6:
                course = math.atan2(goal_delta_y, goal_delta_x)
                path_curvature = 0.0
        return ILOSOutput(
            course=course,
            path_course=path_course,
            path_curvature=path_curvature,
            upcoming_curvature=upcoming_curvature,
            cross_track_error=cross_track,
            segment_index=index,
            along_track=projection * segment_length,
            remaining_distance=remaining_distance,
            integral_bias=self.integral_bias,
            projection=(projection_x, projection_y),
        )

    def integrate(self, cross_track_error: float, dt: float, enabled: bool) -> None:
        if not enabled or dt <= 0.0 or not math.isfinite(cross_track_error):
            return
        if abs(cross_track_error) > 2.0 * self.lookahead:
            return
        denominator = (
            self.lookahead * self.lookahead
            + (cross_track_error + self.integral_bias) ** 2
        )
        derivative = (
            self.integral_gain
            * self.lookahead * self.lookahead
            * cross_track_error
            / max(1e-9, denominator)
        )
        self.integral_bias = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral_bias + derivative * dt),
        )
