"""Forward-only Dubins state-lattice A* over an inflated occupancy grid."""

from dataclasses import dataclass
import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

from .guidance import DubinsPath, plan_dubins_path
from .occupancy_grid import OccupancySnapshot


Pose = Tuple[float, float, float]
StateKey = Tuple[int, int, int, bool]


@dataclass(frozen=True)
class StateLatticeConfig:
    turn_radius: float = 8.0
    sample_step: float = 0.5
    heading_bins: int = 16
    planning_horizon: float = 40.0
    analytic_expansion_distance: float = 12.0
    max_expansions: int = 2500
    turn_penalty: float = 0.05
    start_clearance_radius: float = 4.0


@dataclass(frozen=True)
class LatticePlan:
    path: DubinsPath
    reached_goal: bool
    expanded_states: int
    used_search: bool
    map_revision: int


def _normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DubinsStateLatticePlanner:
    """A* whose three motion primitives obey the Dubins curvature bound."""

    def __init__(self, config: StateLatticeConfig = StateLatticeConfig()) -> None:
        if config.turn_radius <= 0.0 or config.sample_step <= 0.0:
            raise ValueError('turn radius and sample step must be positive')
        if config.heading_bins < 8:
            raise ValueError('at least eight heading bins are required')
        self.config = config
        self.heading_step = 2.0 * math.pi / config.heading_bins
        self.primitive_length = config.turn_radius * self.heading_step

    def _key(
        self,
        pose: Pose,
        grid: OccupancySnapshot,
        clearance_active: bool = False,
    ) -> Optional[StateKey]:
        cell = grid.world_to_cell(pose[0], pose[1])
        if cell is None:
            return None
        heading = int(round(_normalize(pose[2]) / self.heading_step))
        return (
            cell[0], cell[1], heading % self.config.heading_bins,
            bool(clearance_active))

    def _primitive(self, pose: Pose, curvature: float):
        steps = max(1, int(math.ceil(
            self.primitive_length / self.config.sample_step)))
        distance = self.primitive_length / steps
        x, y, yaw = pose
        points: List[Pose] = []
        curvatures: List[float] = []
        for _ in range(steps):
            if abs(curvature) < 1e-12:
                x += distance * math.cos(yaw)
                y += distance * math.sin(yaw)
            else:
                new_yaw = yaw + curvature * distance
                x += (math.sin(new_yaw) - math.sin(yaw)) / curvature
                y += (-math.cos(new_yaw) + math.cos(yaw)) / curvature
                yaw = new_yaw
            yaw = _normalize(yaw)
            points.append((x, y, yaw))
            curvatures.append(curvature)
        return (x, y, yaw), tuple(points), tuple(curvatures)

    def _collision_free(
        self,
        points: Sequence[Pose],
        grid: OccupancySnapshot,
        clearance_center: Optional[Pose] = None,
        clearance_active: Optional[bool] = None,
    ) -> bool:
        if clearance_active is None:
            clearance_active = bool(
                clearance_center is not None
                and self.config.start_clearance_radius > 0.0
                and grid.is_blocked(clearance_center[0], clearance_center[1]))
        collision_free, _ = self._collision_check(
            points, grid, clearance_center, clearance_active)
        return collision_free

    def _collision_check(
        self,
        points: Sequence[Pose],
        grid: OccupancySnapshot,
        clearance_center: Optional[Pose],
        clearance_active: bool,
    ) -> Tuple[bool, bool]:
        for point in points:
            blocked = grid.is_blocked(point[0], point[1])
            inside_clearance = bool(
                clearance_center is not None
                and math.hypot(
                    point[0] - clearance_center[0],
                    point[1] - clearance_center[1],
                ) <= self.config.start_clearance_radius
            )
            if clearance_active and inside_clearance:
                if not blocked:
                    clearance_active = False
                continue
            clearance_active = False
            if blocked:
                return False, False
        return True, clearance_active

    def _limited_goal(self, start: Pose, goal: Pose) -> Tuple[Pose, bool]:
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        distance = math.hypot(dx, dy)
        if distance <= self.config.planning_horizon:
            return goal, True
        ratio = self.config.planning_horizon / distance
        bearing = math.atan2(dy, dx)
        return (
            start[0] + ratio * dx,
            start[1] + ratio * dy,
            bearing,
        ), False

    def _heuristic(self, pose: Pose, goal: Pose) -> float:
        try:
            return plan_dubins_path(
                pose, goal, self.config.turn_radius,
                sample_step=max(1.0, self.config.sample_step),
            ).total_length
        except ValueError:
            return math.hypot(goal[0] - pose[0], goal[1] - pose[1])

    def _analytic_connection(
        self,
        start: Pose,
        goal: Pose,
        grid: OccupancySnapshot,
        clearance_center: Optional[Pose] = None,
        clearance_active: Optional[bool] = None,
    ) -> Optional[DubinsPath]:
        try:
            path = plan_dubins_path(
                start,
                goal,
                self.config.turn_radius,
                self.config.sample_step,
                allow_three_turn_paths=False,
            )
        except ValueError:
            return None
        return path if self._collision_free(
            path.points, grid, clearance_center, clearance_active) else None

    def plan(
        self, start: Pose, goal: Pose, grid: OccupancySnapshot,
    ) -> Optional[LatticePlan]:
        limited_goal, reached_goal = self._limited_goal(start, goal)
        if grid.world_to_cell(limited_goal[0], limited_goal[1]) is None:
            return None

        start_clearance_active = bool(
            self.config.start_clearance_radius > 0.0
            and grid.is_blocked(start[0], start[1]))
        direct = self._analytic_connection(
            start,
            limited_goal,
            grid,
            clearance_center=start,
            clearance_active=start_clearance_active,
        )
        if direct is not None:
            return LatticePlan(
                direct, reached_goal, 0, False, grid.revision)

        start_key = self._key(start, grid, start_clearance_active)
        if start_key is None:
            return None
        queue = [(self._heuristic(start, limited_goal), 0.0, 0, start_key)]
        sequence = 0
        costs: Dict[StateKey, float] = {start_key: 0.0}
        poses: Dict[StateKey, Pose] = {start_key: start}
        parents: Dict[
            StateKey,
            Tuple[StateKey, Tuple[Pose, ...], Tuple[float, ...], str],
        ] = {}
        expanded = 0
        goal_key: Optional[StateKey] = None
        connector: Optional[DubinsPath] = None

        while queue and expanded < self.config.max_expansions:
            _, queued_cost, _, key = heapq.heappop(queue)
            if queued_cost > costs.get(key, math.inf) + 1e-9:
                continue
            pose = poses[key]
            expanded += 1
            distance_to_goal = math.hypot(
                limited_goal[0] - pose[0], limited_goal[1] - pose[1])
            if distance_to_goal <= self.config.analytic_expansion_distance:
                candidate = self._analytic_connection(
                    pose,
                    limited_goal,
                    grid,
                    clearance_center=start,
                    clearance_active=key[3],
                )
                if candidate is not None:
                    goal_key = key
                    connector = candidate
                    break

            for mode, curvature in (
                ('L', 1.0 / self.config.turn_radius),
                ('S', 0.0),
                ('R', -1.0 / self.config.turn_radius),
            ):
                endpoint, points, curvatures = self._primitive(pose, curvature)
                collision_free, next_clearance_active = self._collision_check(
                    points, grid, start, key[3])
                if not collision_free:
                    continue
                neighbor = self._key(
                    endpoint, grid, next_clearance_active)
                if neighbor is None:
                    continue
                step_cost = self.primitive_length * (
                    1.0 + (self.config.turn_penalty if mode != 'S' else 0.0))
                tentative = queued_cost + step_cost
                if tentative >= costs.get(neighbor, math.inf) - 1e-9:
                    continue
                costs[neighbor] = tentative
                poses[neighbor] = endpoint
                parents[neighbor] = (key, points, curvatures, mode)
                sequence += 1
                heapq.heappush(queue, (
                    tentative + self._heuristic(endpoint, limited_goal),
                    tentative,
                    sequence,
                    neighbor,
                ))

        if goal_key is None or connector is None:
            return None
        segments = []
        cursor = goal_key
        while cursor != start_key:
            parent, points, curvatures, mode = parents[cursor]
            segments.append((points, curvatures, mode))
            cursor = parent
        segments.reverse()
        path_points: List[Pose] = [start]
        path_curvatures: List[float] = []
        modes: List[str] = []
        lengths: List[float] = []
        for points, curvatures, mode in segments:
            path_points.extend(points)
            path_curvatures.extend(curvatures)
            modes.append(mode)
            lengths.append(self.primitive_length)
        path_points.extend(connector.points[1:])
        path_curvatures.extend(connector.curvatures)
        modes.extend(connector.modes)
        lengths.extend(connector.segment_lengths)
        path = DubinsPath(
            modes=tuple(modes),
            segment_lengths=tuple(lengths),
            points=tuple(path_points),
            curvatures=tuple(path_curvatures),
        )
        return LatticePlan(path, reached_goal, expanded, True, grid.revision)
