"""Rolling local ENU occupancy grid with inverse laser sensor updates."""

from dataclasses import dataclass
import math
from typing import Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class OccupancyGridConfig:
    width_m: float = 100.0
    height_m: float = 100.0
    resolution: float = 0.5
    max_range: float = 40.0
    hit_log_odds: float = 0.85
    miss_log_odds: float = -0.40
    min_log_odds: float = -4.0
    max_log_odds: float = 4.0
    decay_rate: float = 0.08
    stale_after: float = 8.0
    occupied_probability: float = 0.65
    ray_stride: int = 2


@dataclass(frozen=True)
class OccupancySnapshot:
    origin_east: float
    origin_north: float
    resolution: float
    width: int
    height: int
    probabilities: Tuple[int, ...]
    blocked: Tuple[bool, ...]
    revision: int

    def world_to_cell(self, east: float, north: float) -> Optional[Tuple[int, int]]:
        column = int(math.floor((east - self.origin_east) / self.resolution))
        row = int(math.floor((north - self.origin_north) / self.resolution))
        if 0 <= column < self.width and 0 <= row < self.height:
            return column, row
        return None

    def is_blocked(self, east: float, north: float) -> bool:
        cell = self.world_to_cell(east, north)
        if cell is None:
            return True
        return self.blocked[cell[1] * self.width + cell[0]]


def enu_grid_origin_in_body(
    origin_east: float,
    origin_north: float,
    vessel_east: float,
    vessel_north: float,
    vessel_yaw: float,
) -> Tuple[float, float, float]:
    """Express a fixed-ENU grid origin and orientation in the vessel frame."""
    delta_east = origin_east - vessel_east
    delta_north = origin_north - vessel_north
    cosine = math.cos(vessel_yaw)
    sine = math.sin(vessel_yaw)
    return (
        delta_east * cosine + delta_north * sine,
        -delta_east * sine + delta_north * cosine,
        -vessel_yaw,
    )


def _bresenham(
    start: Tuple[int, int], end: Tuple[int, int],
) -> Iterable[Tuple[int, int]]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0)
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy


class RollingOccupancyGrid:
    """Probability grid that translates in whole-cell increments with the boat."""

    def __init__(self, config: OccupancyGridConfig = OccupancyGridConfig()) -> None:
        if config.resolution <= 0.0:
            raise ValueError('grid resolution must be positive')
        if config.width_m <= 0.0 or config.height_m <= 0.0:
            raise ValueError('grid dimensions must be positive')
        self.config = config
        self.width = max(1, int(math.ceil(config.width_m / config.resolution)))
        self.height = max(1, int(math.ceil(config.height_m / config.resolution)))
        self.origin_east: Optional[float] = None
        self.origin_north: Optional[float] = None
        size = self.width * self.height
        self.log_odds = [0.0] * size
        self.last_observed = [-math.inf] * size
        self.last_decay: Optional[float] = None
        self.revision = 0

    def reset(self) -> None:
        """Discard all spatial evidence when a new task session starts."""
        size = self.width * self.height
        self.origin_east = None
        self.origin_north = None
        self.log_odds = [0.0] * size
        self.last_observed = [-math.inf] * size
        self.last_decay = None
        self.revision = 0

    def _index(self, column: int, row: int) -> int:
        return row * self.width + column

    def world_to_cell(self, east: float, north: float) -> Optional[Tuple[int, int]]:
        if self.origin_east is None or self.origin_north is None:
            return None
        column = int(math.floor((east - self.origin_east) / self.config.resolution))
        row = int(math.floor((north - self.origin_north) / self.config.resolution))
        if 0 <= column < self.width and 0 <= row < self.height:
            return column, row
        return None

    def _desired_origin(self, center: float, cells: int) -> float:
        aligned_center = math.floor(center / self.config.resolution)
        return (aligned_center - cells // 2) * self.config.resolution

    def recenter(self, east: float, north: float) -> None:
        desired_east = self._desired_origin(east, self.width)
        desired_north = self._desired_origin(north, self.height)
        if self.origin_east is None or self.origin_north is None:
            self.origin_east = desired_east
            self.origin_north = desired_north
            return
        shift_columns = int(round(
            (desired_east - self.origin_east) / self.config.resolution))
        shift_rows = int(round(
            (desired_north - self.origin_north) / self.config.resolution))
        if shift_columns == 0 and shift_rows == 0:
            return
        size = self.width * self.height
        shifted_odds = [0.0] * size
        shifted_times = [-math.inf] * size
        for new_row in range(self.height):
            old_row = new_row + shift_rows
            if not 0 <= old_row < self.height:
                continue
            for new_column in range(self.width):
                old_column = new_column + shift_columns
                if not 0 <= old_column < self.width:
                    continue
                new_index = self._index(new_column, new_row)
                old_index = self._index(old_column, old_row)
                shifted_odds[new_index] = self.log_odds[old_index]
                shifted_times[new_index] = self.last_observed[old_index]
        self.log_odds = shifted_odds
        self.last_observed = shifted_times
        self.origin_east = desired_east
        self.origin_north = desired_north
        self.revision += 1

    def _decay(self, now: float) -> None:
        if self.last_decay is None:
            self.last_decay = now
            return
        elapsed = now - self.last_decay
        if elapsed <= 0.0:
            return
        factor = math.exp(-self.config.decay_rate * elapsed)
        for index, value in enumerate(self.log_odds):
            if value != 0.0:
                self.log_odds[index] = value * factor
        self.last_decay = now

    def _apply(self, cell: Tuple[int, int], increment: float, now: float) -> None:
        index = self._index(*cell)
        self.log_odds[index] = max(
            self.config.min_log_odds,
            min(self.config.max_log_odds, self.log_odds[index] + increment),
        )
        self.last_observed[index] = now

    def update_scan(
        self,
        sensor_east: float,
        sensor_north: float,
        yaw: float,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        now: float,
    ) -> None:
        if not ranges or angle_increment <= 0.0:
            return
        self.recenter(sensor_east, sensor_north)
        self._decay(now)
        start = self.world_to_cell(sensor_east, sensor_north)
        if start is None:
            return
        effective_max = min(
            self.config.max_range,
            range_max if math.isfinite(range_max) and range_max > 0.0
            else self.config.max_range,
        )
        stride = max(1, self.config.ray_stride)
        free_updates = set()
        occupied_updates = set()
        for index in range(0, len(ranges), stride):
            try:
                raw_range = float(ranges[index])
            except (TypeError, ValueError, OverflowError):
                continue
            # NaN is an invalid/no-observation beam. It is also used to keep
            # confirmed moving targets out of the static occupancy layer.
            if math.isnan(raw_range) or raw_range == -math.inf:
                continue
            if math.isfinite(raw_range) and raw_range < range_min:
                continue
            hit = (
                math.isfinite(raw_range)
                and range_min <= raw_range < effective_max - self.config.resolution
            )
            if math.isfinite(raw_range):
                distance = min(raw_range, effective_max)
            else:
                distance = effective_max
            if distance < range_min:
                continue
            angle = yaw + angle_min + index * angle_increment
            endpoint = self.world_to_cell(
                sensor_east + distance * math.cos(angle),
                sensor_north + distance * math.sin(angle),
            )
            if endpoint is None:
                boundary_distance = effective_max
                for candidate_distance in (
                    effective_max * value / 20.0 for value in range(19, 0, -1)
                ):
                    candidate = self.world_to_cell(
                        sensor_east + candidate_distance * math.cos(angle),
                        sensor_north + candidate_distance * math.sin(angle),
                    )
                    if candidate is not None:
                        endpoint = candidate
                        boundary_distance = candidate_distance
                        break
                if endpoint is None:
                    continue
                if boundary_distance + self.config.resolution < distance:
                    hit = False
            cells = list(_bresenham(start, endpoint))
            free_cells = cells[:-1] if hit else cells
            free_updates.update(free_cells)
            if hit:
                occupied_updates.add(cells[-1])
        for cell in free_updates - occupied_updates:
            self._apply(cell, self.config.miss_log_odds, now)
        for cell in occupied_updates:
            self._apply(cell, self.config.hit_log_odds, now)
        self.revision += 1

    def update_obstacles(
        self,
        center_east: float,
        center_north: float,
        points: Sequence[Tuple[float, float]],
        now: float,
    ) -> None:
        """Fuse confirmed world-frame point tracks as occupied evidence."""
        if not points:
            return
        self.recenter(center_east, center_north)
        self._decay(now)
        updated = False
        for east, north in points:
            if not math.isfinite(east) or not math.isfinite(north):
                continue
            cell = self.world_to_cell(east, north)
            if cell is None:
                continue
            index = self._index(*cell)
            self.log_odds[index] = max(
                self.log_odds[index],
                min(
                    self.config.max_log_odds,
                    2.0 * self.config.hit_log_odds,
                ),
            )
            self.last_observed[index] = now
            updated = True
        if updated:
            self.revision += 1

    def snapshot(self, now: float, inflation_radius: float = 0.0) -> OccupancySnapshot:
        self._decay(now)
        if self.origin_east is None or self.origin_north is None:
            raise RuntimeError('grid has not been centered')
        occupied_log_odds = math.log(
            self.config.occupied_probability
            / (1.0 - self.config.occupied_probability))
        probabilities: List[int] = []
        occupied = [False] * (self.width * self.height)
        for index, value in enumerate(self.log_odds):
            if now - self.last_observed[index] > self.config.stale_after:
                probabilities.append(-1)
                continue
            probability = 1.0 / (1.0 + math.exp(-value))
            probabilities.append(int(round(100.0 * probability)))
            occupied[index] = value >= occupied_log_odds

        blocked = occupied[:]
        inflation_cells = int(math.ceil(
            max(0.0, inflation_radius) / self.config.resolution))
        if inflation_cells > 0:
            offsets = [
                (dx, dy)
                for dy in range(-inflation_cells, inflation_cells + 1)
                for dx in range(-inflation_cells, inflation_cells + 1)
                if math.hypot(dx, dy) <= inflation_cells
            ]
            for index, is_occupied in enumerate(occupied):
                if not is_occupied:
                    continue
                column = index % self.width
                row = index // self.width
                for dx, dy in offsets:
                    target_column = column + dx
                    target_row = row + dy
                    if (
                        0 <= target_column < self.width
                        and 0 <= target_row < self.height
                    ):
                        blocked[self._index(target_column, target_row)] = True
        return OccupancySnapshot(
            origin_east=self.origin_east,
            origin_north=self.origin_north,
            resolution=self.config.resolution,
            width=self.width,
            height=self.height,
            probabilities=tuple(probabilities),
            blocked=tuple(blocked),
            revision=self.revision,
        )
