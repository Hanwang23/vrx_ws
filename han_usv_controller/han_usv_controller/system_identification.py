"""3-DOF USV data logging and conservative first-order model fitting."""

import argparse
import csv
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class IdentificationSample:
    time_s: float
    east_m: float
    north_m: float
    yaw_rad: float
    surge_mps: float
    sway_mps: float
    yaw_rate_rps: float
    left_thrust: float
    right_thrust: float
    state: str


class IdentificationLogger:
    def __init__(self, path: str, minimum_period: float = 0.1) -> None:
        self.path = Path(path).expanduser().resolve()
        self.minimum_period = max(0.01, float(minimum_period))
        self.last_time: Optional[float] = None
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, sample: IdentificationSample) -> bool:
        if (
            self.last_time is not None
            and sample.time_s - self.last_time < self.minimum_period
        ):
            return False
        values = asdict(sample)
        numeric = [value for value in values.values() if not isinstance(value, str)]
        if not all(math.isfinite(float(value)) for value in numeric):
            return False
        new_file = not self.path.exists() or self.path.stat().st_size == 0
        with self.path.open('a', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values))
            if new_file:
                writer.writeheader()
            writer.writerow(values)
        self.last_time = sample.time_s
        return True


def load_samples(path: str) -> List[IdentificationSample]:
    with Path(path).expanduser().open(newline='', encoding='utf-8') as stream:
        return [
            IdentificationSample(
                time_s=float(row['time_s']),
                east_m=float(row['east_m']),
                north_m=float(row['north_m']),
                yaw_rad=float(row['yaw_rad']),
                surge_mps=float(row['surge_mps']),
                sway_mps=float(row['sway_mps']),
                yaw_rate_rps=float(row['yaw_rate_rps']),
                left_thrust=float(row['left_thrust']),
                right_thrust=float(row['right_thrust']),
                state=row['state'],
            )
            for row in csv.DictReader(stream)
        ]


def _solve_3x3(matrix: Sequence[Sequence[float]], values: Sequence[float]):
    augmented = [list(row) + [float(value)] for row, value in zip(matrix, values)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-9:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * reference
                for value, reference in zip(augmented[row], augmented[column])
            ]
    return tuple(augmented[row][3] for row in range(3))


def fit_first_order_axis(
    samples: Sequence[IdentificationSample],
    state_name: str,
    input_values: Sequence[float],
) -> Dict[str, Optional[float]]:
    """Fit x_dot = a*x + b*u + c with ordinary least squares."""
    rows: List[Tuple[float, float, float]] = []
    outputs: List[float] = []
    for index in range(1, len(samples)):
        dt = samples[index].time_s - samples[index - 1].time_s
        if dt <= 1e-3 or dt > 1.0:
            continue
        previous = float(getattr(samples[index - 1], state_name))
        current = float(getattr(samples[index], state_name))
        control = float(input_values[index - 1])
        if all(math.isfinite(value) for value in (previous, current, control)):
            rows.append((previous, control, 1.0))
            outputs.append((current - previous) / dt)
    if len(rows) < 20:
        return {'sample_count': len(rows), 'valid': False}
    normal = [[0.0] * 3 for _ in range(3)]
    rhs = [0.0] * 3
    for row, output in zip(rows, outputs):
        for i in range(3):
            rhs[i] += row[i] * output
            for j in range(3):
                normal[i][j] += row[i] * row[j]
    solution = _solve_3x3(normal, rhs)
    if solution is None:
        return {'sample_count': len(rows), 'valid': False}
    a, b, c = solution
    predictions = [
        a * row[0] + b * row[1] + c for row in rows]
    output_mean = sum(outputs) / len(outputs)
    residual_sum = sum(
        (actual - predicted) ** 2
        for actual, predicted in zip(outputs, predictions))
    total_sum = sum((actual - output_mean) ** 2 for actual in outputs)
    r_squared = 1.0 - residual_sum / total_sum if total_sum > 1e-12 else 0.0
    input_span = max(row[1] for row in rows) - min(row[1] for row in rows)
    return {
        'sample_count': len(rows),
        'valid': bool(a < -1e-5 and math.isfinite(r_squared)),
        'state_coefficient': a,
        'input_gain': b,
        'bias': c,
        'time_constant_s': -1.0 / a if a < -1e-5 else None,
        'steady_state_gain': -b / a if a < -1e-5 else None,
        'r_squared': r_squared,
        'input_span': input_span,
    }


def fit_three_dof_model(samples: Sequence[IdentificationSample]) -> Dict[str, object]:
    average_thrust = [
        0.5 * (sample.left_thrust + sample.right_thrust)
        for sample in samples]
    differential_thrust = [
        0.5 * (sample.right_thrust - sample.left_thrust)
        for sample in samples]
    surge = fit_first_order_axis(samples, 'surge_mps', average_thrust)
    yaw = fit_first_order_axis(samples, 'yaw_rate_rps', differential_thrust)
    valid = bool(
        surge.get('valid') and yaw.get('valid')
        and float(surge.get('r_squared') or 0.0) >= 0.2
        and float(yaw.get('r_squared') or 0.0) >= 0.2
        and float(surge.get('input_span') or 0.0) >= 100.0
        and float(yaw.get('input_span') or 0.0) >= 40.0
    )
    return {
        'model': 'planar_3dof_first_order_v1',
        'sample_count': len(samples),
        'surge': surge,
        'yaw': yaw,
        'nmpc_ready': valid,
        'fallback_controller': 'ilos_pid',
        'readiness_issues': [] if valid else [
            'collect persistently exciting surge and differential-thrust data',
            'require stable surge and yaw fits with R^2 >= 0.2',
        ],
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('input_csv')
    parser.add_argument('--output', default='han_usv_controller/model_data/fitted_model.json')
    args = parser.parse_args(argv)
    result = fit_three_dof_model(load_samples(args.input_csv))
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, separators=(',', ':')))
    return 0 if result['nmpc_ready'] else 2
