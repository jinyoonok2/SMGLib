import csv
from pathlib import Path


TRACK_DIRS = {
    "yield_control": "yield_control",
    "trajectory_planner": "trajectory_planner",
}


def metrics_dir(track_name=None):
    root_dir = Path(__file__).resolve().parents[3]
    output_dir = root_dir / "logs" / "Social-IMPC-DR" / "metrics"
    if track_name in TRACK_DIRS:
        output_dir = output_dir / TRACK_DIRS[track_name]
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def write_run_metrics(
    *,
    track_name,
    trajectory_mode,
    scenario_config,
    config_path,
    num_moving_drones,
    max_steps,
    completion_step,
    runtime_seconds,
):
    test_name = None
    if scenario_config and scenario_config.get("test_name"):
        test_name = scenario_config["test_name"]
    elif config_path is not None:
        test_name = Path(config_path).stem
    else:
        test_name = f"{track_name or 'interactive'}_{trajectory_mode or 'default'}"

    output_path = metrics_dir(track_name) / f"{test_name}_metrics.csv"
    row = {
        "test_name": test_name,
        "track": track_name or "",
        "trajectory_mode": trajectory_mode or "",
        "config_file": str(config_path) if config_path is not None else "",
        "num_moving_drones": num_moving_drones,
        "max_steps": max_steps,
        "completion_step": completion_step,
        "used_full_step_budget": completion_step >= max_steps,
        "runtime_seconds": f"{runtime_seconds:.6f}",
        "runtime_minutes": f"{runtime_seconds / 60.0:.6f}",
    }

    with open(output_path, mode="w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)

    return output_path
