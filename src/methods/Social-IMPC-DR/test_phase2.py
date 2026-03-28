"""Quick test script for Phase 2 priority-based landing pad simulation."""
import numpy as np
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
src_path = str(Path(__file__).resolve().parents[3] / 'src')
sys.path.insert(0, src_path)

from utils import StandardizedEnvironment
from test import PLAN

# Load everything from the phase2 scenario config
scenario_path = Path(__file__).resolve().parent / 'scenarios' / 'phase2_landing_pad.json'
with open(scenario_path, 'r') as f:
    cfg = json.load(f)

env_type = cfg['env_type']
drones = cfg['drones']
num_moving = cfg['num_moving_drones']

# Test without walls first to verify priority logic
obstacles = []

ini_x_m = [np.array(d['start']) for d in drones]
target_m = [np.array(d['goal']) for d in drones]
ini_v_m = [np.zeros(2) for _ in range(num_moving)]

ini_x = ini_x_m + obstacles
ini_v = ini_v_m + [np.zeros(2) for _ in obstacles]
target = target_m + obstacles

cargo_configs = [
    {'cargo_type': d['cargo_type'], 'time_to_expiry': d['time_to_expiry'], 'patient_acuity': d['patient_acuity']}
    for d in drones
]

print(f"Num agents: {len(ini_x)} ({num_moving} moving + {len(obstacles)} walls)")
for i, c in enumerate(cargo_configs):
    d = np.linalg.norm(ini_x_m[i])
    print(f"  Drone {i}: start={ini_x_m[i]}, dist={d:.2f}, cargo={c['cargo_type']}/{c['patient_acuity']}")

result, agent_list, completion_step = PLAN(
    len(ini_x), ini_x, ini_v, target,
    r_min=cfg['min_radius'], epsilon=cfg['epsilon'], h=cfg['step_size'],
    K=cfg['k_value'], episodes=cfg['max_steps'],
    num_moving_drones=num_moving, wall_collision_multiplier=cfg['wall_collision_multiplier'],
    verbose=cfg.get('verbose', True), env_type=env_type, cargo_configs=cargo_configs,
)

print(f"\nAll done at step {completion_step}")
