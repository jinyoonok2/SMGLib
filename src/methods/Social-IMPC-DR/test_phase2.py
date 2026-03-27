"""Quick test script for Phase 2 priority-based landing pad simulation."""
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
src_path = str(Path(__file__).resolve().parents[3] / 'src')
sys.path.insert(0, src_path)

from utils import StandardizedEnvironment
from test import PLAN

env_type = 'landing_pad'
# Test without walls first to verify priority logic
obstacles = []
positions = StandardizedEnvironment.get_standard_agent_positions(env_type, 3)

ini_x_m = [np.array(p['start']) for p in positions]
target_m = [np.array(p['goal']) for p in positions]
ini_v_m = [np.zeros(2) for _ in range(3)]

ini_x = ini_x_m + obstacles
ini_v = ini_v_m + [np.zeros(2) for _ in obstacles]
target = target_m + obstacles

cargo_configs = [
    {'cargo_type': 'organ',     'time_to_expiry': 60.0,  'patient_acuity': 'critical'},
    {'cargo_type': 'equipment', 'time_to_expiry': 200.0, 'patient_acuity': 'routine'},
    {'cargo_type': 'medication','time_to_expiry': 150.0, 'patient_acuity': 'urgent'},
]

print(f"Num agents: {len(ini_x)} (3 moving + {len(obstacles)} walls)")
for i, c in enumerate(cargo_configs):
    d = np.linalg.norm(ini_x_m[i])
    ct = c['cargo_type']
    pa = c['patient_acuity']
    print(f"  Drone {i}: start={ini_x_m[i]}, dist={d:.2f}, cargo={ct}/{pa}")

result, agent_list, completion_step = PLAN(
    len(ini_x), ini_x, ini_v, target,
    r_min=0.5, epsilon=0.1, h=0.1, K=10, episodes=200,
    num_moving_drones=3, wall_collision_multiplier=1.5,
    verbose=True, env_type=env_type, cargo_configs=cargo_configs,
)

print(f"\nAll done at step {completion_step}")
