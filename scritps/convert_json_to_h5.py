import os
import pickle
import numpy as np
from collections import defaultdict
import json

# Constants
obs_len = 8
pred_len = 12
total_len = obs_len + pred_len
save_dir = 'nuscenes/nuscenes_0'
os.makedirs(save_dir, exist_ok=True)

# Load your data
with open('raw_detections.json', 'r') as f:
    data = json.load(f)

# Group by pedestrian ID
ped_data = defaultdict(list)
for det in data:
    ped_data[det['id']].append(det)

scene_id = 0
sample_id = 0

for pid, traj in ped_data.items():
    if len(traj) < total_len:
        continue

    traj = sorted(traj, key=lambda x: x['frame_id'])

    for i in range(len(traj) - total_len + 1):
        segment = traj[i:i+total_len]
        positions = np.array([[f['x'], f['y']] for f in segment])

        obs = positions[:obs_len]
        pred = positions[obs_len:]

        sample = {
            'obj_trajs': obs[None, :, :],  # (1, 8, 2)
            'obj_trajs_mask': np.ones((1, obs_len), dtype=bool),
            'center_gt_trajs': pred,       # (12, 2)
            'center_gt_mask': np.ones(pred_len, dtype=bool),
            'center_obj_id': 0,
            'scene_id': scene_id,
            'map': np.zeros((3, 224, 224), dtype=np.uint8),  # dummy image
            'lane_centerline': np.zeros((1, 2), dtype=np.float32),
            'lane_centerline_mask': np.zeros((1,), dtype=bool)
        }

        # Save to file
        out_path = os.path.join(save_dir, f"sample_{sample_id}.pkl")
        with open(out_path, 'wb') as f:
            pickle.dump(sample, f)

        sample_id += 1
    scene_id += 1

print(f"[✓] Saved {sample_id} NuScenes-format samples to {save_dir}")
