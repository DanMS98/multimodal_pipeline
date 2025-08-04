import json
import h5py
import numpy as np
from collections import defaultdict

json_file = 'raw_detections.json'
obs_len = 8
pred_len = 12
total_len = obs_len + pred_len

with open(json_file, 'r') as f:
    data = json.load(f)

ped_data = defaultdict(list)
for entry in data:
    ped_data[entry['id']].append(entry)

samples = []

for pid, traj in ped_data.items():
    if len(traj) < total_len:
        continue
    traj = sorted(traj, key=lambda x: x['frame_id'])

    for i in range(len(traj) - total_len + 1):
        segment = traj[i:i + total_len]
        positions = np.array([[f['x'], f['y']] for f in segment])
        obs = positions[:obs_len]
        pred = positions[obs_len:]

        sample = {
            'obj_trajs': obs[None, :, :],  
            'obj_trajs_mask': np.ones((1, obs_len), dtype=bool),
            'center_gt_trajs': pred,       
            'center_gt_mask': np.ones(pred_len, dtype=bool),
            'center_obj_id': 0            
        }
        samples.append(sample)

with h5py.File('my_custom_data.h5', 'w') as f:
    for i, sample in enumerate(samples):
        grp = f.create_group(f'sample_{i}')
        grp.create_dataset('obj_trajs', data=sample['obj_trajs'])
        grp.create_dataset('obj_trajs_mask', data=sample['obj_trajs_mask'])
        grp.create_dataset('center_gt_trajs', data=sample['center_gt_trajs'])
        grp.create_dataset('center_gt_mask', data=sample['center_gt_mask'])
        grp.attrs['center_obj_id'] = sample['center_obj_id']
