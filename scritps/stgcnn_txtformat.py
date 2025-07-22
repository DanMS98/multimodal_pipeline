import json
import random

input_json_path = "raw_detections.json"
train_txt_path = "train.txt"
val_txt_path = "val.txt"
train_split = 0.8
random_seed = 42

with open(input_json_path, "r") as f:
    detections = json.load(f)

ped_trajectories = {}
for det in detections:
    ped_id = det["id"]
    if ped_id not in ped_trajectories:
        ped_trajectories[ped_id] = []
    ped_trajectories[ped_id].append((float(det["frame_id"]), float(det["x"]), float(det["y"])))

random.seed(random_seed)
ped_ids = list(ped_trajectories.keys())
random.shuffle(ped_ids)

split_idx = int(len(ped_ids) * train_split)
train_ids = set(ped_ids[:split_idx])
val_ids = set(ped_ids[split_idx:])

with open(train_txt_path, "w") as train_file, open(val_txt_path, "w") as val_file:
    for ped_id, traj in ped_trajectories.items():
        for frame_id, x, y in traj:
            line = f"{frame_id:.1f}\t{float(ped_id):.1f}\t{x:.2f}\t{y:.2f}\n"
            if ped_id in train_ids:
                train_file.write(line)
            else:
                val_file.write(line)
print(f'saved {len(train_ids)} pedestrians to {train_txt_path} and {len(val_ids)} pedestrians to {val_txt_path}')
