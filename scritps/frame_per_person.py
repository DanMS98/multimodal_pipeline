import json
from collections import defaultdict

input_json_path = "raw_detections.json"

with open(input_json_path, "r") as f:
    detections = json.load(f)

pedestrian_frames = defaultdict(set)

for det in detections:
    ped_id = det["id"]
    frame_id = det["frame_id"]
    pedestrian_frames[ped_id].add(frame_id)

print("Pedestrian ID -> Number of unique frames:")
for ped_id, frames in sorted(pedestrian_frames.items()):
    print(f"{ped_id:>3} -> {len(frames)}")

print("\n Pedestrians with < 20 frames:")
for ped_id, frames in pedestrian_frames.items():
    if len(frames) < 20:
        print(f"ID {ped_id}: {len(frames)} frames")
