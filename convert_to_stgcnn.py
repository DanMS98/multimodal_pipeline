import json
import numpy as np
import pickle
from collections import defaultdict
import argparse


def convert(json_path: str, output_pkl_path: str):
    with open(json_path, 'r') as f:
        detections = json.load(f)


    frame_map = defaultdict(dict)
    for det in detections:
        frame = det['frame_id']
        pid = det['id']
        pos = [det['x'], det['y']]
        frame_map[frame][pid] = pos

    sorted_frames = sorted(frame_map.keys())
    sequences = []
    frame_ids_list = []
    ped_ids_list = []

    for i in range(len(sorted_frames) - 19):  
        frame_ids = sorted_frames[i:i+20]

        common_ids = set(frame_map[frame_ids[0]].keys())
        for fid in frame_ids[1:]:
            common_ids &= frame_map[fid].keys()

        if not common_ids:
            continue

        common_ids = sorted(list(common_ids))
        T, N = 20, len(common_ids)
        seq = np.zeros((T, N, 2))  

        for t, fid in enumerate(frame_ids):
            for n, pid in enumerate(common_ids):
                seq[t, n] = frame_map[fid][pid]

        sequences.append(seq)
        frame_ids_list.append(frame_ids)
        ped_ids_list.append(common_ids)

    data_dict = {
        'data': sequences,
        'frame_ids': frame_ids_list,
        'ped_ids': ped_ids_list
    }

    with open(output_pkl_path, 'wb') as f:
        pickle.dump(data_dict, f)

    print(f"[SUCCESS] Saved {len(sequences)} sequences to {output_pkl_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='raw_detections.json', help='Path to raw_detections.json')
    parser.add_argument('--output', default='stgcnn_input.pkl', help='Output path for the .pkl file')
    args = parser.parse_args()

    convert(args.input, args.output)
