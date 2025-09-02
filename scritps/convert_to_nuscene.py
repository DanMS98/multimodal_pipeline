#!/usr/bin/env python3
"""
Convert your per-detection JSON into NuScenes-style CSV rows and split into train/val/test.

Input record schema (per detection):
{
  "frame_id": int,
  "id": int,              # track id
  "x": float, "y": float, "z": float,
  "dx": float, "dy": float, "dz": float,
  "yaw": float, "score": float, "label": any  # ignored for classing; we force single-class 'pedestrian'
}

Output CSV columns (strings containing JSON):
- cam_token              -> list[str],   len = obs_len + pred_len
- ann_token              -> list[str],   len = obs_len + pred_len
- bounding_box           -> list[[x,y,z,w,l,h]], len = obs_len
- future_bounding_box    -> list[[x,y,z,w,l,h]], len = pred_len
- K                      -> 3x3 intrinsics matrix (list[list[float]])
- attribute              -> list[str] (all "pedestrian"), len = obs_len + pred_len
- label                  -> list[[1]], single-class one-hot per obs step
- future_label           -> list[[1]], single-class one-hot per pred step

Notes:
- We generate stable (deterministic) tokens if you don't pass token maps.
- K defaults to fx,fy,cx,cy unless you pass a JSON.
- Splits are **by track_id** to avoid leakage.
"""

import argparse
import json
import csv
import uuid
import random
from collections import defaultdict
from pathlib import Path

def det_uuid(name: str) -> str:
    return str(uuid.uuid5(uuid.NAMESPACE_URL, name)).replace("-", "")

def load_json_or_none(path):
    if not path:
        return None
    p = Path(path)
    with p.open("r") as f:
        return json.load(f)

def get_K_provider(K_json, fx, fy, cx, cy):
    default_K = [[fx, 0.0, cx],[0.0, fy, cy],[0.0, 0.0, 1.0]]
    if K_json is None:
        return lambda fid: default_K
    if isinstance(K_json, dict) and "K" in K_json:
        K0 = K_json["K"]
        return lambda fid: K0
    if isinstance(K_json, dict) and "frame_to_K" in K_json:
        ftK = K_json["frame_to_K"]
        def _k(fid):
            key = str(fid)
            return ftK.get(key, default_K)
        return _k
    if isinstance(K_json, list) and len(K_json) == 3:
        return lambda fid: K_json
    return lambda fid: default_K

def get_cam_token(frame_id, cam_map):
    if cam_map:
        key = str(frame_id)
        if key in cam_map: return cam_map[key]
    return det_uuid(f"cam_{frame_id}")

def get_ann_token(frame_id, track_id, ann_map):
    if ann_map:
        key = f"{frame_id}_{track_id}"
        if key in ann_map: return ann_map[key]
        fkey = str(frame_id)
        tkey = str(track_id)
        if fkey in ann_map and tkey in ann_map[fkey]:
            return ann_map[fkey][tkey]
    return det_uuid(f"ann_{track_id}_{frame_id}")

def to_box(r):
    # map to [x,y,z,w,l,h] (w=dx, l=dy, h=dz)
    return [float(r["x"]), float(r["y"]), float(r["z"]),
            float(r["dx"]), float(r["dy"]), float(r["dz"])]

def consecutive_enough(window, gap_tol):
    frames = [w["frame_id"] for w in window]
    gaps = [frames[i+1]-frames[i] for i in range(len(frames)-1)]
    return all(1 <= g <= 1+gap_tol for g in gaps)

def write_csv(rows, out_path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["cam_token","ann_token","bounding_box","future_bounding_box",
                    "K","attribute","label","future_label"])
        for r in rows:
            w.writerow(r)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--input_json", required=True, help="detections.json")
    ap.add_argument("--out_dir", required=True, help="folder for train/val/test CSVs")
    ap.add_argument("--obs_len", type=int, default=4)
    ap.add_argument("--pred_len", type=int, default=4)
    ap.add_argument("--gap_tolerance", type=int, default=0, help="allow small frame gaps")
    ap.add_argument("--min_track_len", type=int, default=8)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--train_ratio", type=float, default=0.8)
    ap.add_argument("--val_ratio", type=float, default=0.1)
    # test ratio = remaining
    ap.add_argument("--fx", type=float, default=800.0)
    ap.add_argument("--fy", type=float, default=800.0)
    ap.add_argument("--cx", type=float, default=640.0)
    ap.add_argument("--cy", type=float, default=360.0)
    ap.add_argument("--K_json", type=str, default=None,
                    help="Optional K JSON. See docstring for formats.")
    ap.add_argument("--frame_cam_token_map_json", type=str, default=None)
    ap.add_argument("--frame_ann_token_map_json", type=str, default=None)
    args = ap.parse_args()

    random.seed(args.seed)

    data = json.loads(Path(args.input_json).read_text())
    tracks = defaultdict(list)
    for d in data:
        if not all(k in d for k in ("frame_id","id","x","y","z","dx","dy","dz")):
            continue
        tracks[d["id"]].append(d)
    for k in tracks:
        tracks[k].sort(key=lambda r: r["frame_id"])

    tids = list(tracks.keys())
    random.shuffle(tids)
    n = len(tids)
    n_train = int(n * args.train_ratio)
    n_val = int(n * args.val_ratio)
    train_ids = set(tids[:n_train])
    val_ids = set(tids[n_train:n_train+n_val])
    test_ids = set(tids[n_train+n_val:])

    K_provider = get_K_provider(load_json_or_none(args.K_json),
                                args.fx, args.fy, args.cx, args.cy)
    cam_map = load_json_or_none(args.frame_cam_token_map_json)
    ann_map = load_json_or_none(args.frame_ann_token_map_json)

    obs, fut = args.obs_len, args.pred_len
    min_len = max(args.min_track_len, obs + fut)

    rows_train, rows_val, rows_test = [], [], []

    def push_row(split_rows, window, track_id):
        obs_part = window[:obs]
        fut_part = window[obs:]
        cam_tokens = [get_cam_token(r["frame_id"], cam_map) for r in window]
        ann_tokens = [get_ann_token(r["frame_id"], track_id, ann_map) for r in window]
        bb_obs = [to_box(r) for r in obs_part]
        bb_fut = [to_box(r) for r in fut_part]
        K_mat = K_provider(obs_part[0]["frame_id"])
        attributes = ["pedestrian"] * (obs + fut)   # as requested
        labels_obs = [[1] for _ in obs_part]        # single-class one-hot
        labels_fut = [[1] for _ in fut_part]

        split_rows.append([
            json.dumps(cam_tokens),
            json.dumps(ann_tokens),
            json.dumps(bb_obs),
            json.dumps(bb_fut),
            json.dumps(K_mat),
            json.dumps(attributes),
            json.dumps(labels_obs),
            json.dumps(labels_fut),
        ])

    for tid, seq in tracks.items():
        if len(seq) < min_len:
            continue
        split_rows = rows_train if tid in train_ids else rows_val if tid in val_ids else rows_test
        # sliding windows per track
        for s in range(0, len(seq) - (obs + fut) + 1):
            window = seq[s:s+obs+fut]
            if not consecutive_enough(window, args.gap_tolerance):
                continue
            push_row(split_rows, window, tid)

    out_dir = Path(args.out_dir)
    write_csv(rows_train, out_dir / "train.csv")
    write_csv(rows_val,   out_dir / "val.csv")
    write_csv(rows_test,  out_dir / "test.csv")

    print(f"tracks total: {len(tids)}  -> train {len(train_ids)}, val {len(val_ids)}, test {len(test_ids)}")
    print(f"rows: train {len(rows_train)}, val {len(rows_val)}, test {len(rows_test)}")
    print(f"saved to: {out_dir.resolve()}")
if __name__ == "__main__":
    main()
