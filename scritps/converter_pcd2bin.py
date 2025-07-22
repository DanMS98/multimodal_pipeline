import os
import numpy as np
import open3d as o3d

def convert_pcd_to_bin(pcd_path, bin_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    if pcd.has_colors():
        intensity = np.mean(np.asarray(pcd.colors), axis=1).reshape(-1, 1)
        points = np.hstack((points, intensity))
    else:
        intensity = np.zeros((points.shape[0], 1), dtype=np.float32)
        points = np.hstack((points, intensity))

    points = points.astype(np.float32)
    points.tofile(bin_path)

def batch_convert(folder_in, folder_out):
    os.makedirs(folder_out, exist_ok=True)
    for fname in os.listdir(folder_in):
        if fname.endswith('.pcd'):
            pcd_file = os.path.join(folder_in, fname)
            bin_file = os.path.join(folder_out, fname.replace('.pcd', '.bin'))
            convert_pcd_to_bin(pcd_file, bin_file)
            print(f"Converted: {fname} → {os.path.basename(bin_file)}")

if __name__ == "__main__":
    in_folder = "Dataset/merged/"
    out_folder = "Dataset/merged_bin/"
    batch_convert(in_folder, out_folder)
