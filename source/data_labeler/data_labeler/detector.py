import rclpy
from rclpy.node import Node
import numpy as np
import glob, sys
import open3d
import open3d
import torch
import matplotlib
import numpy as np
import torch
from pathlib import Path
from loguru import logger as lu_logger
import open3d
import json

sys.path.insert(0, "/home/danial/gitclones/OpenPCDet/")

from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.models import build_network, load_data_to_gpu
from pcdet.utils import common_utils
from pcdet.datasets import DatasetTemplate

from interfaces.srv import RunDetection


box_colormap = [
    [1, 1, 1],
    [0, 1, 0],
    [0, 1, 1],
    [1, 1, 0],
]

# Global visualizer and geometry list
vis = None
added_geometries = []

def get_coor_colors(obj_labels):
    colors = matplotlib.colors.XKCD_COLORS.values()
    max_color_num = obj_labels.max()

    color_list = list(colors)[:max_color_num + 1]
    colors_rgba = [matplotlib.colors.to_rgba_array(color) for color in color_list]
    label_rgba = np.array(colors_rgba)[obj_labels]
    label_rgba = label_rgba.squeeze()[:, :3]

    return label_rgba


def init_scene():
    global vis
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.zeros(3)

    origin = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    vis.add_geometry(origin)
    added_geometries.append(origin)


def draw_scenes(points, gt_boxes=None, ref_boxes=None, ref_labels=None, ref_scores=None, point_colors=None, draw_origin=True):
    global vis, added_geometries

    if isinstance(points, torch.Tensor):
        points = points.cpu().numpy()
    if isinstance(gt_boxes, torch.Tensor):
        gt_boxes = gt_boxes.cpu().numpy()
    if isinstance(ref_boxes, torch.Tensor):
        ref_boxes = ref_boxes.cpu().numpy()

    for g in added_geometries:
        vis.remove_geometry(g, reset_bounding_box=False)
    added_geometries.clear()

    if draw_origin:
        origin = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        vis.add_geometry(origin)
        added_geometries.append(origin)

    pts = open3d.geometry.PointCloud()
    pts.points = open3d.utility.Vector3dVector(points[:, :3])
    if point_colors is None:
        pts.colors = open3d.utility.Vector3dVector(np.ones((points.shape[0], 3)))
    else:
        pts.colors = open3d.utility.Vector3dVector(point_colors)

    vis.add_geometry(pts)
    added_geometries.append(pts)

    if gt_boxes is not None:
        draw_box(vis, gt_boxes, (0, 0, 1))

    if ref_boxes is not None:
        draw_box(vis, ref_boxes, (0, 1, 0), ref_labels, ref_scores)

    vis.poll_events()
    vis.update_renderer()


def translate_boxes_to_open3d_instance(gt_box):
    center = gt_box[0:3]
    lwh = gt_box[3:6]
    axis_angles = np.array([0, 0, gt_box[6] + 1e-10])
    rot = open3d.geometry.get_rotation_matrix_from_axis_angle(axis_angles)
    box3d = open3d.geometry.OrientedBoundingBox(center, rot, lwh)

    line_set = open3d.geometry.LineSet.create_from_oriented_bounding_box(box3d)
    lines = np.asarray(line_set.lines)
    lines = np.concatenate([lines, np.array([[1, 4], [7, 6]])], axis=0)
    line_set.lines = open3d.utility.Vector2iVector(lines)

    return line_set, box3d


def draw_box(vis, gt_boxes, color=(0, 1, 0), ref_labels=None, score=None):
    for i in range(gt_boxes.shape[0]):
        line_set, box3d = translate_boxes_to_open3d_instance(gt_boxes[i])
        if ref_labels is None:
            line_set.paint_uniform_color(color)
        else:
            line_set.paint_uniform_color(box_colormap[ref_labels[i]])
        vis.add_geometry(line_set)
        added_geometries.append(line_set)

    return vis




class DemoDataset(DatasetTemplate):
    def __init__(self, dataset_cfg, class_names, training=True, root_path=None, logger=None, ext='.bin'):
        """
        Args:
            root_path:
            dataset_cfg:
            class_names:
            training:
            logger:
        """
        super().__init__(
            dataset_cfg=dataset_cfg, class_names=class_names, training=training, root_path=root_path, logger=logger
        )
        self.root_path = root_path
        self.ext = ext
        data_file_list = glob.glob(str(root_path / f'*{self.ext}')) if self.root_path.is_dir() else [self.root_path]

        data_file_list.sort()
        self.sample_file_list = data_file_list

    def __len__(self):
        return len(self.sample_file_list)

    def __getitem__(self, index):
        if self.ext == '.bin':
            points = np.fromfile(self.sample_file_list[index], dtype=np.float32).reshape(-1, 4)
        elif self.ext == '.npy':
            points = np.load(self.sample_file_list[index])
        else:
            raise NotImplementedError

        input_dict = {
            'points': points,
            'frame_id': index,
        }

        data_dict = self.prepare_data(data_dict=input_dict)
        return data_dict
    


class Detector(Node):
    def __init__(self, show_detection=True):
        super().__init__('detector_node')
        self.srv = self.create_service(RunDetection, 'run_detection', self.run_detection_callback)

        cfg_file = '/home/danial/gitclones/OpenPCDet/tools/cfgs/kitti_models/pointrcnn_iou.yaml'
        ckpt_file = '/home/danial/gitclones/OpenPCDet/checkpoints/pointrcnn_iou_7875.pth'
        dataset_path = '/home/danial/rosws/Dataset/merged_bin/'
        print(f"Dataset path: {dataset_path}")
        ext = '.bin'
        self.logger = lu_logger
        self.show_detection = show_detection

        cfg_from_yaml_file(cfg_file, cfg)
        self.demo_dataset = DemoDataset(
            dataset_cfg=cfg.DATA_CONFIG, class_names=cfg.CLASS_NAMES, training=False,
            root_path=Path(dataset_path), ext=ext, logger=self.logger
        )
        self.model = build_network(model_cfg=cfg.MODEL, num_class=cfg.CLASS_NAMES.__len__(), dataset=self.demo_dataset)
        self.model.load_params_from_file(ckpt_file, logger=common_utils.create_logger(), to_cpu=False)
        self.model.cuda() 
        self.model.eval()
        self.all_detections = []
        if self.show_detection:
            self.logger.info('Visualizing detection results...')
            init_scene()


    def run_detection_callback(self, request, response):
        self.prev_detections = []  # [{id, x, y, z}]
        self.next_id = 0
        
        with torch.no_grad():
            for idx, data_dict in enumerate(self.demo_dataset):
                self.logger.info(f'Visualized sample index: \t{idx + 1}')
                data_dict = self.demo_dataset.collate_batch([data_dict])
                load_data_to_gpu(data_dict)
                pred_dicts, _ = self.model.forward(data_dict)

                boxes = pred_dicts[0]['pred_boxes']
                scores = pred_dicts[0]['pred_scores']
                labels = pred_dicts[0]['pred_labels']

                mask = labels == 2
                boxes = boxes[mask]
                scores = scores[mask]
                labels = labels[mask]

                # score_thresh = 0.4
                score_thresh = 0.1
                mask = scores >= score_thresh
                boxes = boxes[mask]
                scores = scores[mask]
                labels = labels[mask]

                boxes_np = boxes.detach().cpu().numpy()
                scores_np = scores.detach().cpu().numpy()
                labels_np = labels.detach().cpu().numpy()

                self.logger.info(f'Boxes: {boxes_np}, \
                                  Scores: {scores_np}, \
                                  Labels: {labels_np}')
                
                frame_id = int(data_dict['frame_id'][0])

                current_dets = []
                assigned_ids = set()

                for i in range(len(boxes_np)):

                    x, y, z = boxes_np[i][:3]
                    min_dist = float('inf')
                    matched_id = None

                    for prev_detections in self.prev_detections:
                        if prev_detections['id'] in assigned_ids:
                            continue
                        dx = prev_detections['x'] - x
                        dy = prev_detections['y'] - y
                        dz = prev_detections['z'] - z
                        dist = np.sqrt(dx**2 + dy**2 + dz**2)

                        if dist < 0.5 and dist < min_dist:
                            min_dist = dist
                            matched_id = prev_detections['id']

                    if matched_id is not None:
                        assigned_id = matched_id
                        assigned_ids.add(matched_id)
                    else:
                        assigned_id = self.next_id
                        self.next_id += 1


                    det = {
                        "frame_id": frame_id,
                        "id": assigned_id,
                        "x": float(boxes_np[i][0]),
                        "y": float(boxes_np[i][1]),
                        "z": float(boxes_np[i][2]),
                        "dx": float(boxes_np[i][3]),
                        "dy": float(boxes_np[i][4]),
                        "dz": float(boxes_np[i][5]),
                        "yaw": float(boxes_np[i][6]),
                        "score": float(scores_np[i]),
                        "label": int(labels_np[i])
                    }
                    current_dets.append(det)
                    self.all_detections.append(det)
                
                self.prev_detections = current_dets

                if self.show_detection:
                    draw_scenes(
                        points=data_dict['points'][:, 1:],
                        ref_boxes=boxes,
                        ref_scores=scores,
                        ref_labels=labels
                    )

                
        self.logger.info('Detection done.')
        output_path = 'raw_detections.json'
        msg = f'[INFO] Saved {len(self.all_detections)} detections to {output_path}'
        with open(output_path, 'w') as f:
            json.dump(self.all_detections, f, indent=2)

        response.result_json = msg 
        self.logger.info(msg)
        self.all_detections.clear()  
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()





