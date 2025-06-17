# Sensor Sync: Pedestrian detection pipeline (ROS 2 Humble)

These ROS 2 packages provide nodes to transform raw LiDAR, radar, and camera data into synchronized and calibrated messages, labeled and ready to train data.

## 📦 Package: `sensor_sync` (more coming)

### ✅ Features

- Subscribes to `/ouster/points`, `/radar_data/point_cloud`, and `/camera/camera/color/image_raw` (`sensor_msgs/msg/PointCloud2`)
- Applies transformation from sensor frames to `base_link`
- todo: using pseudolabeling for pedestrian detection.

---

## 🔧 Installation

```bash
cd ~/ros_ws/src
git clone git@github.com:DanMS98/multimodal_pipeline.git
cd ..
colcon build
source install/setup.bash
```

---

## 🔍 Visualizer: Projecting LiDAR and Radar Points onto Camera

The `visualizer.py` node projects 3D points from LiDAR and radar onto the 2D camera image using known `tf_static` transformations and camera intrinsics.  Just to check Transformations.

### 🖼️ Camera View

![Visualizer GIF](media/visualizer_demo.gif)

