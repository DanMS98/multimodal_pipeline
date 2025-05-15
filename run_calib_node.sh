export XDG_SESSION_TYPE=x11
colcon build --packages-select sensor_sync && source install/setup.bash && ros2 run sensor_sync calibration_node.py
