# 4Mecanum_ws-ROS2

A ROS2 workspace for a 4-wheel mecanum-drive mobile robot, supporting omnidirectional movement.

## 📦 Packages

| Package | Purpose |
|---|---|
| `mecanum_description` | Robot URDF/model description |
| `mecanum_firmware` | Low-level hardware interface for the mecanum drive |
| `mecanum_bringup` | Launch files to start the robot stack |
| `mecanum_controller` | Mecanum-wheel drive controller (omnidirectional kinematics) |
| `mecanum_py_examples` | Python example nodes |

## 🛠️ Tech

- **Framework:** ROS2
- **Language:** Python (with URDF/XML for description)
- **Robot type:** 4-wheel mecanum drive (supports strafing and omnidirectional motion)

## 🚀 Usage

```bash
# From the workspace root
colcon build
source install/setup.bash
ros2 launch mecanum_bringup <launch_file>.launch.py
```

## 📌 Notes

This workspace was built to explore mecanum-wheel kinematics and control in ROS2 — a natural extension after working with standard differential-drive robots.
