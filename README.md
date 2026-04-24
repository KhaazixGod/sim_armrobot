# pcgrad Package

This package provides a robotic arm simulation environment within **NVIDIA Isaac Sim**, integrated with **ROS 2** for control.

## 📋 System Requirements

To run this package, you will need:
- **Ubuntu 22.04**
- **ROS 2 Humble**
- **NVIDIA Isaac Sim** (Version 4.5.0)
- A powerful NVIDIA GPU with Ray Tracing (RTX) support.

## 🛠 Installation

1. Navigate to your workspace directory:
   ```bash
   cd ~/Project/ros2_ws
   ```

2. Build the package using `colcon`:
   ```bash
   colcon build --packages-select pcgrad
   ```

3. Source the environment:
   ```bash
   source install/setup.bash
   ```

## 🚀 Usage

This package includes a launch file to start both the simulation environment and a mock controller node simultaneously.

Run the following command:
```bash
ros2 launch pcgrad pcgrad_launch.py
```

## 📡 ROS 2 Topics

The environment publishes the following camera streams:

- `/camera_top/image_raw` (sensor_msgs/Image): Top-down view of the workspace.
- `/camera_side/image_raw` (sensor_msgs/Image): Side-angle view of the workspace.

You can visualize these topics using `rqt_image_view`:
```bash
ros2 run rqt_image_view rqt_image_view
```


## 📂 Project Structure

- `launch/pcgrad_launch.py`: Configures the launch sequence with a delay to ensure Isaac Sim is ready.
- `pcgrad/run_environment.py`: Main source code managing Isaac Sim and the ROS 2 node integration.
- `pcgrad/joints_state.py`: Python node publishing mock control data.
- `setup.py`: Defines entry points for ROS 2 executables.

## ⚠️ Important Notes

- Ensure that all asset paths in `run_environment.py` are correct for your local setup.
- Isaac Sim requires time to initialize the GPU. The launch file is configured with a 3-second delay before the environment node starts.
