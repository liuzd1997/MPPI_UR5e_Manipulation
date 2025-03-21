# MPPI Control for UR5e Robot: Efficient Manipulation in Presence of Dynamic Uncertainties Via Output-Sampled Model-Based Learning Control

This repository contains a Model Predictive Path Integral (MPPI) controller implementation for the UR5e robot using ROS and Gazebo simulation. The controller features GPU acceleration using CUDA for improved performance.

## Features

- MPPI-based motion planning and control
- GPU-accelerated computation using CUDA
- Obstacle avoidance
- Joint limit constraints
- Smooth trajectory generation
- Real-time visualization

## Prerequisites

- ROS Noetic
- Python 3.8
- CUDA 11.8
- Gazebo
- UR5e robot simulation packages

## Dependencies

- NumPy >= 1.24.4
- SciPy >= 1.10.1
- CuPy >= 12.3.0
- tf2_ros
- geometry_msgs
- sensor_msgs
- trajectory_msgs
- visualization_msgs

## Installation

1. Create a new ROS workspace:
```bash
mkdir -p ~/ur5e_sim_ws/src
cd ~/ur5e_sim_ws/src
```

2. Clone this repository:
```bash
git clone https://github.com/liuzd1997/MPPI_UR5e_Manipulation.git
```

3. Install Python dependencies:
```bash
pip3 install numpy scipy cupy-cuda11x
```

4. Build the workspace:
```bash
cd ~/ur5e_sim_ws
catkin_make
source devel/setup.bash
```

## Usage

1. Start the simulation:
```bash
cd ~/ur5e_sim_ws
./src/mppi_control/scripts/start_mppi.sh
```

This will launch:
- Gazebo simulation
- UR5e robot
- MPPI controller

## Configuration

The controller can be configured through ROS parameters:

- `num_samples`: Number of samples for MPPI (default: 100)
- `horizon`: Prediction horizon (default: 10)
- `lambda`: Temperature parameter (default: 1.0)
- `control_rate`: Control frequency (default: 10.0)
- `mppi_sigma`: Sample noise (default: 0.15)

## GPU Acceleration

The controller uses CUDA for GPU acceleration. Make sure you have:
- NVIDIA GPU with CUDA support
- CUDA 11.8 installed
- CuPy installed with CUDA 11.8 support

If GPU acceleration is not available, the controller will automatically fall back to CPU mode.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. 