#!/bin/bash

# Set system Python environment
export PATH=/usr/bin:$PATH
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH

# Set workspace path
WS_PATH=~/ur5e_sim_ws

# Build workspace
cd $WS_PATH
catkin_make

# Setup ROS environment
source devel/setup.bash

# Terminal 1: Launch Gazebo
gnome-terminal --tab --title="Gazebo" -- bash -c "
export PATH=/usr/bin:\$PATH;
source $WS_PATH/devel/setup.bash;
roslaunch gazebo_ros empty_world.launch world_name:=\$(rospack find mppi_control)/worlds/ur5e_table.world;
exec bash"

# Wait for Gazebo to fully start
sleep 5

# Terminal 2: Launch UR5e
gnome-terminal --tab --title="UR5e" -- bash -c "
export PATH=/usr/bin:\$PATH;
source $WS_PATH/devel/setup.bash;
roslaunch ur_gazebo ur5e_bringup.launch robot_name:=ur5e use_sim_time:=true gazebo_model_path:=\$(rospack find mppi_control)/worlds;
exec bash"

# Wait for UR5e to fully start
sleep 5

# Terminal 3: Launch controller
gnome-terminal --tab --title="Controller" -- bash -c "
export PATH=/usr/bin:\$PATH;
export CUDA_HOME=/usr/local/cuda-11.8;
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:\$LD_LIBRARY_PATH;
source $WS_PATH/devel/setup.bash;
export PYTHONPATH=/usr/local/lib/python3.8/dist-packages:/usr/lib/python3/dist-packages:\$PYTHONPATH;
echo 'Environment variables:';
echo 'CUDA_HOME=' \$CUDA_HOME;
echo 'LD_LIBRARY_PATH=' \$LD_LIBRARY_PATH;
echo 'PYTHONPATH=' \$PYTHONPATH;
echo 'Python path:' \$(which python3);
echo 'NumPy location:' \$(python3 -c 'import numpy; print(numpy.__file__)');
echo 'CuPy location:' \$(python3 -c 'import cupy; print(cupy.__file__)');
rosrun mppi_control mppi_ur5e_controller.py;
exec bash" 