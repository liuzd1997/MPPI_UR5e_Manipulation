
~/ur5e_sim_ws/src/mppi_control/scripts/start_mppi.sh
sudo pip3 install cupy-cuda12x
# Terminal 1: Launch just Gazebo with the world
export PATH=/usr/bin:$PATH
roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find mppi_control)/worlds/obstacles2.world

# Terminal 2: After Gazebo is running, launch UR5e but tell it to use existing Gazebo
export PATH=/usr/bin:$PATH
roslaunch ur_gazebo ur5e_bringup.launch robot_name:=ur5e use_sim_time:=true gazebo_model_path:=$(rospack find mppi_control)/worlds





# Terminal 3: Launch your controller
export PATH=/usr/bin:$PATH
cd ~/ur5e_sim_ws
catkin_make
source devel/setup.bash
rospack find mppi_control
rosrun mppi_control mppi_ur5e_controller.py


rosservice call /gazebo/set_model_configuration "model_name: 'robot'
urdf_param_name: 'robot_description'
joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joint_positions: [1.0, -1.0, 1.57, 0.5, 0.5, 0.0]"
rosrun gazebo_ros spawn_model -param robot_description -urdf -model ur5e -x 0.5 -y 0 -z 0.275

# next
1) forword kinematics
2) visualization
2) cost modification
3) cuda accelaration






## Open a terminal
run

````bash
export PATH=/usr/bin:$PATH
roslaunch ur_gazebo ur5_bringup.launch
````

## in VS code terminal
run

````bash
export PATH=/usr/bin:$PATH
cd ~/ur5e_sim_ws
source devel/setup.bash
rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model sphere1 -x 0.2 -y 0.2 -z 0.3
rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model sphere2 -x 0.25 -y 0.25 -z 0.15
rosrun mppi_control mppi_ur5e_controller.py
````

next step
global planner
cuda



rostopic pub -1 /eff_joint_traj_controller/command \
trajectory_msgs/JointTrajectory '
joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
points:
- positions: [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
  time_from_start: {secs: 3, nsecs: 0}
- positions: [1.0, -1.0, 1.57, 0.5, 0.5, 0.0]
  time_from_start: {secs: 6, nsecs: 0}
'


rosrun gazebo_ros spawn_model \
  -file /path/to/box.sdf \
  -sdf \
  -model obstacle_box01 \
  -x 1.0 -y 0.2 -z 0.0

  rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/box.sdf -sdf -model box_obstacle -x 1.0 -y 0.2 -z 0.0


  rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model box_obstacle -x 0.5 -y 0.5 -z 0.25

  rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model box_obstacle x 1.0 -y 1.0 -z 0.3


rosrun gazebo_ros spawn_model -sdf -model sphere1 -x 0.5 -y 0.5 -z 0.25 -file $(rospack find gazebo_ros)/worlds/sphere.sdf
rosrun gazebo_ros spawn_model -sdf -model sphere2 -x 1.0 -y 1.0 -z 0.3 -file $(rospack find gazebo_ros)/worlds/sphere.sdf

rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model sphere1 -x 0.5 -y 0.5 -z 0.25
rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model sphere2 -x 0.2 -y 0.2 -z 0.3
rosrun gazebo_ros spawn_model -file ~/ur5e_sim_ws/src/mppi_control/worlds/sphere.sdf -sdf -model sphere2 -x 0.25 -y 0.25 -z 0.1

