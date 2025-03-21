#!/usr/bin/env python3

import rospy
import numpy as np
import scipy as sp
import os
import sys
import traceback

rospy.loginfo("Starting MPPI controller initialization...")
rospy.loginfo(f"Python executable: {sys.executable}")
rospy.loginfo(f"Python version: {sys.version}")
rospy.loginfo(f"Python path: {sys.path}")
rospy.loginfo(f"CUDA_HOME: {os.environ.get('CUDA_HOME', 'Not set')}")
rospy.loginfo(f"LD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH', 'Not set')}")

try:
    import cupy as cp
    USE_CUDA = True
    rospy.loginfo("CUDA acceleration available. Using GPU.")
    rospy.loginfo(f"CuPy version: {cp.__version__}")
    rospy.loginfo(f"CUDA device count: {cp.cuda.runtime.getDeviceCount()}")
    rospy.loginfo(f"Current CUDA device: {cp.cuda.runtime.getDevice()}")
    rospy.loginfo(f"NumPy version: {np.__version__}")
    rospy.loginfo(f"SciPy version: {sp.__version__}")
except ImportError as e:
    USE_CUDA = False
    rospy.logwarn(f"CUDA acceleration not available: {e}")
    rospy.logwarn("Traceback:")
    rospy.logwarn(traceback.format_exc())
except Exception as e:
    USE_CUDA = False
    rospy.logwarn(f"CUDA initialization failed: {e}")
    rospy.logwarn("Traceback:")
    rospy.logwarn(traceback.format_exc())

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
import geometry_msgs.msg


class MPPIRobotController:
    def __init__(self):
        # Get MPPI related parameters from the parameter server
        self.num_samples = rospy.get_param("~num_samples", 100)      # Reduced to 100 samples
        self.horizon = rospy.get_param("~horizon", 10)               # Reduced to 10 prediction steps
        self.lambda_ = rospy.get_param("~lambda", 1.0)              # Temperature parameter
        self.control_rate = rospy.get_param("~control_rate", 10.0)  # Control frequency
        self.dt = 1.0 / self.control_rate
        
        self.sigma = rospy.get_param("~mppi_sigma", 0.15)  # Sample noise

        # UR5e joint names, please correspond with urdf/driver
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
        # Current joint positions/velocities
        self.current_q = np.zeros(6)
        self.current_q_dot = np.zeros(6)
        self.initialized = False

        # TF setup for tracking link positions
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))  # Buffer for 10 seconds
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
        # Define the links you want to track
        self.tracked_links = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "tool0"  # End effector
        ]
    
        # Add visualization publisher
        self.link_marker_pub = rospy.Publisher('/robot_link_markers', MarkerArray, queue_size=1)

        # Set an example target pose (in joint space)
        self.q_des = np.array([0.45, -1.1, 1.0, -1.5, 0.4, 1])

        # Improved obstacle definition
        self.obstacles = [
            {
                'center': np.array([0.4, 0.25, 0.3]),
                'radius': 0.02,
                'type': 'sphere'
            }
        ]
        self.safe_margin = 0.05  # Safety margin
        
        # Define critical links (for obstacle avoidance calculation)
        self.critical_links = [
            'upper_arm_link',
            'forearm_link',
            'wrist_1_link',
            'tool0'
        ]
        
        # Set weights for all links
        self.obstacle_weights = {
            'base_link': 1.0,
            'shoulder_link': 1.0,
            'upper_arm_link': 1.0,
            'forearm_link': 1.0,
            'wrist_1_link': 1.0,
            'wrist_2_link': 1.0,
            'wrist_3_link': 1.0,
            'tool0': 1.0
        }
        
        # Joint limits
        self.joint_limits_lower = np.array([-6.28, -3.14, -3.14, -6.28, -6.28, -6.28])
        self.joint_limits_upper = np.array([6.28, 3.14, 3.14, 6.28, 6.28, 6.28])
        # Joint velocity limits
        self.joint_vel_limits = np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0])

        # Subscribe to JointState
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb, queue_size=1)

        # Publish to the appropriate topic for Gazebo
        self.cmd_pub = rospy.Publisher(
            "/eff_joint_traj_controller/command",  # Update this topic if necessary
            JointTrajectory,
            queue_size=1
        )

        # Control loop
        self.rate = rospy.Rate(self.control_rate)

        self.mean_control_seq = np.zeros((self.horizon, 6), dtype=np.float32)

        # TF setup (get end effector pose)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.base_link = "base_link"
        self.end_link  = "tool0"

        rospy.loginfo("MPPI Controller (Joint-space) + TF for EE Pose Initialized.")

        self.lambda_min = 0.1
        self.lambda_max = 2.0
        self.lambda_adaptation_rate = 0.1
        self.prev_min_cost = float('inf')
        
        # Cost weights
        self.w_goal = 1.0          # Goal position weight
        self.w_obstacle = 0.5      # Obstacle avoidance weight
        self.w_smoothness = 0.3    # Smoothness weight
        self.w_energy = 0.2        # Energy efficiency weight
        
        # Add kinematic constraints
        self.max_acceleration = np.array([1.0] * 6)  # Maximum acceleration limits

        self.use_cuda = USE_CUDA
        if self.use_cuda:
            rospy.loginfo("Initializing GPU arrays...")
            try:
                # Transfer constant arrays to GPU
                self.gpu_joint_limits_lower = cp.array(self.joint_limits_lower)
                self.gpu_joint_limits_upper = cp.array(self.joint_limits_upper)
                self.gpu_joint_vel_limits = cp.array(self.joint_vel_limits)
                self.gpu_max_acceleration = cp.array(self.max_acceleration)
                rospy.loginfo("GPU arrays initialized successfully")
            except Exception as e:
                rospy.logwarn(f"Failed to initialize GPU arrays: {e}")
                self.use_cuda = False
        else:
            rospy.logwarn("Running in CPU mode")

        # Add TF caching related variables
        self.last_tf_update = rospy.Time.now()
        self.tf_update_interval = rospy.Duration(0.1)  # Update TF every 0.1 seconds
        self.cached_link_positions = {}

    def joint_state_cb(self, msg):
        # Get current joints from /joint_states
        name_idx_map = {n: i for i, n in enumerate(msg.name)}
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in name_idx_map:
                j_idx = name_idx_map[joint_name]
                self.current_q[i] = msg.position[j_idx]
                self.current_q_dot[i] = msg.velocity[j_idx]
        
        self.initialized = True

    def run(self):
        rospy.loginfo("Waiting for joint states...")
        while not rospy.is_shutdown() and not self.initialized:
            rospy.sleep(0.1)
        
        rospy.loginfo("Start MPPI control loop...")
        while not rospy.is_shutdown():
            if self.use_cuda:
                try:
                    # GPU version sampling and computation
                    gpu_mean_control = cp.array(self.mean_control_seq[None, :, :])
                    gpu_noise = cp.random.normal(
                        loc=0.0, scale=self.sigma,
                        size=(self.num_samples, self.horizon, 6)
                    )
                    gpu_control_samples = gpu_mean_control + gpu_noise
                    
                    # Compute costs (GPU parallel)
                    gpu_costs = self.parallel_rollout_cost(gpu_control_samples)
                    
                    # Transfer results back to CPU
                    costs = cp.asnumpy(gpu_costs)
                    control_samples = cp.asnumpy(gpu_control_samples)
                    
                    # Add debug information
                    if rospy.get_time() % 1.0 < 0.1:  # Print once per second
                        rospy.loginfo(f"GPU computation successful. Mean cost: {np.mean(costs):.3f}")
                except Exception as e:
                    rospy.logwarn(f"GPU computation failed: {e}, falling back to CPU")
                    rospy.logwarn("Traceback:")
                    rospy.logwarn(traceback.format_exc())
                    self.use_cuda = False
                    # Use CPU version as fallback
                    control_samples = self.mean_control_seq[None, :, :] + np.random.normal(
                        loc=0.0, scale=self.sigma,
                        size=(self.num_samples, self.horizon, 6)
                    )
                    costs = np.zeros(self.num_samples)
                    for i in range(self.num_samples):
                        costs[i] = self.rollout_cost(control_samples[i])
            else:
                # Original CPU version code
                control_samples = self.mean_control_seq[None, :, :] + np.random.normal(
                    loc=0.0, scale=self.sigma,
                    size=(self.num_samples, self.horizon, 6)
                )
                costs = np.zeros(self.num_samples)
                for i in range(self.num_samples):
                    costs[i] = self.rollout_cost(control_samples[i])
            
            # 3) Calculate weights using the cost
            min_cost = np.min(costs)
            self.update_lambda(min_cost)
            weights = np.exp(-(costs - min_cost) / self.lambda_)
            weights_sum = np.sum(weights)
            
            # 4) Obtain the new control sequence (weighted average)
            new_control_seq = np.einsum("i,ijk->jk", weights, control_samples) / weights_sum
            
            # 5) Take the first step control
            u = new_control_seq[0]
            u = np.clip(u, -self.joint_vel_limits, self.joint_vel_limits)
            # 6) Convert to the next target joint angles
            q_next = np.clip(self.current_q + u * self.dt, self.joint_limits_lower, self.joint_limits_upper)
            
            # 7) Publish the joint trajectory command
            self.publish_trajectory_command(q_next)

            self.mean_control_seq = new_control_seq
            
            # Log the end-effector pose
            ee_pose = self.get_end_effector_pose()
            if ee_pose is not None:
                x, y, z, qx, qy, qz, qw = ee_pose
                rospy.loginfo(
                    "Joints= [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f],  End-Effector= (%.3f, %.3f, %.3f)"
                    % (self.current_q[0], self.current_q[1], self.current_q[2],
                       self.current_q[3], self.current_q[4], self.current_q[5],
                       x, y, z)
                )
            else:
                rospy.logwarn("End-effector TF not available yet.")

            # 8) Wait for the next loop
            self.rate.sleep()

    def update_lambda(self, min_cost):
        """Adaptively update temperature parameter"""
        if self.prev_min_cost != float('inf'):
            cost_improvement = self.prev_min_cost - min_cost
            if cost_improvement > 0:
                # Cost is improving, increase exploration
                self.lambda_ = min(self.lambda_max, 
                                 self.lambda_ * (1 + self.lambda_adaptation_rate))
            else:
                # Cost is not improving, decrease exploration
                self.lambda_ = max(self.lambda_min, 
                                 self.lambda_ * (1 - self.lambda_adaptation_rate))
        self.prev_min_cost = min_cost

    def rollout_cost(self, controls):
        """Improved cost function calculation"""
        q_sim = np.copy(self.current_q)
        q_dot_sim = np.copy(self.current_q_dot)
        cost = 0.0
        prev_u = None

        for t in range(self.horizon):
            u_t = controls[t]
            
            # Velocity and acceleration constraints
            u_t = np.clip(u_t, -self.joint_vel_limits, self.joint_vel_limits)
            if prev_u is not None:
                acc = (u_t - prev_u) / self.dt
                acc = np.clip(acc, -self.max_acceleration, self.max_acceleration)
                u_t = prev_u + acc * self.dt
            
            # Update state
            q_dot_sim = u_t
            q_sim += q_dot_sim * self.dt
            q_sim = np.clip(q_sim, self.joint_limits_lower, self.joint_limits_upper)
            
            # Goal position cost
            goal_cost = np.linalg.norm(q_sim - self.q_des) ** 2
            
            # Obstacle avoidance cost
            obstacle_cost = self.obstacle_avoidance_cost(q_sim)
            
            # Smoothness cost
            smoothness_cost = 0.0
            if prev_u is not None:
                smoothness_cost = np.sum((u_t - prev_u) ** 2)
            
            # Energy efficiency cost
            energy_cost = np.sum(u_t ** 2)
            
            # Total cost
            cost += (self.w_goal * goal_cost +
                    self.w_obstacle * obstacle_cost +
                    self.w_smoothness * smoothness_cost +
                    self.w_energy * energy_cost)
            
            prev_u = u_t
            
        return cost
    
    def get_link_positions(self):
        """
        Get positions of all tracked links using caching mechanism to reduce TF query frequency
        """
        current_time = rospy.Time.now()
        if (current_time - self.last_tf_update) < self.tf_update_interval:
            return self.cached_link_positions
            
        # Update TF cache
        self.cached_link_positions = {}
        try:
            for link in self.tracked_links:
                trans = self.tf_buffer.lookup_transform(
                    "world",  # Target frame (world frame)
                    link,     # Source frame (link we're interested in)
                    rospy.Time(0),  # Get latest transform
                    rospy.Duration(0.1)  # Wait up to 0.1 seconds
                )
                
                # Extract position from transform
                self.cached_link_positions[link] = np.array([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
        
        self.last_tf_update = current_time
        return self.cached_link_positions

    def obstacle_avoidance_cost(self, q):
        """
        Calculate obstacle avoidance cost
        Check all links using smooth cost function
        """
        cost = 0.0
        
        # Get positions of all links
        link_positions = self.get_link_positions()
        if not link_positions:  # If positions cannot be obtained, use simple check
            for obs in self.obstacles:
                dist_to_obs = np.linalg.norm(q[:3] - obs['center'])
                if dist_to_obs < obs['radius'] + self.safe_margin:
                    cost += 200.0 / (dist_to_obs + 1e-6)
            return cost
        
        # Check all links
        for link, position in link_positions.items():
            link_weight = self.obstacle_weights.get(link, 1.0)
            
            for obs in self.obstacles:
                dist_to_obs = np.linalg.norm(position - obs['center'])
                safe_dist = obs['radius'] + self.safe_margin
                
                if dist_to_obs < safe_dist:
                    # Use smooth cost function
                    normalized_dist = dist_to_obs / safe_dist
                    cost += 200.0 * link_weight * (1 - normalized_dist)**2
                    
        return cost
    
    def joint_limit_cost(self, q):
        """Calculate the cost for joint limits."""
        cost = 0.0
        margin_factor = 10.0
        for i in range(6):
            if q[i] < self.joint_limits_lower[i]:
                diff = (self.joint_limits_lower[i] - q[i])
                cost += margin_factor * (diff**2)
            elif q[i] > self.joint_limits_upper[i]:
                diff = (q[i] - self.joint_limits_upper[i])
                cost += margin_factor * (diff**2)
        return cost

    def publish_trajectory_command(self, q_target):
        """
        Send a single-point JointTrajectory command to
        /arm_controller/command
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = q_target.tolist()
        point.time_from_start = rospy.Duration(self.dt)  # Reach in the next dt seconds
        
        traj_msg.points.append(point)
        traj_msg.header.stamp = rospy.Time.now()
        
        self.cmd_pub.publish(traj_msg)

    def get_end_effector_pose(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_link,  
                self.end_link,   
                rospy.Time(0)   
            )
            trans = transform_stamped.transform.translation
            rot   = transform_stamped.transform.rotation
            return (trans.x, trans.y, trans.z, rot.x, rot.y, rot.z, rot.w)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))
            return None

    def parallel_rollout_cost(self, gpu_control_samples):
        """GPU parallel version of rollout cost calculation"""
        if not self.use_cuda:
            return None
            
        num_samples = gpu_control_samples.shape[0]
        gpu_costs = cp.zeros(num_samples)
        
        # Transfer current state to GPU
        gpu_q_sim = cp.tile(cp.array(self.current_q), (num_samples, 1))
        gpu_q_dot_sim = cp.tile(cp.array(self.current_q_dot), (num_samples, 1))
        gpu_q_des = cp.array(self.q_des)
        
        # Calculate costs for all obstacles at once
        gpu_obstacle_costs = cp.zeros(num_samples)
        for obs in self.obstacles:
            obs_center = cp.array(obs['center'])
            # Calculate distance from all joint positions to obstacles
            # Only use first three joint positions to calculate distance to obstacles
            dist_to_obs = cp.linalg.norm(gpu_q_sim[:, :3] - obs_center, axis=1)
            safe_dist = obs['radius'] + self.safe_margin
            mask = dist_to_obs < safe_dist
            gpu_obstacle_costs[mask] += 200.0 * (1 - dist_to_obs[mask]/safe_dist)**2
        
        for t in range(self.horizon):
            # Process all samples in batch
            gpu_u_t = gpu_control_samples[:, t, :]
            
            # Velocity limits
            gpu_u_t = cp.clip(gpu_u_t, -self.gpu_joint_vel_limits, self.gpu_joint_vel_limits)
            
            # Update state
            gpu_q_dot_sim = gpu_u_t
            gpu_q_sim += gpu_q_dot_sim * self.dt
            gpu_q_sim = cp.clip(gpu_q_sim, self.gpu_joint_limits_lower, self.gpu_joint_limits_upper)
            
            # Calculate various costs
            goal_cost = cp.sum((gpu_q_sim - gpu_q_des) ** 2, axis=1)
            energy_cost = cp.sum(gpu_u_t ** 2, axis=1)
            
            # Accumulate costs
            gpu_costs += (self.w_goal * goal_cost +
                        self.w_obstacle * gpu_obstacle_costs +
                        self.w_energy * energy_cost)
        
        return gpu_costs

def main():
    rospy.init_node("mppi_ur5e_controller", anonymous=True)
    controller = MPPIRobotController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()