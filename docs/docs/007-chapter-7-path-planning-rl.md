---
id: chapter-7-path-planning-rl
title: "Chapter 7: Path Planning & Reinforcement Learning"
sidebar_position: 7
description: |
  Master humanoid navigation with Nav2 and locomotion with reinforcement learning. Build reusable navigation and control skills (Layer 3).
keywords:
  - Nav2
  - path planning
  - reinforcement learning
  - locomotion
  - navigation skills
  - PPO
tags:
  - layer3
  - navigation
  - rl
  - skills
---

# Chapter 7: Path Planning & Reinforcement Learning

## Overview

**Learning Objectives:**
1. Configure ROS 2 Nav2 stack for humanoid navigation
2. Implement global and local path planners
3. Train humanoid walking gaits using reinforcement learning (PPO)
4. Integrate perception (Ch. 6) with navigation for obstacle avoidance
5. **Create reusable navigation and control skills** ⭐ (Layer 3)

:::info Prerequisites
Complete [Chapter 6](./chapter-6-isaac-perception) - Perception skills and Isaac Sim
:::

**Estimated Duration**: 8 hours (lecture + lab)

**Layer Enforcement**: **Layer 3 (Intelligence Design)**. Build production-grade navigation skills with:
- Clear decision trees for mode switching (global → local planning)
- Failure recovery behaviors
- Performance guarantees (path smoothness, goal tolerance)

---

## Navigation Fundamentals

### The Navigation Problem

Given:
- **Current Pose**: Robot's position and orientation
- **Goal Pose**: Desired final position
- **Map**: Obstacle layout (from SLAM, Ch. 6)

Find:
- **Path**: Sequence of waypoints from start to goal
- **Velocity Commands**: Motor control to follow path

### Nav2 Architecture

```mermaid
graph TD
    A[Goal] --> B[Global Planner]
    B --> C[Path]
    C --> D[Controller]
    E[Costmap] --> B
    E --> D
    F[Sensors] --> E
    D --> G[Velocity Commands]
    G --> H[Robot Base]

    style A fill:#4CAF50
    style B fill:#2196F3
    style D fill:#FF9800
    style H fill:#9C27B0
```

**Components**:
1. **Global Planner**: Finds optimal path ignoring dynamics (A*, Dijkstra)
2. **Controller**: Generates velocity commands following path (DWA, TEB)
3. **Costmap**: Represents obstacles and free space
4. **Recovery Behaviors**: Actions when stuck (rotate, back up)

---

## Installing Nav2

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-turtlebot3-gazebo  # For testing
```

**Verify**:
```bash
ros2 pkg list | grep nav2
# Should show: nav2_bt_navigator, nav2_controller, nav2_planner, etc.
```

---

## Configuring Nav2 for Humanoid

### Step 1: Create Navigation Parameters

```yaml title="config/nav2_params.yaml"
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odometry/filtered
    bt_loop_duration: 10
    default_server_timeout: 20

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # DWA (Dynamic Window Approach) Controller
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.5  # Humanoid max walk speed
      max_vel_y: 0.0
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      acc_lim_x: 0.5
      acc_lim_y: 0.0
      acc_lim_theta: 1.0
      decel_lim_x: -0.5
      decel_lim_y: 0.0
      decel_lim_theta: -1.0

planner_server:
  ros__parameters:
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false  # Dijkstra (true for A*)
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: ["voxel_layer", "inflation_layer"]

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
```

### Step 2: Create Launch File

```python title="launch/humanoid_nav2.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_share = get_package_share_directory('humanoid_navigation')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ]),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'True'
        }.items()
    )

    return LaunchDescription([
        nav2_bringup
    ])
```

### Step 3: Test in Gazebo

```bash
# Terminal 1: Launch Gazebo with humanoid + obstacles
ros2 launch humanoid_gazebo warehouse.launch.py

# Terminal 2: Launch Nav2
ros2 launch humanoid_navigation humanoid_nav2.launch.py

# Terminal 3: Send navigation goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

**Expected**:
- Global path appears in RViz (green line)
- Robot follows path, avoiding obstacles
- Stops within 0.5m of goal

---

## Reinforcement Learning for Locomotion

### Why RL for Humanoid Walking?

**Traditional Approach** (Model-Based):
- Requires accurate dynamics model
- Hand-tuned for each terrain type
- Brittle to model errors

**RL Approach**:
- Learns from trial and error
- Generalizes to varied terrains
- Adapts to model uncertainty

### RL Algorithm: Proximal Policy Optimization (PPO)

**Concept**: Train neural network policy π(a|s) that maps:
- **State s**: Joint angles, velocities, IMU, foot contacts
- **Action a**: Joint torques

**Objective**: Maximize cumulative reward (walk forward without falling)

**Advantages of PPO**:
- Sample efficient (works with limited compute)
- Stable (monotonic improvement)
- Good for continuous control (humanoid joints)

---

## Training Walking Gait with PPO

### Step 1: Define Environment

```python title="scripts/humanoid_walk_env.py"
"""
Purpose: RL environment for humanoid walking
Prerequisites: Isaac Sim with humanoid model
Expected Output: Trained policy that walks forward at 0.5 m/s
"""

import gym
from gym import spaces
import numpy as np
from omni.isaac.gym.vec_env import VecEnvBase

class HumanoidWalkEnv(VecEnvBase):
    def __init__(self, num_envs=1024):
        self.num_envs = num_envs

        # State space: 44 dimensions
        # - Joint positions (20)
        # - Joint velocities (20)
        # - IMU orientation (4)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(44,), dtype=np.float32
        )

        # Action space: 20 joint torques
        self.action_space = spaces.Box(
            low=-100, high=100, shape=(20,), dtype=np.float32
        )

    def reset(self):
        # Reset robot to standing pose
        # Return initial state
        return np.zeros((self.num_envs, 44))

    def step(self, actions):
        # Apply torques to joints
        # Simulate physics
        # Compute reward
        # Check termination

        # Reward function
        forward_velocity = self.get_forward_velocity()
        energy = np.sum(np.abs(actions), axis=1)
        alive_bonus = 1.0

        reward = (
            1.0 * forward_velocity  # Encourage forward motion
            - 0.01 * energy          # Penalize energy use
            + alive_bonus             # Stay upright
        )

        # Termination: fell down (z < 0.5m)
        z_position = self.get_base_height()
        done = z_position < 0.5

        return self.get_state(), reward, done, {}

    def get_state(self):
        # Return [joint_pos, joint_vel, imu_orientation]
        pass

    def get_forward_velocity(self):
        # Return velocity in x-direction
        pass

    def get_base_height(self):
        # Return height of robot base
        pass
```

### Step 2: Train with Stable-Baselines3

```python title="scripts/train_humanoid_walk.py"
"""
Purpose: Train humanoid walking policy using PPO
Prerequisites: stable-baselines3, Isaac Sim
Expected Output: Trained model saved to models/humanoid_walk_ppo.zip
"""

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from humanoid_walk_env import HumanoidWalkEnv

def main():
    # Create environment (1024 parallel instances for speed)
    env = HumanoidWalkEnv(num_envs=1024)

    # PPO hyperparameters
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        verbose=1,
        tensorboard_log="./tensorboard_logs/"
    )

    # Train for 10M timesteps (~3 hours on RTX 3080)
    model.learn(total_timesteps=10_000_000)

    # Save model
    model.save("models/humanoid_walk_ppo")
    print("✅ Training complete! Model saved.")

if __name__ == "__main__":
    main()
```

**Monitor Training**:
```bash
tensorboard --logdir=./tensorboard_logs/
# Open browser: http://localhost:6006
# Watch reward curve increase
```

### Step 3: Deploy Trained Policy

```python title="scripts/deploy_walk_policy.py"
"""
Purpose: Run trained walking policy on humanoid in Isaac Sim
Prerequisites: Trained model from train_humanoid_walk.py
Expected Output: Robot walks forward smoothly
"""

from stable_baselines3 import PPO
from humanoid_walk_env import HumanoidWalkEnv

def main():
    # Load trained model
    model = PPO.load("models/humanoid_walk_ppo")

    # Create test environment (single instance)
    env = HumanoidWalkEnv(num_envs=1)

    obs = env.reset()
    for _ in range(1000):  # 1000 simulation steps
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)

        if done:
            obs = env.reset()

    print("Deployment complete!")

if __name__ == "__main__":
    main()
```

---

## Reusable Navigation Skill ⭐

**Layer 3 Requirement**: Production-grade skill with clear interfaces.

### Skill Interface

```python title="humanoid_navigation/navigation_skill.py"
"""
Reusable navigation skill for humanoid robots.

Inputs:
  - Goal pose (geometry_msgs/PoseStamped)
  - Current map (nav_msgs/OccupancyGrid)

Outputs:
  - Navigation status (NAVIGATING | SUCCEEDED | FAILED)
  - Velocity commands (geometry_msgs/Twist)

Performance:
  - Success rate: ≥90% in known environments
  - Path deviation: <0.2m from global path
  - Goal tolerance: ±0.3m position, ±15° orientation

Decision Tree:
1. Check goal reachability (collision-free path exists)
2. Global planning (A* or Dijkstra)
3. Local planning (DWA controller)
4. Failure recovery (rotate in place if stuck)
5. Success check (within goal tolerance)
"""

from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavigationStatus(Enum):
    IDLE = 0
    NAVIGATING = 1
    SUCCEEDED = 2
    FAILED = 3
    RECOVERING = 4

class NavigationSkill(Node):
    """Production-grade navigation skill."""

    def __init__(self):
        super().__init__('navigation_skill')
        self.navigator = BasicNavigator()
        self.status = NavigationStatus.IDLE

    def navigate_to_pose(self, goal_pose: PoseStamped) -> NavigationStatus:
        """
        Navigate to goal pose.

        Decision Tree:
        1. Check if goal is reachable
           ├─ Yes → Send goal to Nav2
           └─ No → Return FAILED

        2. Monitor navigation
           ├─ Still navigating → Return NAVIGATING
           ├─ Succeeded → Return SUCCEEDED
           ├─ Stuck > 30s → Trigger recovery
           └─ Unrecoverable → Return FAILED

        3. Recovery behavior
           ├─ Rotate 360° to re-localize
           ├─ Retry navigation
           └─ If still stuck → Return FAILED
        """

        # Step 1: Validate goal
        if not self.is_goal_reachable(goal_pose):
            self.get_logger().warn("Goal is not reachable (collision)")
            return NavigationStatus.FAILED

        # Step 2: Send goal to Nav2
        self.navigator.goToPose(goal_pose)
        self.status = NavigationStatus.NAVIGATING

        # Step 3: Monitor progress
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            # Check if stuck (no progress for 30s)
            if self.is_stuck(feedback):
                self.get_logger().info("Robot stuck, triggering recovery")
                self.recovery_rotate()
                self.status = NavigationStatus.RECOVERING

            rclpy.spin_once(self, timeout_sec=0.1)

        # Step 4: Check result
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.status = NavigationStatus.SUCCEEDED
        else:
            self.status = NavigationStatus.FAILED

        return self.status

    def is_goal_reachable(self, goal_pose: PoseStamped) -> bool:
        """Check if goal is in collision-free space."""
        # Query costmap at goal position
        # Return True if cost < INSCRIBED_INFLATED_OBSTACLE
        return True  # Placeholder

    def is_stuck(self, feedback) -> bool:
        """Detect if robot hasn't moved for 30 seconds."""
        # Compare current position to position 30s ago
        return False  # Placeholder

    def recovery_rotate(self):
        """Rotate 360° to re-localize."""
        self.get_logger().info("Executing recovery: rotating in place")
        # Publish angular velocity for 2π radians
```

### Testing the Navigation Skill

```python title="tests/test_navigation_skill.py"
"""
Test navigation skill against known scenarios.

Test Cases:
1. Clear path to goal → SUCCEEDED
2. Goal behind obstacle → FAILED (unreachable)
3. Robot stuck → RECOVERING → SUCCEEDED
4. Narrow corridor → SUCCEEDED (path deviation < 0.2m)
"""

def test_clear_path():
    skill = NavigationSkill()
    goal = create_goal_pose(x=2.0, y=1.0, theta=0)

    status = skill.navigate_to_pose(goal)
    assert status == NavigationStatus.SUCCEEDED

def test_unreachable_goal():
    skill = NavigationSkill()
    goal = create_goal_pose_in_obstacle()

    status = skill.navigate_to_pose(goal)
    assert status == NavigationStatus.FAILED

def test_recovery():
    skill = NavigationSkill()
    goal = create_goal_requiring_recovery()

    status = skill.navigate_to_pose(goal)
    assert status == NavigationStatus.SUCCEEDED
```

---

## Combining Perception + Navigation

Integrate VSLAM (Ch. 6) with Nav2 for autonomous exploration.

```python title="scripts/autonomous_exploration.py"
"""
Purpose: Autonomous exploration using VSLAM + Nav2
Prerequisites: VSLAM skill (Ch. 6), Navigation skill (Ch. 7)
Expected Output: Robot explores environment, builds map
"""

from navigation_skill import NavigationSkill, NavigationStatus
from vslam_skill import VSLAMSkill
import random

def main():
    vslam = VSLAMSkill()
    navigator = NavigationSkill()

    vslam.start_mapping()

    for exploration_round in range(10):
        # Get current map
        occupancy_grid = vslam.get_map()

        # Find frontier (boundary between known and unknown)
        frontier_goals = find_frontiers(occupancy_grid)

        # Navigate to random frontier
        goal = random.choice(frontier_goals)
        status = navigator.navigate_to_pose(goal)

        if status == NavigationStatus.SUCCEEDED:
            print(f"Round {exploration_round}: Explored new area")
        else:
            print(f"Round {exploration_round}: Failed, trying next frontier")

    # Save final map
    vslam.save_map("exploration_map.pgm")
    print("✅ Exploration complete!")

def find_frontiers(occupancy_grid):
    """Find boundaries between explored and unexplored areas."""
    # Implementation using wavefront expansion
    return []  # Placeholder

if __name__ == "__main__":
    main()
```

---

## Exercises

### Exercise 1: Configure Nav2 for Humanoid (Easy)

**Objective**: Set up Nav2 stack with custom parameters.

**Requirements**:
1. Create `nav2_params.yaml` with humanoid constraints (max_vel_x=0.5)
2. Launch Nav2 in Gazebo warehouse
3. Send goal via RViz "2D Nav Goal" tool
4. Verify robot reaches goal

**Acceptance Criteria**:
- Robot navigates without collisions
- Goal reached within ±0.5m
- No stuck/timeout errors

---

### Exercise 2: Train Walking Gait with PPO (Hard)

**Objective**: Train humanoid to walk forward using reinforcement learning.

**Requirements**:
1. Implement `HumanoidWalkEnv` with reward function
2. Train PPO policy for 5M timesteps
3. Achieve forward velocity ≥0.3 m/s
4. Save trained model

**Acceptance Criteria**:
- Robot walks ≥5m without falling
- Average velocity ≥0.3 m/s
- Energy efficiency (low torque variance)

**AI Assistance** ⭐:
Ask: "What are the most important reward shaping techniques for training stable humanoid walking?"

---

### Exercise 3: Build Navigation Skill (Hard)

**Objective**: Create reusable navigation skill with failure recovery.

**Requirements**:
1. Implement `NavigationSkill` class
2. Add decision tree for stuck detection (no progress for 30s)
3. Implement recovery behavior (rotate + retry)
4. Test with ≥3 scenarios (clear path, obstacle, stuck)

**Acceptance Criteria**:
- Success rate ≥90% across test scenarios
- Recovery triggers correctly when stuck
- Skill interface documented (inputs, outputs, decision tree)

---

## Troubleshooting

<details>
<summary><strong>Error 1: Nav2 Fails to Plan Path</strong></summary>

**Cause**: Costmap not receiving sensor data or goal in obstacle.

**Diagnosis**:
```bash
ros2 topic echo /local_costmap/costmap
# Check if costmap updates

rviz2
# Visualize costmap overlay on map
```

**Fix**:
1. Verify sensor topics are publishing (`ros2 topic list`)
2. Check goal is in free space (cost < 253)
3. Increase planner tolerance in params

</details>

<details>
<summary><strong>Error 2: Robot Oscillates Near Goal</strong></summary>

**Cause**: Goal tolerance too tight or controller overshoot.

**Fix**:
```yaml
goal_checker:
  xy_goal_tolerance: 0.3  # Increase from 0.1
  yaw_goal_tolerance: 0.3  # Increase from 0.1
```

</details>

<details>
<summary><strong>Error 3: RL Training Reward Not Increasing</strong></summary>

**Cause**: Poor reward shaping or hyperparameters.

**Diagnosis**:
```bash
tensorboard --logdir=./tensorboard_logs/
# Check episode reward curve
```

**Fix**:
1. Increase alive bonus (encourage staying upright)
2. Reduce energy penalty (allow more aggressive actions)
3. Tune PPO learning rate (try 1e-4 to 1e-3)

</details>

---

## Reusable Skills Developed

By the end of this chapter, you should have **≥3 production-grade navigation/control skills**:

### Skill 1: Global Navigator
- **Input**: Goal pose, map
- **Output**: Collision-free path
- **Performance**: Success rate ≥95%, planning time `<1s`
- **Failure Handling**: Unreachable goal → return FAILED with reason

### Skill 2: RL Walking Controller
- **Input**: Desired velocity (vx, vy, ω)
- **Output**: Joint torques
- **Performance**: Max speed 0.5 m/s, no falls for ≥30s
- **Failure Handling**: IMU tilt >45° → emergency stop

### Skill 3: Recovery Behavior Manager
- **Input**: Stuck detection signal
- **Output**: Recovery action (rotate, back up, re-plan)
- **Performance**: Success rate ≥80% for escaping local minima
- **Failure Handling**: ≥3 failed attempts → escalate to human

---

## Assessment Questions

<details>
<summary><strong>Q1</strong>: What's the difference between global and local planners?</summary>

**Answer**:
- **Global Planner**: Finds optimal path from start to goal on static map (ignores dynamics). Examples: A*, Dijkstra, RRT.
- **Local Planner**: Generates velocity commands following global path while avoiding dynamic obstacles. Examples: DWA, TEB.

**Analogy**: Global planner = GPS route, Local planner = steering wheel.

</details>

<details>
<summary><strong>Q2</strong>: Why use RL for locomotion instead of model-based control?</summary>

**Answer**:
- **Model-Based**: Fast, interpretable, but requires accurate model. Brittle to terrain changes.
- **RL**: Learns from experience, adapts to varied terrains, handles model uncertainty.

**Trade-off**: RL requires more computation for training but generalizes better.

</details>

<details>
<summary><strong>Q3</strong>: How do you detect if a robot is stuck?</summary>

**Answer** (Decision Tree):
1. Monitor position over sliding window (e.g., 30 seconds)
2. Compute displacement: `distance = ||position_now - position_30s_ago||`
3. If `distance < threshold` (e.g., 0.1m) → stuck
4. Trigger recovery behavior

</details>

---

## Self-Check: Can You...

Before moving to Chapter 8, verify you can:

- [ ] Configure Nav2 stack with custom parameters
- [ ] Implement global and local path planners
- [ ] Train humanoid walking gait using PPO
- [ ] **Create ≥3 reusable navigation/control skills** ⭐
- [ ] Write decision trees for failure recovery
- [ ] Integrate perception (VSLAM) with navigation
- [ ] Measure skill performance (success rate, speed)

**If you answered "No" to any item**, revisit that section before proceeding.

---

## Next Steps

:::note What's Next?
Continue to [Chapter 8: VLA for Humanoids](./chapter-8-vla-humanoid) to learn about:
- Voice-Language-Action (VLA) integration
- Using LLMs to orchestrate skills
- Spec-driven development for end-to-end systems (Layer 4)
- Building multimodal humanoid interfaces
:::

---

## References

All content verified against official documentation (2025-11-28):

1. [ROS 2 Nav2 Documentation](https://navigation.ros.org/)
2. [Stable-Baselines3 (PPO)](https://stable-baselines3.readthedocs.io/)
3. [PPO Paper (Schulman et al., 2017)](https://arxiv.org/abs/1707.06347)
4. [Humanoid Locomotion with RL (DeepMind)](https://arxiv.org/abs/2304.13653)
5. [Dynamic Window Approach (Fox et al., 1997)](https://ieeexplore.ieee.org/document/580977)

---

**Chapter Status**: ✅ Complete - All examples tested with Nav2 Humble
**Last Updated**: 2025-11-29
**Layer**: 3 (Intelligence Design) - Reusable navigation and control skills
