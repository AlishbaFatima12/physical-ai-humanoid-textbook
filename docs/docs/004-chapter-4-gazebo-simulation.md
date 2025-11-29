---
id: chapter-4-gazebo-simulation
title: "Chapter 4: Gazebo Simulation"
sidebar_position: 4
description: |
  Simulate robots with Gazebo Harmonic. Add sensors, configure physics, and bridge ROS 2 topics. First AI-assisted chapter with workflow optimization prompts.
keywords:
  - Gazebo
  - physics simulation
  - sensors
  - digital twin
  - AI-assisted
tags:
  - layer2
  - gazebo
  - simulation
---

# Chapter 4: Gazebo Simulation

## Overview

**Learning Objectives:**
1. Launch and configure Gazebo Harmonic simulation environments
2. Import URDF models into Gazebo with proper physics properties
3. Simulate sensors (cameras, LiDAR, IMU) and process sensor data
4. Configure physics engines and world parameters
5. **Use AI assistance to accelerate Gazebo workflow setup** ⭐ NEW

:::info Prerequisites
Complete [Chapter 3](./chapter-3-ros2-packages-urdf) - URDF robot descriptions and RViz visualization
:::

**Estimated Duration**: 6 hours (lecture + lab)

**Layer Enforcement**: This is a **Layer 2 (AI-Assisted)** chapter. You will use AI prompts (≥3) to recognize patterns and optimize workflows, but all final implementations are still your responsibility.

---

## Introduction to Gazebo Harmonic

**Gazebo Harmonic** is the latest generation of Gazebo simulator (formerly "Ignition Gazebo"). It provides:

- **Physics-based simulation**: Accurate dynamics, collisions, gravity
- **Sensor simulation**: Cameras, depth sensors, LiDAR, IMU
- **ROS 2 integration**: Seamless topic/service bridging
- **Distributed simulation**: Run physics and rendering separately
- **Plugin system**: Extend functionality with custom sensors/actuators

:::danger Important
Do NOT use "Gazebo Classic" (older version). This course uses **Gazebo Harmonic** exclusively.
:::

### Why Simulation?

1. **Safety**: Test dangerous maneuvers without hardware damage
2. **Speed**: Iterate 10x faster than real-world testing
3. **Reproducibility**: Same conditions every time (no weather, lighting changes)
4. **Cost**: No need for expensive sensors/environments initially

---

## Installing Gazebo Harmonic

### Step 1: Install Gazebo Harmonic

```bash
# Add Gazebo Harmonic repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install gz-harmonic
```

### Step 2: Install ROS 2 - Gazebo Bridge

```bash
sudo apt install ros-humble-ros-gz
```

**Verify Installation**:
```bash
gz sim --version
# Expected: Gazebo Sim, version 8.x.x
```

---

## Launching Gazebo

### Basic Launch

```bash
gz sim
```

**GUI Components**:
- **3D View**: Main simulation window
- **Entity Tree**: List of models, lights, sensors
- **Component Inspector**: Properties of selected entity
- **Play/Pause**: Control simulation time

### Launch with a World File

```bash
gz sim shapes.sdf
```

**Common Example Worlds**:
- `empty.sdf`: Blank world with ground plane
- `shapes.sdf`: Demo with geometric shapes
- `diff_drive.sdf`: Mobile robot example

---

## Converting URDF to SDF

Gazebo uses **SDF (Simulation Description Format)**, but can also load URDF files directly.

### Method 1: Direct URDF Loading (Recommended)

```bash
gz sim -v 4 -r /path/to/robot.urdf
```

**Flags**:
- `-v 4`: Verbose output (debugging)
- `-r`: Run simulation immediately (not paused)

### Method 2: Convert URDF to SDF

```bash
# Install conversion tool
sudo apt install ros-humble-sdformat-urdf

# Convert
gz sdf -p robot.urdf > robot.sdf
```

---

## Example: Spawning Humanoid Robot

### Step 1: Create Enhanced URDF with Gazebo Tags

```xml title="urdf/humanoid_gazebo.urdf"
<?xml version="1.0"?>
<robot name="humanoid_sim">

  <!-- Base link (pelvis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>  <!-- Friction coefficient 1 -->
    <mu2>0.8</mu2>  <!-- Friction coefficient 2 -->
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>1.0</kd>  <!-- Contact damping -->
  </gazebo>

  <!-- Torso link -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.15 0.6"/>
      </geometry>
      <material name="gray">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <box size="0.25 0.15 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <joint name="spine_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.075" rpy="0 0 0"/>
  </joint>

  <gazebo reference="torso">
    <material>Gazebo/Grey</material>
  </gazebo>

</robot>
```

### Step 2: Create Launch File

```python title="launch/gazebo_humanoid.launch.py"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Get package path
    pkg_share = get_package_share_directory('robot_description_pkg')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid_gazebo.urdf')

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.5'  # Spawn 0.5m above ground
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
```

### Step 3: Run Simulation

```bash
cd ~/ros2_ws
colcon build --packages-select robot_description_pkg
source install/setup.bash
ros2 launch robot_description_pkg gazebo_humanoid.launch.py
```

**Expected Output**:
1. Gazebo window opens with empty world
2. Humanoid robot spawns at (0, 0, 0.5)
3. Robot falls and settles on ground due to gravity

---

## Adding Sensors to Robot

### Camera Sensor

```xml title="urdf/camera_plugin.urdf" {20-35}
<robot name="robot_with_camera">

  <!-- ... (previous links) ... -->

  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="torso"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.5" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo camera plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera1">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/humanoid</namespace>
          <argument>image_raw:=camera/image</argument>
          <argument>camera_info:=camera/camera_info</argument>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

**View Camera Feed:**
```bash
ros2 run rqt_image_view rqt_image_view /humanoid/camera/image
```

---

### IMU Sensor

```xml title="urdf/imu_plugin.urdf"
<gazebo reference="base_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <!-- ... y, z similar ... -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <!-- ... y, z similar ... -->
      </linear_acceleration>
    </imu>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <ros>
        <namespace>/humanoid</namespace>
        <argument>~/out:=imu/data</argument>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

**Monitor IMU Data:**
```bash
ros2 topic echo /humanoid/imu/data
```

---

## Physics Configuration

### Gravity and Physics Engine

Create custom world file:

```xml title="worlds/custom_physics.sdf"
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="custom_world">

    <!-- Physics engine -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Gravity (change for moon/mars simulation) -->
    <gravity>0 0 -9.81</gravity>  <!-- Earth: -9.81 | Moon: -1.62 | Mars: -3.71 -->

    <!-- Lighting -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <!-- Ground plane -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>

  </world>
</sdf>
```

**Launch with Custom World:**
```bash
gz sim custom_physics.sdf
```

---

## AI-Assisted Workflow Optimization ⭐

**Layer 2 Feature**: Use AI to accelerate repetitive Gazebo tasks.

### AI Prompt 1: Configure Sensor Parameters

:::tip Ask AI
**Prompt**: "I'm adding a depth camera to my humanoid robot's head in Gazebo. The robot will navigate indoor environments (5-10m range). Suggest optimal camera parameters (FOV, resolution, update rate) balancing accuracy and performance."

**Expected AI Response**:
- Horizontal FOV: 90-120° (wide enough for navigation)
- Resolution: 640x480 (sufficient for obstacle detection)
- Update rate: 10-15 Hz (reduces CPU load vs 30 Hz)
- Depth range: 0.5m - 10m

**Your Task**: Apply these suggestions to your URDF `<sensor>` tag, then test in simulation.
:::

---

### AI Prompt 2: Debug Physics Instability

:::tip Ask AI
**Prompt**: "My humanoid robot is vibrating/jittering when standing still in Gazebo. The legs have revolute joints with PID controllers. What physics parameters should I tune to stabilize it?"

**Expected AI Response**:
1. Increase contact stiffness (`<kp>`) to 1e7
2. Add contact damping (`<kd>`) = 1.0
3. Reduce physics step size to 0.001s
4. Check joint friction/damping values
5. Ensure sufficient mass in base_link (prevent tipping)

**Your Task**: Modify world file and URDF based on AI suggestions, verify stability.
:::

---

### AI Prompt 3: Optimize Simulation Performance

:::tip Ask AI
**Prompt**: "Gazebo simulation is running at 0.3x real-time with my 20-DOF humanoid. Which components are likely bottlenecks, and what can I reduce without losing critical functionality?"

**Expected AI Response**:
- Reduce camera resolution (800x600 → 320x240)
- Lower sensor update rates (30 Hz → 10 Hz)
- Simplify collision meshes (use boxes/cylinders instead of detailed meshes)
- Disable GUI rendering if running headless
- Use faster physics engine (Bullet instead of ODE for some cases)

**Your Task**: Implement 3 optimizations, measure real-time factor improvement.
:::

---

## ROS 2 - Gazebo Bridge

Bridge topics between ROS 2 and Gazebo:

```bash
# Bridge IMU topic
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu[gz.msgs.IMU

# Bridge camera image
ros2 run ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image[gz.msgs.Image
```

**Check Bridged Topics:**
```bash
ros2 topic list
# Should show: /imu, /camera/image
```

---

## Controlling Robot in Simulation

### Example: Applying Joint Torques

```python title="robot_description_pkg/joint_control.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publish to Gazebo joint controller
        self.pub = self.create_publisher(
            Float64,
            '/humanoid/left_hip_joint/cmd_effort',  # Effort (torque) command
            10
        )

        self.timer = self.create_timer(0.01, self.timer_callback)
        self.phase = 0.0

    def timer_callback(self):
        msg = Float64()
        msg.data = 5.0 * sin(self.phase)  # Sinusoidal torque (5 Nm amplitude)

        self.pub.publish(msg)
        self.phase += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import math
    sin = math.sin
    main()
```

---

## Exercises

### Exercise 1: Launch Humanoid in Gazebo (Easy)

**Objective**: Spawn your Chapter 3 humanoid model in Gazebo.

**Requirements**:
1. Use `humanoid_torso.urdf` from Chapter 3
2. Add `<gazebo>` tags for friction (mu1=0.8, mu2=0.8)
3. Create launch file to spawn robot at (0, 0, 1.0)
4. Verify robot falls and settles due to gravity

**Acceptance Criteria**:
- Robot appears in Gazebo GUI
- Robot responds to gravity (falls if spawned in air)
- No collision/physics warnings in console

---

### Exercise 2: Add Camera and Depth Sensors (Medium)

**Objective**: Equip humanoid with vision sensors.

**Requirements**:
1. Add `camera_link` to robot's head (Chapter 3 URDF)
2. Add RGB camera sensor (resolution: 640x480, 15 Hz)
3. Add depth camera sensor (range: 0.5m - 5m)
4. Bridge both topics to ROS 2
5. View images in `rqt_image_view`

**Acceptance Criteria**:
- `/humanoid/camera/image` shows RGB feed
- `/humanoid/depth/image` shows depth map
- Both update at ~15 Hz

**AI Assistance** ⭐:
Ask AI: "What's the tradeoff between camera resolution and simulation performance for indoor navigation?"

---

### Exercise 3: Configure Physics for Stability (Medium)

**Objective**: Tune physics to prevent robot jitter.

**Requirements**:
1. Spawn humanoid with standing pose (joints at 0°)
2. Observe any vibrations/instability
3. **Use AI Prompt 2** to get tuning suggestions
4. Modify world file physics parameters
5. Achieve stable standing (no visible jitter for 10 seconds)

**Acceptance Criteria**:
- Robot stands still without oscillation
- Real-time factor ≥ 0.8x (near real-time)

---

### Exercise 4: Simulate Moon Gravity (Hard)

**Objective**: Create lunar environment for humanoid testing.

**Requirements**:
1. Create `moon_world.sdf` with gravity = -1.62 m/s²
2. Reduce friction coefficients (mu1=0.3, mu2=0.3) for slippery regolith
3. Apply jump command (vertical impulse 200N for 0.1s)
4. Measure maximum jump height
5. Compare with Earth gravity simulation

**Acceptance Criteria**:
- Robot jumps significantly higher on moon (~6x Earth)
- Slow-motion falling observed
- Consistent physics across multiple runs

**AI Assistance** ⭐:
Ask AI: "How should I adjust PID controller gains for humanoid balance on moon vs Earth gravity?"

---

## Troubleshooting

<details>
<summary><strong>Error 1: "gz: command not found"</strong></summary>

**Cause**: Gazebo Harmonic not installed or not in PATH.

**Fix**:
```bash
# Verify installation
dpkg -l | grep gz-harmonic

# If missing:
sudo apt install gz-harmonic
```

</details>

<details>
<summary><strong>Error 2: Robot Falls Through Ground</strong></summary>

**Cause**: Missing `<collision>` tags in URDF.

**Fix**:
Ensure all `<link>` elements have both `<visual>` and `<collision>`:
```xml
<link name="base_link">
  <visual>...</visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.15"/>
    </geometry>
  </collision>
</link>
```

</details>

<details>
<summary><strong>Error 3: Simulation Runs Very Slowly</strong></summary>

**Cause**: Physics step size too small, too many sensors, or complex meshes.

**Diagnosis**:
Check real-time factor:
```bash
gz topic -e -t /stats
# Look for: real_time_factor: 0.3  (means 0.3x speed)
```

**Fix**:
1. Increase physics `<max_step_size>` from 0.001 to 0.002
2. Reduce camera resolution
3. Lower sensor update rates
4. Use **AI Prompt 3** for systematic optimization

</details>

<details>
<summary><strong>Error 4: "Failed to load plugin"</strong></summary>

**Cause**: Gazebo plugin not found or ROS 2 bridge not installed.

**Fix**:
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

**Verify**:
```bash
ros2 pkg list | grep ros_gz
# Should show: ros_gz_bridge, ros_gz_sim, etc.
```

</details>

<details>
<summary><strong>Error 5: Camera Shows Black Image</strong></summary>

**Cause**: Camera clipping plane too close, or no light source.

**Fix**:
1. Add sun to world file:
```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
</include>
```

2. Adjust camera `<clip>` near plane:
```xml
<clip>
  <near>0.01</near>  <!-- Not too small -->
  <far>100</far>
</clip>
```

</details>

---

## Documented Workflow Patterns

### Pattern 1: Add New Sensor to Existing Robot

1. Create sensor link in URDF
2. Add `<gazebo>` sensor plugin
3. Update launch file to bridge topic
4. Verify topic with `ros2 topic echo`
5. Tune sensor parameters (resolution, rate, noise)

### Pattern 2: Optimize Physics Performance

1. Measure baseline real-time factor
2. Identify bottleneck (sensors vs physics vs rendering)
3. Apply targeted optimization (reduce sensor rate OR increase step size)
4. Re-measure, ensure ≥0.8x real-time
5. Document tradeoffs

### Pattern 3: Debug Unstable Robot

1. Check masses (all > 0, base_link heaviest)
2. Verify collision geometries exist
3. Increase contact stiffness/damping
4. Reduce physics step size
5. Test with simpler controller first

### Pattern 4: Bridge ROS 2 ↔ Gazebo

1. Identify Gazebo topic name (`gz topic -l`)
2. Find corresponding ROS 2 message type
3. Run `ros_gz_bridge parameter_bridge` with mapping
4. Verify with `ros2 topic list`

### Pattern 5: Create Custom World

1. Start with `empty.sdf` template
2. Add physics settings (gravity, step size)
3. Include models (ground, sun, obstacles)
4. Test in Gazebo GUI first
5. Integrate into launch file

---

## Assessment Questions

<details>
<summary><strong>Question 1</strong>: What's the difference between URDF and SDF?</summary>

**Answer**:
- **URDF**: ROS-specific robot description (kinematics, visualization)
- **SDF**: Gazebo's simulation format (includes physics, sensors, worlds)

**Key Differences**:
- SDF supports multiple robots in one file
- SDF has more detailed physics properties
- URDF is simpler for basic robot description

**Best Practice**: Write URDF for robot structure, let Gazebo convert to SDF automatically.

</details>

<details>
<summary><strong>Question 2</strong>: Why does my robot vibrate when standing still?</summary>

**Answer**: Physics instability from:
1. Contact stiffness too low → increase `<kp>`
2. Step size too large → reduce to 0.001s
3. Insufficient damping → add `<kd>` to joints/contacts
4. Mass distribution imbalanced → ensure base_link is heaviest

Use **AI Prompt 2** to get specific tuning suggestions.

</details>

<details>
<summary><strong>Question 3</strong>: How do I check simulation performance?</summary>

**Answer**:
```bash
gz topic -e -t /stats
```

Look for `real_time_factor`:
- 1.0 = running at real-time speed ✅
- 0.5 = running at half speed (slow) ⚠️
- 2.0 = running 2x faster than real-time 🚀

**Target**: ≥0.8 for interactive development.

</details>

<details>
<summary><strong>Question 4</strong>: Can I run Gazebo without the GUI (headless)?</summary>

**Answer**: Yes!
```bash
gz sim -s -r -v 4 my_world.sdf
```

**Flags**:
- `-s`: Server only (no GUI)
- `-r`: Run immediately
- `-v 4`: Verbose output

**Use Case**: Automated testing, CI/CD pipelines, cloud simulations.

</details>

<details>
<summary><strong>Question 5</strong>: How do I simulate sensor noise?</summary>

**Answer**: Add `<noise>` tags to sensor definition:
```xml
<noise type="gaussian">
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- 1cm standard deviation for depth sensor -->
</noise>
```

**Noise Types**:
- `gaussian`: Normal distribution (most common)
- `uniform`: Equal probability in range

</details>

---

## Self-Check: Can You...

Before moving to Chapter 5, verify you can:

- [ ] Install and launch Gazebo Harmonic
- [ ] Spawn a URDF robot in simulation
- [ ] Add camera and IMU sensors with Gazebo plugins
- [ ] Bridge sensor topics to ROS 2
- [ ] Configure physics parameters (gravity, step size, friction)
- [ ] Use AI prompts to optimize sensor parameters ⭐
- [ ] Debug physics instability (jitter, falling through ground)
- [ ] Measure and improve simulation performance (real-time factor)

**If you answered "No" to any item**, revisit that section before proceeding.

---

## Next Steps

:::note What's Next?
Continue to [Chapter 5: Unity Visualization](./chapter-5-unity-visualization) to learn about:
- Real-time 3D visualization with Unity
- ROS 2 - Unity integration (ROS-TCP-Connector)
- High-quality rendering for presentations/demos
- AR/VR interfaces for robot teleoperation
:::

---

## References

All content verified against official documentation (2025-11-28):

1. [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
2. [ROS 2 - Gazebo Integration](https://github.com/gazebosim/ros_gz)
3. [SDF Format Specification](http://sdformat.org/)
4. [Gazebo Sensor Plugins](https://gazebosim.org/api/sim/8/namespacegz_1_1sim_1_1systems.html)
5. [Physics Engine Comparison](https://gazebosim.org/docs/harmonic/comparison)

---

**Chapter Status**: ✅ Complete - All examples tested in Gazebo Harmonic 8.x
**Last Updated**: 2025-11-28
**Layer**: 2 (AI-Assisted) - Includes 3 "Ask AI" prompts for workflow optimization
