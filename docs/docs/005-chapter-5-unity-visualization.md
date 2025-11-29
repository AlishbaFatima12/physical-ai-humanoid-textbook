---
id: chapter-5-unity-visualization
title: "Chapter 5: Unity Visualization"
sidebar_position: 5
description: |
  Create photorealistic robot visualizations with Unity. Connect to ROS 2, import URDF models, and build stunning demo environments.
keywords:
  - Unity
  - visualization
  - ROS 2
  - digital twin
  - rendering
tags:
  - layer2
  - unity
  - visualization
---

# Chapter 5: Unity Visualization

## Overview

**Learning Objectives:**
1. Set up Unity 2022.3 LTS for robotics visualization
2. Connect Unity to ROS 2 using Unity Robotics Hub
3. Visualize robot state and sensor data in real-time
4. Create photorealistic rendering environments
5. **Use AI to streamline Unity-ROS 2 integration workflow** ⭐

:::info Prerequisites
Complete [Chapter 4](./chapter-4-gazebo-simulation) - Gazebo simulation and ROS 2 bridging
:::

**Estimated Duration**: 5 hours (lecture + lab)

**Layer Enforcement**: **Layer 2 (AI-Assisted)**. Use AI prompts (≥3) to optimize Unity workflows.

---

## Why Unity for Robotics?

**Unity vs Gazebo**:
- **Gazebo**: Physics simulation, sensor accuracy, testing
- **Unity**: Photorealistic rendering, presentations, marketing demos

**Use Cases**:
1. **Investor Demos**: Stunning visuals for funding pitches
2. **Conference Presentations**: High-quality videos
3. **User Interfaces**: Real-time robot monitoring dashboards
4. **Documentation**: Tutorial videos with clear visuals

---

## Installing Unity

### Step 1: Download Unity Hub

```bash
# Download from https://unity.com/download
# Install Unity Hub (graphical installer)
```

### Step 2: Install Unity 2022.3 LTS

1. Open Unity Hub
2. Click "Installs" → "Install Editor"
3. Select **2022.3 LTS** (Long Term Support)
4. Add Modules:
   - Linux Build Support
   - Documentation

---

## Unity Robotics Hub Setup

### Step 1: Create New Unity Project

1. Unity Hub → "New Project"
2. Template: **3D (URP)** (Universal Render Pipeline)
3. Project Name: `Humanoid_Visualization`
4. Create

### Step 2: Install Unity Robotics Hub

```
Window → Package Manager → Add package from git URL:
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

### Step 3: Install URDF Importer

```
Add package from git URL:
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

---

## Connecting Unity to ROS 2

### ROS 2 Side: Start TCP Endpoint

```bash
sudo apt install ros-humble-ros-tcp-endpoint
source /opt/ros/humble/setup.bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Expected Output**:
```
[INFO] Starting ROS TCP Endpoint at 0.0.0.0:10000
```

### Unity Side: Configure Connection

1. **Robotics** → **ROS Settings**
2. Set:
   - Protocol: **ROS 2**
   - ROS IP Address: `127.0.0.1` (localhost)
   - ROS Port: `10000`
3. Click "**Connect**"

**Verify**: Unity console shows "Connected to ROS"

---

## Importing URDF to Unity

### Step 1: Prepare URDF

Ensure your humanoid URDF has materials:

```xml title="humanoid_unity.urdf"
<gazebo reference="base_link">
  <material>Gazebo/White</material>
</gazebo>
```

### Step 2: Import URDF

1. **Assets** → **Import Robot from URDF**
2. Select `humanoid_torso.urdf`
3. Import Settings:
   - Axis Type: **Y Axis**
   - Mesh Decomposer: **VHACD**
4. Click "**Import Robot**"

**Result**: Robot appears in Hierarchy as GameObject

---

## Visualizing Robot State

### Subscribe to Joint States

**Create C# Script**: `JointStateSubscriber.cs`

```csharp title="Assets/Scripts/JointStateSubscriber.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ArticulationBody[] joints;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<MJointStateMsg>("/joint_states");

        ros.Subscribe<MJointStateMsg>("/joint_states", JointStateCallback);

        // Get all articulation bodies (Unity's joint representation)
        joints = GetComponentsInChildren<ArticulationBody>();
    }

    void JointStateCallback(MJointStateMsg jointStateMsg)
    {
        for (int i = 0; i < jointStateMsg.name.Length; i++)
        {
            string jointName = jointStateMsg.name[i];
            float position = (float)jointStateMsg.position[i];

            // Find matching Unity joint
            ArticulationBody joint = System.Array.Find(joints,
                j => j.name.Contains(jointName));

            if (joint != null)
            {
                var drive = joint.xDrive;
                drive.target = position * Mathf.Rad2Deg; // Convert to degrees
                joint.xDrive = drive;
            }
        }
    }
}
```

**Attach Script**: Drag onto robot GameObject in Hierarchy

---

## AI-Assisted Workflow ⭐

### AI Prompt 1: Optimize Unity Performance

:::tip Ask AI
**Prompt**: "My Unity scene with a 20-DOF humanoid robot is running at 15 FPS. What are the top 5 optimization techniques for real-time ROS 2 visualization?"

**Expected AI Response**:
1. Reduce shadow quality (Realtime → Baked)
2. Use LOD (Level of Detail) for distant objects
3. Limit articulation bodies to essential joints
4. Reduce post-processing effects
5. Use occlusion culling

**Your Task**: Implement 3 optimizations, measure FPS improvement.
:::

---

### AI Prompt 2: Unity-ROS Message Mapping

:::tip Ask AI
**Prompt**: "I need to display IMU data (orientation, angular velocity) from ROS 2 sensor_msgs/Imu in Unity. Suggest the best Unity components and scripts to visualize this data."

**Expected AI Response**:
- Orientation → Apply quaternion to Transform.rotation
- Angular velocity → Show as colored arrow (Gizmos)
- Linear acceleration → Particle system or trail renderer

**Your Task**: Create C# script to visualize IMU data.
:::

---

### AI Prompt 3: Lighting for Photorealism

:::tip Ask AI
**Prompt**: "I'm creating a demo video of my humanoid robot in Unity. Recommend lighting setup (types, angles, intensities) for professional-looking indoor scene."

**Expected AI Response**:
- Key Light: Directional, 45° angle, intensity 1.5
- Fill Light: Area light, opposite side, intensity 0.5
- Rim Light: Spot light behind robot, intensity 1.0
- HDRI Skybox for ambient lighting

**Your Task**: Implement 3-point lighting in your scene.
:::

---

## Creating Demo Scene

### Step 1: Add Environment

1. **GameObject** → **3D Object** → **Plane** (ground)
2. Scale: (10, 1, 10)
3. Add Material:
   - Albedo: Gray (#808080)
   - Smoothness: 0.3

### Step 2: Lighting

```
Lighting → Environment → Skybox Material → None
Lighting → Ambient → Source: Color (#404040)
```

Add Directional Light:
- Rotation: (50, -30, 0)
- Intensity: 1.2
- Shadows: **Soft Shadows**

### Step 3: Camera Setup

1. Main Camera position: (2, 1.5, -3)
2. Rotation: (10, -30, 0)
3. Add **Cinemachine Virtual Camera**:
   - Follow: Robot base_link
   - Look At: Robot torso

---

## Exercises

### Exercise 1: Connect Unity to ROS 2 (Easy)

**Requirements**:
1. Install Unity Robotics Hub
2. Start ROS 2 TCP endpoint
3. Connect Unity to ROS 2
4. Verify connection in Unity console

**Acceptance Criteria**:
- Unity console shows "Connected to ROS"
- No timeout errors

---

### Exercise 2: Visualize Robot Joint States (Medium)

**Requirements**:
1. Import humanoid URDF to Unity
2. Create `JointStateSubscriber.cs` script
3. Publish joint states from ROS 2:
   ```bash
   ros2 topic pub /joint_states sensor_msgs/JointState ...
   ```
4. Observe robot moving in Unity

**Acceptance Criteria**:
- Robot joints move in sync with ROS 2 messages
- No lag > 100ms

**AI Assistance** ⭐:
Ask AI: "How should I handle joint name mismatches between URDF and Unity ArticulationBody names?"

---

### Exercise 3: Create Professional Demo Scene (Medium)

**Requirements**:
1. Add environment (floor, walls, props)
2. Implement 3-point lighting
3. Add camera with Cinemachine follow
4. Record 30-second video

**Acceptance Criteria**:
- Scene looks professional (no default gray background)
- Robot is well-lit, no harsh shadows
- Video exports at 1080p 60fps

---

### Exercise 4: Sync Gazebo & Unity (Hard)

**Requirements**:
1. Run Gazebo simulation with humanoid
2. Bridge `/joint_states` to Unity
3. Robot moves identically in both
4. Display camera feed from Gazebo in Unity UI

**Acceptance Criteria**:
- Both simulations synchronized (`<50ms` delay)
- Camera feed shows in Unity Canvas

**AI Assistance** ⭐:
Ask AI: "What's causing 200ms latency between Gazebo and Unity, and how can I reduce it?"

---

## Troubleshooting

<details>
<summary><strong>Error 1: "Could not connect to ROS"</strong></summary>

**Cause**: TCP endpoint not running or wrong IP.

**Fix**:
```bash
# Verify endpoint is running
ros2 node list
# Should show: /ros_tcp_endpoint

# Check IP address
hostname -I
```

In Unity ROS Settings, use the correct IP address.

</details>

<details>
<summary><strong>Error 2: Robot Explodes on Import</strong></summary>

**Cause**: Incorrect URDF axis or mass values.

**Fix**:
1. Check URDF masses (all > 0)
2. Use **Y Axis** for Unity import
3. Reduce collision mesh complexity

</details>

<details>
<summary><strong>Error 3: Low FPS in Play Mode</strong></summary>

**Cause**: Too many shadows, post-processing, or complex meshes.

**Fix**: Use **AI Prompt 1** for systematic optimization.

</details>

---

## Workflow Patterns

### Pattern 1: URDF → Unity Import
1. Validate URDF with `check_urdf`
2. Set Unity import axis to Y
3. Use VHACD mesh decomposer
4. Verify joint names match ROS 2

### Pattern 2: ROS 2 Message Visualization
1. Identify ROS message type
2. Create C# script with `ROSConnection.Subscribe()`
3. Map ROS data to Unity components
4. Test with `ros2 topic pub`

### Pattern 3: Scene Optimization
1. Measure baseline FPS
2. Profile with Unity Profiler
3. Apply targeted optimizations
4. Re-measure FPS

### Pattern 4: Lighting Setup
1. Disable ambient lighting
2. Add key + fill + rim lights
3. Bake static shadows
4. Add HDRI skybox

### Pattern 5: Camera Follow
1. Install Cinemachine package
2. Create Virtual Camera
3. Set Follow/Look At targets
4. Tune damping for smooth motion

---

## Assessment Questions

<details>
<summary><strong>Q1</strong>: When should I use Unity vs Gazebo?</summary>

**Answer**:
- **Gazebo**: Physics testing, sensor accuracy, algorithm validation
- **Unity**: Demos, presentations, UI development, marketing videos

</details>

<details>
<summary><strong>Q2</strong>: How do I convert quaternion from ROS 2 to Unity?</summary>

**Answer**:
ROS 2 and Unity have different coordinate systems:
```csharp
// ROS 2: (x, y, z, w)
// Unity: (x, y, z, w) but different axes

Quaternion unityQuat = new Quaternion(
    -rosQuat.y,  // Flip Y
    rosQuat.z,
    rosQuat.x,
    rosQuat.w
);
```

</details>

<details>
<summary><strong>Q3: Can Unity simulate physics like Gazebo?</strong></summary>

**Answer**: Unity has physics (PhysX), but:
- Less accurate than Gazebo for robotics
- Missing robot-specific features (contact sensors, JointController)
- **Best Practice**: Simulate in Gazebo, visualize in Unity

</details>

---

## Self-Check

Before Chapter 6, verify you can:

- [ ] Install Unity 2022.3 LTS and Robotics Hub
- [ ] Import URDF to Unity
- [ ] Connect Unity to ROS 2 TCP endpoint
- [ ] Subscribe to `/joint_states` and visualize
- [ ] Use AI prompts for Unity optimization ⭐
- [ ] Create professional demo scenes with lighting
- [ ] Record and export videos

---

## Next Steps

:::note What's Next?
Continue to [Chapter 6: Isaac Perception](./chapter-6-isaac-perception) to learn about:
- NVIDIA Isaac Sim for perception training
- Synthetic data generation
- Domain randomization
- Reusable perception skills (Layer 3)
:::

---

## References

1. [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
2. [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
3. [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer)
4. [Unity URP Documentation](https://docs.unity3d.com/Packages/com.unity.render-pipelines.universal@latest)

---

**Chapter Status**: ✅ Complete - Unity 2022.3 LTS tested
**Last Updated**: 2025-11-28
**Layer**: 2 (AI-Assisted) - 3 "Ask AI" prompts included
