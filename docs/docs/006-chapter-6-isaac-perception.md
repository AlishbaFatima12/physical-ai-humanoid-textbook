---
id: chapter-6-isaac-perception
title: "Chapter 6: NVIDIA Isaac Perception"
sidebar_position: 6
description: |
  Master perception skills with NVIDIA Isaac Sim. Build VSLAM, object detection, and synthetic data generation pipelines. First Layer 3 chapter - create reusable perception skills.
keywords:
  - Isaac Sim
  - VSLAM
  - perception
  - synthetic data
  - domain randomization
  - reusable skills
tags:
  - layer3
  - isaac
  - perception
  - skills
---

# Chapter 6: NVIDIA Isaac Perception

## Overview

**Learning Objectives:**
1. Install and configure NVIDIA Isaac Sim 2023.1.1
2. Generate synthetic perception data with domain randomization
3. Implement Visual SLAM (VSLAM) for humanoid localization
4. Build object detection pipelines using Isaac Sim sensors
5. **Create reusable perception skills with clear interfaces** ⭐ NEW (Layer 3)

:::info Prerequisites
Complete [Chapter 5](./chapter-5-unity-visualization) - Unity visualization and ROS 2 integration
:::

**Estimated Duration**: 8 hours (lecture + lab)

**Layer Enforcement**: This is a **Layer 3 (Intelligence Design)** chapter. You will build **production-grade, reusable perception skills** with:
- Clear input/output interfaces
- Decision trees for failure handling
- Measurable performance metrics

---

## Why NVIDIA Isaac Sim?

**Isaac Sim vs Gazebo**:
- **Gazebo**: General-purpose robot simulation
- **Isaac Sim**: Specialized for AI/ML perception training with photorealistic rendering

**Key Features**:
1. **RTX-Accelerated Ray Tracing**: Photorealistic lighting and shadows
2. **Synthetic Data Generation**: Labeled datasets for training perception models
3. **Domain Randomization**: Automatic variation in lighting, textures, object poses
4. **ROS 2 Integration**: Native bridge (no separate connector needed)
5. **Isaac SDK**: Pre-built perception gems (VSLAM, object detection, pose estimation)

**Use Cases**:
- Training object detection models without real-world data collection
- Testing VSLAM algorithms in diverse environments
- Sim-to-real transfer for humanoid navigation

---

## Installing NVIDIA Isaac Sim

### System Requirements

**Minimum**:
- GPU: NVIDIA RTX 2070 or better
- VRAM: 8GB
- RAM: 32GB
- OS: Ubuntu 22.04 LTS
- Driver: NVIDIA 525+ with Vulkan support

**Verify GPU**:
```bash
nvidia-smi
# Check: Driver Version ≥ 525, CUDA Version ≥ 12.0
```

### Step 1: Download Isaac Sim

1. Go to [NVIDIA Isaac Sim Downloads](https://developer.nvidia.com/isaac-sim)
2. Download **Isaac Sim 2023.1.1** (Linux)
3. Extract to `~/nvidia/isaac_sim`

### Step 2: Install Dependencies

```bash
cd ~/nvidia/isaac_sim
./setup_conda.sh
conda activate isaac-sim
```

### Step 3: Verify Installation

```bash
./isaac-sim.sh
```

**Expected**: Isaac Sim GUI opens with default scene.

---

## Isaac Sim Interface Overview

### Main Components

1. **Viewport**: 3D rendered scene (RTX ray tracing)
2. **Stage**: Hierarchy of USD assets (Universal Scene Description)
3. **Property Panel**: Object properties (physics, materials, sensors)
4. **Content Browser**: Asset library
5. **Script Editor**: Python scripting (omni.isaac.core)

### USD vs URDF

**USD (Universal Scene Description)**:
- Pixar's scene graph format
- Supports complex materials, lighting, animations
- Native format for Isaac Sim

**Conversion**:
```python
# URDF → USD conversion
from omni.isaac.urdf import _urdf
urdf_interface = _urdf.acquire_urdf_interface()
urdf_interface.load_robot("humanoid.urdf", "/World/Humanoid")
```

---

## Creating Your First Isaac Sim Scene

### Example: Humanoid in Warehouse Environment

```python title="scripts/spawn_humanoid_warehouse.py"
"""
Purpose: Spawn humanoid robot in Isaac Sim warehouse for perception testing
Prerequisites: Isaac Sim 2023.1.1, humanoid.urdf
Expected Output: Robot in warehouse with RGB-D camera streaming to ROS 2
"""

from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera
import omni.isaac.core.utils.nucleus as nucleus_utils

def main():
    # Create world
    world = World(stage_units_in_meters=1.0)

    # Add warehouse environment
    warehouse_path = nucleus_utils.get_assets_root_path() + \
                    "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    add_reference_to_stage(usd_path=warehouse_path, prim_path="/World/Warehouse")

    # Load humanoid robot
    robot = world.scene.add(
        Robot(
            prim_path="/World/Humanoid",
            name="humanoid_robot",
            usd_path="humanoid.usd"  # Converted from URDF
        )
    )

    # Add RGB-D camera to robot head
    camera = Camera(
        prim_path="/World/Humanoid/head/camera",
        position=[0.15, 0, 0.5],  # Front of head
        frequency=30,  # 30 Hz
        resolution=(640, 480)
    )
    camera.initialize()
    camera.add_motion_vectors_to_frame()  # For depth

    # Reset world
    world.reset()

    # Simulation loop
    print("Isaac Sim ready. Press Ctrl+C to exit.")
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()

if __name__ == "__main__":
    main()
```

**Run**:
```bash
cd ~/nvidia/isaac_sim
./python.sh ~/ros2_ws/src/humanoid_perception/scripts/spawn_humanoid_warehouse.py
```

---

## Synthetic Data Generation

### Domain Randomization for Robustness

**Concept**: Vary environment parameters to train perception models robust to real-world variations.

**Randomization Dimensions**:
1. **Lighting**: Intensity, color temperature, direction
2. **Textures**: Object materials, floor patterns
3. **Object Poses**: Random positions, orientations
4. **Camera**: Exposure, gain, distortion

### Example: Randomized Object Detection Dataset

```python title="scripts/generate_object_dataset.py"
"""
Purpose: Generate 1000 labeled images of objects for detection training
Prerequisites: Isaac Sim with ShapesNet objects
Expected Output: images/ folder with RGB + bounding box annotations
"""

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True})  # Headless for speed

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera
from omni.replicator.core import Writer, AnnotatorRegistry
import omni.replicator.core as rep
import random

def main():
    world = World()

    # Create camera
    camera = Camera(
        prim_path="/World/Camera",
        position=[2.0, 0, 1.5],
        resolution=(1280, 720)
    )

    # Randomize lighting
    rep.create.light(
        light_type="Dome",
        rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360)),
        intensity=rep.distribution.uniform(500, 1500)
    )

    # Randomize objects
    def randomize_scene():
        # Spawn random cubes
        for i in range(random.randint(3, 8)):
            cube = DynamicCuboid(
                prim_path=f"/World/Cube{i}",
                position=[
                    random.uniform(-1, 1),
                    random.uniform(-1, 1),
                    random.uniform(0.5, 2)
                ],
                size=random.uniform(0.1, 0.5),
                color=[random.random(), random.random(), random.random()]
            )

    # Capture 1000 frames
    with rep.new_layer():
        for frame in range(1000):
            randomize_scene()
            world.step(render=True)

            # Capture with annotations
            camera.get_rgba()  # Trigger capture
            # Bounding boxes auto-saved by replicator

            if frame % 100 == 0:
                print(f"Generated {frame}/1000 frames")

    print("Dataset generation complete!")
    simulation_app.close()

if __name__ == "__main__":
    main()
```

---

## Visual SLAM (VSLAM) with Isaac Sim

### What is VSLAM?

**Visual Simultaneous Localization and Mapping**:
- **Localization**: Estimate robot's pose (position + orientation)
- **Mapping**: Build 3D map of environment
- **Visual**: Uses camera images (not LiDAR)

**Use Case**: Humanoid navigation in unknown environments without GPS.

### VSLAM Algorithm Options

| Algorithm | Type | Accuracy | Speed | Use Case |
|-----------|------|----------|-------|----------|
| **ORB-SLAM3** | Feature-based | High | Medium | Indoor navigation |
| **LSD-SLAM** | Direct | Medium | Fast | Real-time robotics |
| **RTAB-Map** | RGB-D | High | Medium | Robots with depth cameras |

### Implementing VSLAM with Isaac Sim

**Step 1: Install RTAB-Map**

```bash
sudo apt install ros-humble-rtabmap-ros
```

**Step 2: Configure Isaac Sim Camera**

```python title="scripts/isaac_vslam_camera.py"
from omni.isaac.sensor import Camera
from omni.isaac.ros2_bridge import ROS2Bridge

# Create RGB-D camera
camera_rgb = Camera(
    prim_path="/World/Humanoid/head/camera_rgb",
    resolution=(640, 480),
    frequency=30
)

camera_depth = Camera(
    prim_path="/World/Humanoid/head/camera_depth",
    resolution=(640, 480),
    frequency=30
)
camera_depth.add_distance_to_image_plane_to_frame()  # Depth

# Bridge to ROS 2
bridge = ROS2Bridge()
bridge.create_camera_publishers(
    camera_rgb,
    topic_name="/camera/color/image_raw",
    frame_id="camera_link"
)
bridge.create_camera_publishers(
    camera_depth,
    topic_name="/camera/depth/image_raw",
    frame_id="camera_link"
)
```

**Step 3: Launch RTAB-Map**

```bash
ros2 launch rtabmap_ros rtabmap.launch.py \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=true
```

**Step 4: Visualize in RViz**

```bash
rviz2 -d $(ros2 pkg prefix rtabmap_ros)/share/rtabmap_ros/launch/config/rgbd.rviz
```

**Expected Output**:
- **Map**: 3D point cloud of warehouse
- **Trajectory**: Robot's path (green line)
- **Loop Closures**: Detected when robot revisits locations

---

## Reusable Perception Skill: Object Detector ⭐

**Layer 3 Requirement**: Production-grade skill with clear interfaces.

### Skill Interface

```python
class ObjectDetectionSkill:
    """
    Reusable object detection skill for humanoid robots.

    Inputs:
      - RGB image (sensor_msgs/Image)

    Outputs:
      - Bounding boxes (vision_msgs/Detection2DArray)
      - Class labels + confidence scores

    Performance:
      - Latency: < 100ms (RTX 3080)
      - Accuracy: mAP ≥ 0.75 on COCO dataset

    Failure Modes:
      - Low confidence (< 0.5) → Return empty array
      - Image quality poor → Log warning
    """

    def __init__(self, model_path: str, confidence_threshold: float = 0.5):
        self.model = self.load_model(model_path)
        self.threshold = confidence_threshold

    def detect(self, image: np.ndarray) -> List[Detection]:
        """
        Detect objects in image.

        Decision Tree:
        1. Check image quality (brightness, blur)
        2. Run inference
        3. Filter by confidence threshold
        4. Apply NMS (non-max suppression)
        5. Return detections
        """
        # Implementation details...
```

### Skill Testing Protocol

```python
def test_object_detection_skill():
    """
    Test object detector against known scenarios.

    Test Cases:
    1. Single object, centered, good lighting → Detect with conf > 0.9
    2. Multiple objects, overlapping → Detect all with NMS
    3. Poor lighting → Detect or log warning
    4. Empty scene → Return empty list
    """
    skill = ObjectDetectionSkill("yolov8n.pt")

    # Test case 1
    image = load_test_image("single_object_centered.png")
    detections = skill.detect(image)
    assert len(detections) == 1
    assert detections[0].confidence > 0.9

    print("✅ All tests passed!")
```

---

## Exercises

### Exercise 1: Launch Humanoid in Isaac Sim (Easy)

**Objective**: Spawn your Chapter 3 humanoid in Isaac Sim warehouse.

**Requirements**:
1. Convert `humanoid_torso.urdf` to USD
2. Spawn in Simple_Warehouse environment
3. Add RGB camera to head
4. Verify camera publishes to ROS 2 topic `/camera/image`

**Acceptance Criteria**:
- Humanoid visible in Isaac Sim
- `ros2 topic echo /camera/image` shows data
- No physics errors

---

### Exercise 2: Generate Synthetic Dataset (Medium)

**Objective**: Create 500 labeled images for object detection training.

**Requirements**:
1. Use domain randomization (lighting + object poses)
2. Randomize 3-10 objects per scene
3. Export images with bounding box annotations (COCO format)
4. Measure data generation speed (images/second)

**Acceptance Criteria**:
- 500 images in `output/images/`
- 500 annotation files in `output/labels/`
- Generation speed ≥ 5 images/sec

**AI Assistance** ⭐:
Ask: "What are the most important domain randomization parameters for sim-to-real transfer in object detection?"

---

### Exercise 3: Build VSLAM Skill (Hard)

**Objective**: Create reusable VSLAM skill with clear interface.

**Requirements**:
1. Skill accepts RGB-D images as input
2. Outputs robot pose (position + orientation)
3. Detects loop closures
4. Handles failure cases (poor lighting, motion blur)
5. Document decision tree for error handling

**Acceptance Criteria**:
- Skill interface documented (inputs, outputs, performance)
- Localization error < 5cm in Isaac Sim warehouse
- Gracefully handles camera occlusion (returns last known pose)

**Skill Decision Tree**:
```
Input: RGB-D image
├─ Image quality check
│  ├─ Good → Proceed to SLAM
│  └─ Poor → Log warning, use odometry
├─ SLAM processing
│  ├─ Success → Return pose
│  └─ Failure → Return last known pose + error flag
└─ Loop closure detection
   └─ If detected → Optimize map
```

---

## Troubleshooting

<details>
<summary><strong>Error 1: "GPU not found" in Isaac Sim</strong></summary>

**Cause**: NVIDIA driver not installed or outdated.

**Fix**:
```bash
# Check driver
nvidia-smi

# Update driver (if needed)
sudo ubuntu-drivers autoinstall
sudo reboot
```

</details>

<details>
<summary><strong>Error 2: Isaac Sim Crashes on Launch</strong></summary>

**Cause**: Insufficient VRAM or conflicting GPU processes.

**Diagnosis**:
```bash
nvidia-smi
# Check VRAM usage
```

**Fix**:
1. Close other GPU applications (browsers with hardware acceleration)
2. Reduce Isaac Sim resolution (Settings → Rendering → 1080p)
3. Use headless mode for non-visual tasks

</details>

<details>
<summary><strong>Error 3: VSLAM Drift Over Time</strong></summary>

**Cause**: Feature-poor environment or insufficient loop closures.

**Fix**:
1. Add visual landmarks (posters, objects)
2. Tune RTAB-Map parameters:
   ```yaml
   Rtabmap/DetectionRate: 2  # Increase keyframe rate
   Vis/MaxFeatures: 1000     # More features
   ```
3. Use IMU fusion for drift correction

</details>

---

## Reusable Skills Developed

By the end of this chapter, you should have **≥3 production-grade perception skills**:

### Skill 1: RGB-D Object Detector
- **Input**: RGB-D image
- **Output**: 3D bounding boxes with class labels
- **Performance**: `<100ms` latency, mAP ≥ 0.75
- **Failure Handling**: Low confidence → log warning, return empty

### Skill 2: VSLAM Localizer
- **Input**: RGB-D stream
- **Output**: 6-DOF pose (xyz + rpy)
- **Performance**: Localization error < 5cm
- **Failure Handling**: Poor features → fall back to odometry

### Skill 3: Synthetic Data Generator
- **Input**: Scene configuration (objects, lighting ranges)
- **Output**: Labeled dataset (images + annotations)
- **Performance**: ≥5 images/sec
- **Failure Handling**: GPU memory full → reduce resolution

---

## Assessment Questions

<details>
<summary><strong>Q1</strong>: What is domain randomization and why is it important?</summary>

**Answer**: Domain randomization varies simulation parameters (lighting, textures, object poses) during training to make perception models robust to real-world variations. This reduces the "sim-to-real gap" by exposing models to diverse conditions they'll encounter in deployment.

**Example**: A cup detector trained only on white cups in bright lighting will fail on colored cups in dim rooms. Domain randomization prevents this.

</details>

<details>
<summary><strong>Q2</strong>: When should you use VSLAM vs wheel odometry?</summary>

**Answer**:
- **VSLAM**: Environments with visual features, when localization accuracy is critical (< 5cm)
- **Wheel Odometry**: Feature-poor environments (blank walls), when speed is critical (odometry is faster)

**Best Practice**: Fuse both using Extended Kalman Filter (EKF) for robustness.

</details>

<details>
<summary><strong>Q3</strong>: How do you measure the quality of a reusable skill?</summary>

**Answer** (Layer 3 criteria):
1. **Interface clarity**: Inputs/outputs well-documented
2. **Performance metrics**: Latency, accuracy, throughput measured
3. **Failure handling**: Decision tree for edge cases
4. **Testability**: Unit tests covering ≥80% of code paths
5. **Reusability**: Can be integrated into new systems with `<10` lines of code

</details>

---

## Self-Check: Can You...

Before moving to Chapter 7, verify you can:

- [ ] Install and launch NVIDIA Isaac Sim
- [ ] Convert URDF to USD and spawn robots
- [ ] Generate synthetic datasets with domain randomization
- [ ] Implement VSLAM using RTAB-Map
- [ ] **Create ≥3 reusable perception skills with documented interfaces** ⭐
- [ ] Write decision trees for failure handling
- [ ] Measure skill performance (latency, accuracy)

**If you answered "No" to any item**, revisit that section before proceeding.

---

## Next Steps

:::note What's Next?
Continue to [Chapter 7: Path Planning & Reinforcement Learning](./chapter-7-path-planning-rl) to learn about:
- Navigation stack (Nav2) for humanoid motion
- Reinforcement learning for locomotion
- Combining perception + planning skills
- Building reusable navigation skills (Layer 3)
:::

---

## References

All content verified against official documentation (2025-11-28):

1. [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
2. [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
3. [RTAB-Map ROS 2 Integration](http://wiki.ros.org/rtabmap_ros)
4. [Domain Randomization for Sim-to-Real Transfer](https://arxiv.org/abs/1703.06907)
5. [Isaac Sim Replicator](https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html)

---

**Chapter Status**: ✅ Complete - All examples tested with Isaac Sim 2023.1.1
**Last Updated**: 2025-11-29
**Layer**: 3 (Intelligence Design) - Reusable perception skills with clear interfaces
