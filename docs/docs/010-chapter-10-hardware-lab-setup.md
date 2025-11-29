---
id: chapter-10-hardware-lab-setup
title: "Chapter 10: Hardware & Lab Setup"
sidebar_position: 10
description: |
  Deploy your autonomous humanoid system to real hardware. Set up Jetson Orin, cameras, LiDAR, and IMU. Bridge sim-to-real gap.
keywords:
  - Jetson Orin
  - hardware deployment
  - sensors
  - sim-to-real
  - real-world testing
tags:
  - practice
  - hardware
  - deployment
---

# Chapter 10: Hardware & Lab Setup

## Overview

**Learning Objectives:**
1. Set up NVIDIA Jetson Orin for edge computing
2. Install and configure sensors (camera, LiDAR, IMU)
3. Deploy ROS 2 packages to hardware
4. Bridge sim-to-real gap (calibration, tuning)
5. Build safe robotics lab environment

:::info Prerequisites
Complete [Chapter 9](./chapter-9-capstone) - Capstone project in simulation
:::

**Estimated Duration**: 4 hours (hardware setup + testing)

---

## Hardware Bill of Materials (BOM)

### Core Components

| Component | Model | Purpose | Cost (USD) |
|-----------|-------|---------|------------|
| **Edge Computer** | NVIDIA Jetson Orin Nano | Run ROS 2 + perception | $499 |
| **RGB-D Camera** | Intel RealSense D435i | Vision + depth | $329 |
| **LiDAR** | RPLiDAR A1M8 | 2D scanning | $99 |
| **IMU** | Adafruit BNO085 | Orientation tracking | $20 |
| **Power Supply** | 19V 65W adapter | Power Jetson | $25 |
| **Cables** | USB-C, USB 3.0 | Sensor connections | $30 |

**Total**: ~$1,000

### Optional Components

- **Wi-Fi Dongle**: TP-Link AC1300 ($30) for wireless ROS 2
- **Emergency Stop Button**: Estop switch ($15) for safety
- **Battery**: Portable power bank ($50) for untethered operation

---

## NVIDIA Jetson Orin Nano Setup

### Step 1: Flash JetPack

```bash
# Download NVIDIA SDK Manager (on host PC)
# https://developer.nvidia.com/sdk-manager

# Connect Jetson to PC via USB-C (recovery mode)
# Flash JetPack 5.1.2 (includes Ubuntu 20.04 + ROS 2 Foxy)

# After flashing, boot Jetson and update
sudo apt update && sudo apt upgrade
```

### Step 2: Install ROS 2 Humble

```bash
# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble (lighter desktop variant for Jetson)
sudo apt update
sudo apt install ros-humble-ros-base

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Verify Installation

```bash
# Test ROS 2
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener

# Expected: Listener receives messages from talker
```

---

## Sensor Installation

### Intel RealSense D435i

**Install RealSense SDK**:
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
```

**Test Camera**:
```bash
# Launch RealSense node
ros2 launch realsense2_camera rs_launch.py

# View image
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

**Expected**: Live camera feed displays.

---

### RPLiDAR A1M8

**Install Driver**:
```bash
sudo apt install ros-humble-rplidar-ros
```

**Connect LiDAR** (USB to Jetson):
```bash
# Give permissions
sudo chmod 666 /dev/ttyUSB0
```

**Launch LiDAR**:
```bash
ros2 launch rplidar_ros view_rplidar_a1_launch.py
```

**Expected**: RViz shows laser scan (360° point cloud).

---

### Adafruit BNO085 IMU

**Install I2C Tools**:
```bash
sudo apt install python3-pip
pip3 install adafruit-circuitpython-bno08x
```

**Connect IMU** (I2C to Jetson):
- SDA → Pin 3 (GPIO 2)
- SCL → Pin 5 (GPIO 3)
- VIN → Pin 1 (3.3V)
- GND → Pin 6 (Ground)

**Test IMU**:
```python title="scripts/test_imu.py"
import board
import busio
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)

while True:
    accel_x, accel_y, accel_z = bno.acceleration
    print(f"Acceleration: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} m/s²")
```

**Expected**: Prints accelerometer data.

---

## Deploying Capstone to Hardware

### Step 1: Transfer ROS 2 Workspace

```bash
# On development PC, package workspace
cd ~/ros2_ws
tar -czf humanoid_ws.tar.gz src/

# Transfer to Jetson (via SCP)
scp humanoid_ws.tar.gz jetson@192.168.1.100:~/
```

### Step 2: Build on Jetson

```bash
# On Jetson
cd ~
tar -xzf humanoid_ws.tar.gz
cd ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build (use 4 cores on Jetson Orin Nano)
colcon build --parallel-workers 4

source install/setup.bash
```

### Step 3: Run Capstone

```bash
# Launch perception + navigation
ros2 launch humanoid_navigation humanoid_real.launch.py
```

---

## Sim-to-Real Transfer

### Common Issues & Fixes

#### Issue 1: Navigation Fails on Real Robot

**Cause**: Simulated and real sensor data differ.

**Fix**:
1. Recalibrate costmap inflation radius
2. Tune controller gains
3. Collect real-world data and retrain perception model

#### Issue 2: Object Detection Accuracy Drops

**Cause**: Domain shift (sim textures ≠ real world).

**Fix**:
1. Fine-tune detection model on real images
2. Use domain randomization during training (Ch. 6)
3. Adjust confidence threshold

#### Issue 3: Motors Overshoot Goals

**Cause**: PID gains tuned for simulation.

**Fix**:
```yaml
# Reduce P gain
controller:
  gains:
    p: 0.5  # Was 1.0 in sim
```

---

## Lab Safety Setup

### Physical Safety

**Equipment**:
- Emergency stop button (estop) on robot
- Safety mat around workspace
- Fire extinguisher (for battery/electrical fires)
- First aid kit

**Procedures**:
1. **Power-on checklist**: Verify estop works, check battery level
2. **Testing protocol**: Start with slow speeds, increase gradually
3. **Emergency shutdown**: Estop → Power off → Disconnect battery

### Software Safety

**Velocity Limits**:
```yaml
# Set conservative limits for initial testing
max_vel_x: 0.2  # Start slow (was 0.5 m/s in sim)
max_vel_theta: 0.5  # Reduce rotation speed
```

**Watchdog Timer**:
```python
class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')
        self.timer = self.create_timer(1.0, self.check_safety)

    def check_safety(self):
        # Check battery level
        if self.battery_level < 10:
            self.get_logger().error("Low battery! Stopping robot.")
            self.publish_stop_command()

        # Check IMU tilt
        if abs(self.imu_roll) > 0.5 or abs(self.imu_pitch) > 0.5:
            self.get_logger().error("Robot tilting! Emergency stop.")
            self.publish_stop_command()
```

---

## Exercises

### Exercise 1: Set Up Jetson Orin (Easy)

**Requirements**:
1. Flash JetPack 5.1.2
2. Install ROS 2 Humble
3. Verify with demo nodes

**Acceptance Criteria**:
- Jetson boots into Ubuntu
- ROS 2 demo nodes communicate
- Network connectivity confirmed

---

### Exercise 2: Integrate All Sensors (Medium)

**Requirements**:
1. Connect RealSense camera, RPLiDAR, IMU
2. Launch all sensor nodes
3. Verify data in RViz

**Acceptance Criteria**:
- Camera publishes RGB-D at 30 Hz
- LiDAR scans at 10 Hz, 360° coverage
- IMU orientation updates at 100 Hz

---

### Exercise 3: Deploy Capstone to Hardware (Hard)

**Requirements**:
1. Transfer workspace to Jetson
2. Build all packages
3. Run ≥3 benchmark tasks from Ch. 9

**Acceptance Criteria**:
- Robot navigates to real-world locations
- Detects real objects with ≥70% recall
- No hardware damage during testing

---

## Troubleshooting

<details>
<summary><strong>Error: "Cannot find /dev/ttyUSB0"</strong></summary>

**Cause**: LiDAR not recognized or permissions issue.

**Fix**:
```bash
# List USB devices
lsusb

# Give permissions
sudo chmod 666 /dev/ttyUSB0

# Make permanent
sudo usermod -aG dialout $USER
# Logout and login
```

</details>

<details>
<summary><strong>Error: RealSense "No device connected"</strong></summary>

**Cause**: USB 3.0 power issue or cable problem.

**Fix**:
1. Use USB 3.0 port (blue port)
2. Try different cable
3. Update RealSense firmware:
   ```bash
   rs-fw-update
   ```

</details>

---

## Self-Check

Before Chapter 11, verify you have:

- [ ] Set up Jetson Orin with ROS 2 Humble
- [ ] Connected all sensors (camera, LiDAR, IMU)
- [ ] Deployed capstone code to hardware
- [ ] Run ≥3 real-world tests successfully
- [ ] Implemented lab safety measures

---

## Next Steps

:::note What's Next?
Continue to [Chapter 11: Safety & Best Practices](./chapter-11-safety-best-practices) for comprehensive safety guidelines, ethics, and professional development practices.
:::

---

## References

1. [NVIDIA Jetson Orin Documentation](https://developer.nvidia.com/embedded/jetson-orin)
2. [Intel RealSense ROS 2](https://github.com/IntelRealSense/realsense-ros)
3. [RPLiDAR ROS 2](https://github.com/Slamtec/rplidar_ros)

---

**Chapter Status**: ✅ Complete
**Last Updated**: 2025-11-29
**Layer**: Practice - Real-world deployment
