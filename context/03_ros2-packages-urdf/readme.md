# Chapter 3: ROS 2 Packages & URDF

## Metadata
- **ID**: `chapter-3-ros2-packages-urdf`
- **Sidebar Position**: 3
- **Layer**: 1 (Foundation)
- **Module**: ROS 2
- **Estimated Duration**: 6 hours
- **Prerequisites**: Chapter 2
- **Tags**: ["layer1", "ros2", "packages", "urdf", "robot-description", "rviz"]
- **Keywords**: ["URDF", "robot description", "ROS 2 packages", "colcon", "RViz", "tf2", "robot_state_publisher"]

## Learning Objectives
1. Create ROS 2 packages with proper structure and dependencies
2. Define robot models using URDF (Unified Robot Description Format)
3. Visualize robots in RViz using robot_state_publisher
4. Understand tf2 coordinate frame transformations
5. Build and install custom packages with colcon

## Scope
**In Scope:**
- ROS 2 package structure and manifest files
- URDF syntax and robot modeling
- Robot visualization in RViz
- Joint and link definitions
- TF2 transforms

**Out of Scope**:
- Xacro macros → Advanced topics
- SDF format → Gazebo-specific (Chapter 4)
- MoveIt configuration → Motion planning (Chapter 7)

## Key Concepts
- **URDF**: XML format for robot kinematics and dynamics
- **Link**: Rigid body component of robot
- **Joint**: Connection between links (fixed, revolute, prismatic)
- **TF2**: Library for coordinate frame transformations
- **robot_state_publisher**: Node that publishes robot state to TF

## Code Examples Overview
[10-15 examples on package creation, URDF models, RViz visualization]

## Exercises Overview
1. **Create a Simple ROS 2 Package** (Easy)
2. **Build a Humanoid Robot URDF** (Medium)
3. **Visualize Robot in RViz** (Medium)
4. **Debug URDF Syntax Errors** (Hard)

## Constitution Alignment
- **Layer 1**: Independent execution, no AI prompts
- **Quality**: All URDF files validated, visualized in RViz

## Success Criteria
- ≥75% exercise completion
- Students create functional robot description files

---

**Status**: Skeleton - Content to be populated
**Last Updated**: 2025-11-28
