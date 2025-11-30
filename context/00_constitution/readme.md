# Constitution: Physical AI & Humanoid Robotics Course

## Overview

This document summarizes the constitutional principles governing the "Physical AI & Humanoid Robotics" textbook. All chapters must align with these foundational rules.

## 4-Layer Learning Framework

### Layer 1: Foundation - Independent Execution
- **Chapters**: 1-3
- **AI Assistance**: NONE - Students execute independently
- **Success Metric**: ≥75% task completion without help
- **Focus**: Basic ROS 2, URDF, independent troubleshooting

### Layer 2: AI-Assisted - Workflow Acceleration
- **Chapters**: 4-5
- **AI Assistance**: Pattern recognition, workflow optimization
- **Success Metric**: ≥5 documented patterns per module
- **Focus**: Gazebo, Unity, efficient development practices

### Layer 3: Intelligence Design - Reusable Skills
- **Chapters**: 6-7
- **AI Assistance**: Autonomous reasoning, decision trees
- **Success Metric**: ≥3 reusable intelligence components created
- **Focus**: Isaac Sim perception, RL, skill-building

### Layer 4: Spec-Driven Integration - Capstone
- **Chapters**: 8-9
- **AI Assistance**: Orchestration of Layer 3 skills
- **Success Metric**: ≥70% capstone success rate
- **Focus**: VLA integration, specification-first methodology

## Module Guidelines

### Module 1: The Robotic Nervous System (ROS 2)
- **Chapters**: 2-3
- **Core Concepts**: Nodes, topics, services, actions, packages, URDF

### Module 2: Digital Twin (Simulation)
- **Chapters**: 4-5
- **Core Concepts**: Gazebo physics, Unity rendering, simulation-to-real transfer

### Module 3: AI-Robot Brain (NVIDIA Isaac)
- **Chapters**: 6-7
- **Core Concepts**: VSLAM, perception pipelines, RL for locomotion

### Module 4: Vision-Language-Action (VLA)
- **Chapters**: 8
- **Core Concepts**: Multimodal AI, voice commands, action planning

## Quality Standards (§V)

### Zero Untested Code (§V.1)
- **Requirement**: Every code snippet MUST execute successfully in target environment
- **Validation**: ROS 2 Humble + Ubuntu 22.04, Isaac Sim 2023.1.1, Gazebo Harmonic
- **Pass Criteria**: 100% code execution success

### Zero Factual Errors (§V.1)
- **Requirement**: All technical claims verified against official documentation
- **Sources**: docs.ros.org, docs.nvidia.com/isaac, peer-reviewed papers (DOI/arXiv)
- **Pass Criteria**: 100% citations valid, no broken links

### Safety Compliance (§V.4)
**For all robot control code**:
- Velocity limits: linear ≤0.5 m/s, angular ≤0.3 rad/s
- Emergency stop topic: `/emergency_stop`
- Timeout: 1 second for velocity commands
- Safety constraints documented in code comments

### Accessibility
- All images include alt-text (WCAG AA minimum)
- Cross-references use relative paths
- Troubleshooting sections cover top 5 common errors

## Simplicity Principles

- **Smallest Viable Change**: Generate chapters iteratively
- **No Premature Abstraction**: Use simple Markdown templates
- **Testable Increments**: Each chapter independently deployable

## Enforcement

All chapters validated against:
1. Content quality checklist
2. Technical accuracy (code testing)
3. Safety compliance (grep checks)
4. Layer progression rules
5. Docusaurus build success (zero errors)

---

**Version**: 1.0.0
**Last Updated**: 2025-11-28
**Source**: `.specify/memory/constitution.md`
