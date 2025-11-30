# Chapter 9: Capstone Project

## Metadata
- **ID**: `chapter-9-capstone`
- **Sidebar Position**: 9
- **Layer**: 4 (Spec-Driven)
- **Module**: Capstone
- **Estimated Duration**: 12 hours
- **Prerequisites**: Chapter 8
- **Tags**: ["layer4", "capstone", "integration", "end-to-end", "spec-driven"]
- **Keywords**: ["capstone project", "integration", "autonomous humanoid", "end-to-end system", "specification-driven"]

## Learning Objectives
1. Design and specify an end-to-end autonomous humanoid system
2. Integrate ≥3 skills from Layers 3-4 (VSLAM, navigation, VLA)
3. Validate system performance in Isaac Sim or Gazebo
4. Demonstrate voice-controlled multi-step tasks
5. Achieve ≥70% task completion success rate

## Scope
**In Scope:**
- Complete system integration (perception → planning → control)
- Multi-modal task execution (voice command → robot action)
- Performance benchmarking and debugging
- Demo creation for portfolio

**Out of Scope**:
- Hardware deployment → Chapter 10
- Custom research contributions → PhD territory
- Production-grade system hardening → Industry setting

## Key Concepts
- **End-to-End System**: All components integrated from sensing to actuation
- **Task Decomposition**: Breaking complex commands into skill sequences
- **Robustness Testing**: Edge case handling and failure recovery

## Capstone Project Examples
1. **Voice-Controlled Object Retrieval**:
   - Command: "Find the red cube and bring it here"
   - Skills: Object detection → Path planning → Navigation → Manipulation

2. **Autonomous Room Navigation**:
   - Command: "Go to the kitchen and tell me what you see"
   - Skills: VSLAM → Navigation → Object detection → Voice synthesis

## Exercises Overview
1. **Create Capstone Specification** (Hard) - **spec.md required**
2. **Implement End-to-End System** (Hard)
3. **Benchmark Performance** (Hard) - **≥70% success rate**
4. **Create Demo Video** (Medium) - **For portfolio**

## Constitution Alignment
- **Layer 4**: Spec-driven, ≥3 skill integration required
- **Safety**: All velocity limits, emergency stops enforced

## Success Criteria
- ≥70% exercise completion
- ≥70% capstone task success rate
- Complete specification document
- Functional demo

---

**Status**: Skeleton - Content to be populated
**Last Updated**: 2025-11-28
