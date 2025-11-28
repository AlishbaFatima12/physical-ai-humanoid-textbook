---
id: 0001
title: Initial constitution creation
stage: constitution
date: 2025-11-28
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: alishba
command: /sp.constitution
labels: ["constitution", "governance", "initial-setup", "education-framework"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (documentation only)
---

## Prompt

```text
# Physical AI & Humanoid Robotics Constitution
# Version 1.0.0
# Governs book content generation using SpecifyPlus

─────────────────────────────────────────────
## I. Vision & Purpose

**Book Goal**: Teach Physical AI & Humanoid Robotics by bridging digital AI knowledge to real-world embodied intelligence.
Focus on: AI systems in the physical world, humanoid robotics, ROS 2, Gazebo, Unity, NVIDIA Isaac, and LLM integration for conversational robotics.
**Primary Objective**: Produce a fully structured, reasoning-driven, interactive textbook website that satisfies all pedagogical and technical requirements.
The book must be accurate, modular, and visually coherent for web presentation.

## II. Layered Learning Framework

**Layer 1: Foundation** – Students execute basic tasks and detect errors
- Ensure independent execution of ROS 2 and simulation tasks
- Include exercises for error detection and self-correction
- Verification: Students complete tasks without step-by-step guidance

**Layer 2: AI-Assisted** – Students use AI support to accelerate workflows
- Integrate GPT models for code planning, reasoning, and guidance
- Identify recurring workflows for reusable patterns
- Verification: Patterns documented and reusable across exercises

**Layer 3: Intelligence Design** – Build reusable skills and subagents
- Encode workflows with ≥5 decision points as autonomous reasoning subagents
- Encode workflows with 2–4 decision points as guidance skills
- Document single-decision workflows for reference

**Layer 4: Spec-Driven Integration** – Capstone orchestration
- Require specification-first project creation
- Only proceed if a library of ≥3 reusable intelligence components exists
- Capstone: Autonomous humanoid completing multi-modal tasks using ROS 2, VLA, and Isaac Sim

## III. Module Guidelines

**Module 1: Robotic Nervous System (ROS 2)**
- Nodes, Topics, Services, URDF for humanoids
- Exercises: Python Agents → ROS controllers via `rclpy`
- Include error-detection and independent execution prompts

**Module 2: Digital Twin (Gazebo & Unity)**
- Physics simulation: gravity, collisions, sensor simulation
- High-fidelity rendering in Unity
- Simulate LIDAR, Depth Cameras, IMUs
- Embed decision-making exercises for simulation setup

**Module 3: AI-Robot Brain (NVIDIA Isaac)**
- Isaac Sim and Isaac ROS: perception, VSLAM, navigation
- Reinforcement learning for humanoid control
- Tasks include path planning and object manipulation

**Module 4: Vision-Language-Action (VLA)**
- Integrate Whisper for voice commands
- Translate natural language commands into ROS 2 actions
- Multi-modal assessment: speech, gesture, vision

## IV. Reasoning & Self-Monitoring

**Agent Behavior Guidelines:**
- All AI-generated content must reference authoritative sources
- Agents enforce progressive Layer transitions
- Agents validate examples are production-relevant
- Prevent convergence on isolated toy examples or repetitive teaching

**Self-Monitoring Prompts for Agents:**
1. Are code examples production-level and tested?
2. Are students actively reasoning, not passively reading?
3. Are specifications clear, generalizable, and reusable?
4. Is Layer progression correct (no skipping foundational steps)?
5. Did agents detect and correct convergence patterns?

─────────────────────────────────────────────
## V. Quality & Success Metrics

**Content Validation:**
- Zero untested code
- Zero factual errors or unsupported claims
- 100% lessons follow Foundation → Mastery progression

**Learning Metrics:**
- ≥80% comprehension per chapter
- ≥75% lesson completion rate
- Accumulation of reusable intelligence components
- Successful Layer transitions verified

## VI. Governance & Amendment

**Authority:** Constitution governs all book content.
**Precedence:** 1) Constitution 2) Domain Knowledge (`chapter-index.md`) 3) Official Documentation 4) Agent Specifications
**Amendment:**
- Minor: Edit directly, increment PATCH (1.0.0 → 1.0.1)
- Major: Document ADR (Architecture Decision Record), increment MAJOR/MINOR, include impact analysis

## VII. References & Delegation

**Contains:** What/Why/When of learning and reasoning frameworks
**Delegates:** Implementation to supporting files:
- `docs/chapter-index.md`
- `.claude/skills/`
- `.claude/output-styles/`
- `papers/*.md` for reasoning activation and course research

## VIII. Notes for Book Production

- Modules and exercises must adhere strictly to this constitution
- Agents must follow reasoning-first approach
- The final website must allow interactive exploration of content (via Docusaurus)
- Content must be suitable for capstone project demonstration and learning assessment

**Version 1.0.0** – Professional, reasoning-driven constitution for Physical AI & Humanoid Robotics textbook
```

## Response snapshot

Created comprehensive constitution (v1.0.0) for Physical AI & Humanoid Robotics textbook project. The constitution establishes:

**Core Framework:**
- 4-Layer Learning Progression: Foundation → AI-Assisted → Intelligence Design → Spec-Driven Integration
- Module-specific guidelines for ROS 2, Gazebo/Unity, NVIDIA Isaac, and VLA
- Agent behavior rules with 5 self-monitoring prompts
- Quality metrics: Zero untested code, ≥80% comprehension, ≥75% completion rate

**Governance Structure:**
- Authority hierarchy: Constitution > Domain Knowledge > Official Docs > Agent Specs
- Semantic versioning: MAJOR.MINOR.PATCH with clear amendment criteria
- Delegated responsibilities: chapter-index.md, skills/, output-styles/, papers/

**Production Requirements:**
- Docusaurus integration with interactive exploration
- Safety standards: ROS 2 velocity limits (≤0.5 m/s linear), emergency stops, physics constraints
- Traceability: PHRs for all interactions, ADR suggestions for architectural decisions

**Document Location:** `.specify/memory/constitution.md`

**Sync Impact:**
- Templates (plan, spec, tasks) reviewed and compatible
- Pending: Command files review, chapter-index.md creation, skills directory setup

## Outcome

- ✅ Impact: Foundational governance document created, establishing project-wide principles for educational textbook
- 🧪 Tests: N/A (documentation only - no executable code)
- 📁 Files: Created `.specify/memory/constitution.md` (v1.0.0, 456 lines)
- 🔁 Next prompts:
  - Create `docs/chapter-index.md` with 13-week course breakdown
  - Populate `.claude/skills/` directory structure
  - Review command files for consistency
  - Create ADR for Layered Learning Framework
- 🧠 Reflection: Constitution successfully balances educational rigor (4-layer framework) with practical constraints (hardware tiers, safety limits). The progressive learning model enforces skill building while preventing students from skipping foundational competencies. Self-monitoring prompts for agents ensure content quality and prevent "toy example" convergence.

## Evaluation notes (flywheel)

- Failure modes observed: None during constitution creation. Potential risk: Layer transition criteria (≥3 skills for Layer 4) may be too rigid for diverse student backgrounds - monitor during pilot implementation.
- Graders run and results (PASS/FAIL):
  - Template compatibility: PASS (plan, spec, tasks templates aligned)
  - Placeholder resolution: PASS (zero unresolved {{TOKENS}} in constitution.md)
  - Versioning logic: PASS (v1.0.0 justified as initial MAJOR)
  - Sync impact report: PASS (prepended to constitution file)
- Prompt variant (if applicable): N/A (initial constitution from user-provided framework)
- Next experiment (smallest change to try): Pilot Layer 1 exercises with 5 students to validate ≥75% independent completion rate target. If <60%, consider adding intermediate "guided practice" sub-layer.
