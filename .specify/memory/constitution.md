<!--
═══════════════════════════════════════════════════════════════════════════════
SYNC IMPACT REPORT - Constitution Update
═══════════════════════════════════════════════════════════════════════════════

Version Change: Initial (0.0.0) → 1.0.0
Date: 2025-11-28
Change Type: MAJOR (Initial constitution creation for Physical AI & Humanoid Robotics textbook)

Modified/Added Principles:
  ✅ NEW: Vision & Purpose - Defines book goals and pedagogical objectives
  ✅ NEW: Layered Learning Framework (4 layers: Foundation → AI-Assisted → Intelligence Design → Spec-Driven Integration)
  ✅ NEW: Module Guidelines (ROS 2, Digital Twin, AI-Robot Brain, VLA)
  ✅ NEW: Reasoning & Self-Monitoring - Agent behavior and validation prompts
  ✅ NEW: Quality & Success Metrics - Content validation and learning metrics
  ✅ NEW: Governance & Amendment - Authority hierarchy and versioning policy
  ✅ NEW: References & Delegation - Document relationships
  ✅ NEW: Book Production Notes - Docusaurus integration requirements

Template Synchronization Status:
  ✅ plan-template.md - Reviewed, compatible with constitution principles
  ✅ spec-template.md - Reviewed, aligns with user scenario and requirements structure
  ✅ tasks-template.md - Reviewed, supports modular task organization
  ⚠️  PENDING: Command files review (*.md in .claude/commands/)
  ⚠️  PENDING: Create docs/chapter-index.md (delegated reference)
  ⚠️  PENDING: Review .claude/skills/ directory structure
  ⚠️  PENDING: Review papers/*.md for reasoning activation patterns

Follow-Up TODOs:
  1. Create docs/chapter-index.md with weekly module breakdown
  2. Populate .claude/skills/ with reusable intelligence components
  3. Review command files for consistency with new principles
  4. Verify Docusaurus configuration supports interactive exploration
  5. Create initial ADR for Layered Learning Framework architecture

Rationale for Version 1.0.0 (MAJOR):
  - Initial constitution creation (from template)
  - Establishes foundational governance for entire textbook project
  - Defines backward-incompatible education framework (4-layer progression)
  - Sets quality gates and success metrics that will govern all content

═══════════════════════════════════════════════════════════════════════════════
-->

# Physical AI & Humanoid Robotics Constitution

**Project Type**: Educational Textbook & Interactive Website (Docusaurus)
**Scope**: Course content, code examples, simulations, documentation, and student exercises
**Target Audience**: Students bridging digital AI knowledge to embodied robotics intelligence

---

## I. Vision & Purpose

### Book Goal
Teach **Physical AI & Humanoid Robotics** by bridging digital AI knowledge to real-world embodied intelligence. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions.

**Core Focus Areas**:
- AI systems operating in the physical world
- Humanoid robotics and embodied intelligence
- ROS 2 (Robot Operating System) for robot control
- Gazebo and Unity for digital twin simulation
- NVIDIA Isaac for AI-powered perception and navigation
- LLM integration for conversational robotics (Vision-Language-Action models)

### Primary Objective
Produce a **fully structured, reasoning-driven, interactive textbook website** that satisfies all pedagogical and technical requirements. The book MUST be:
- **Accurate**: Zero untested code, zero factual errors
- **Modular**: Each module independently completable with clear prerequisites
- **Visually Coherent**: Docusaurus-rendered with interactive exploration capabilities
- **Production-Relevant**: Code examples must be testable and realistic, not toy examples

### Success is Measured By
- **Content Quality**: All outputs strictly follow user intent and constitution principles
- **Traceability**: Prompt History Records (PHRs) created automatically for every user interaction
- **Decision Visibility**: Architectural Decision Records (ADRs) suggested for significant decisions
- **Incremental Validity**: All changes are small, testable, and reference code precisely

---

## II. Layered Learning Framework

This framework enforces **progressive skill development** from basic execution to autonomous intelligence design. Students MUST NOT skip layers.

### Layer 1: Foundation – Independent Execution
**Goal**: Students execute basic tasks and detect errors without assistance

**Requirements**:
- Students independently execute ROS 2 node creation, topic publishing/subscribing
- Students independently launch Gazebo/Isaac simulations
- Exercises include error detection and self-correction prompts
- NO step-by-step hand-holding; provide expected outputs for self-validation

**Verification**:
- Students complete tasks without AI guidance
- Students identify and fix basic errors (syntax, missing dependencies, configuration issues)
- Pass rate: ≥75% task completion independently

### Layer 2: AI-Assisted – Workflow Acceleration
**Goal**: Students use AI support to accelerate workflows and identify patterns

**Requirements**:
- Integrate GPT models for code planning, reasoning, and guided debugging
- Students learn to formulate effective prompts for AI assistance
- Identify recurring workflows (e.g., "create ROS 2 publisher node") for pattern documentation
- AI tools suggest optimizations but students understand the "why"

**Verification**:
- Patterns documented and reusable across exercises (≥5 documented patterns per module)
- Students explain AI suggestions rather than blindly copy-paste
- Reduced time-to-completion for repetitive tasks (measurable via student self-reports)

### Layer 3: Intelligence Design – Reusable Skills & Subagents
**Goal**: Build reusable intelligence components (skills, subagents) for complex workflows

**Classification Criteria**:
- **Autonomous Reasoning Subagents**: Workflows with ≥5 decision points (e.g., "Debug failing ROS 2 node" requires dependency check → log analysis → network inspection → config validation → fix suggestion)
- **Guidance Skills**: Workflows with 2–4 decision points (e.g., "Tune PID controller" requires model behavior → parameter adjustment → simulation test)
- **Reference Workflows**: Single-decision workflows documented for lookup (e.g., "Install Isaac Sim on Ubuntu 22.04")

**Requirements**:
- Skills stored in `.claude/skills/` directory with clear interfaces
- Each skill includes: purpose, inputs, outputs, decision tree, example usage
- Skills are testable and composable

**Verification**:
- ≥3 reusable intelligence components created per capstone project
- Skills successfully reused across different student projects
- Documentation quality: students can use skills without instructor intervention

### Layer 4: Spec-Driven Integration – Capstone Orchestration
**Goal**: Orchestrate autonomous humanoid projects using specification-first methodology

**Requirements**:
- MUST create specification (spec.md) before implementation
- ONLY proceed if a library of ≥3 reusable intelligence components exists from Layer 3
- Capstone project demonstrates autonomous humanoid completing multi-modal tasks:
  - Voice command reception (Whisper)
  - Natural language understanding (LLM)
  - Path planning (ROS 2 navigation stack)
  - Obstacle avoidance (Isaac Sim VSLAM)
  - Object manipulation (ROS 2 MoveIt)

**Verification**:
- Capstone spec.md approved before coding begins
- Successfully integrates ≥3 Layer 3 skills
- System functions end-to-end in simulation (Isaac Sim or Gazebo)
- Optional: Deployment to real hardware (Unitree G1, Jetson Orin Nano)

---

## III. Module Guidelines

### Module 1: The Robotic Nervous System (ROS 2)
**Focus**: Middleware for robot control and communication

**Core Topics**:
- ROS 2 Nodes, Topics, Services, Actions
- Bridging Python AI agents to ROS controllers using `rclpy`
- URDF (Unified Robot Description Format) for humanoid robots
- Launch files and parameter management

**Layer 1 Exercises**:
- Create a publisher/subscriber node pair (temperature sensor → display)
- Launch a URDF humanoid model in RViz
- Error detection: "Why does my node fail to communicate?" (namespace issues, QoS mismatch)

**Success Criteria**:
- Students independently create ROS 2 packages with ≥3 nodes
- Students explain ROS 2 graph architecture (nodes, topics, relationships)
- Students troubleshoot common errors (dependency issues, topic name typos) without assistance

### Module 2: Digital Twin (Gazebo & Unity)
**Focus**: Physics simulation and high-fidelity rendering

**Core Topics**:
- Gazebo physics engine: gravity, collisions, friction
- Sensor simulation: LIDAR, depth cameras, IMUs (Inertial Measurement Units)
- Unity for photorealistic rendering and human-robot interaction visualization
- Sim-to-real transfer principles

**Decision-Making Exercises**:
- "Should you use Gazebo Classic or Gazebo Harmonic?" (decision tree: existing URDF compatibility vs. new physics features)
- "How to tune simulation physics for realistic humanoid locomotion?" (timestep selection, solver configuration)

**Success Criteria**:
- Students launch humanoid simulations in Gazebo with realistic physics
- Students add custom sensors (camera, LIDAR) to URDF and visualize in simulation
- Students identify discrepancies between simulation and real-world behavior (friction coefficients, sensor noise)

### Module 3: AI-Robot Brain (NVIDIA Isaac)
**Focus**: AI-powered perception, navigation, and reinforcement learning

**Core Topics**:
- Isaac Sim and Isaac ROS 2 integration
- Visual SLAM (Simultaneous Localization and Mapping) for navigation
- Reinforcement learning for bipedal locomotion control
- Path planning and object manipulation

**Tasks**:
- Implement VSLAM for humanoid navigation in cluttered environment
- Train RL policy for stable walking (Isaac Gym integration)
- Object detection and grasping using Isaac Sim perception stack

**Success Criteria**:
- Students deploy VSLAM pipeline on simulated humanoid (navigation accuracy ≥90% in test environment)
- Students train RL policy achieving stable bipedal walk (≥10 steps without falling)
- Students integrate perception → planning → control pipeline for object manipulation task

### Module 4: Vision-Language-Action (VLA)
**Focus**: Convergence of LLMs and robotics for natural interaction

**Core Topics**:
- Voice-to-Action: OpenAI Whisper for voice command transcription
- Cognitive Planning: LLM translates natural language ("Clean the room") into ROS 2 action sequences
- Multi-modal assessment: speech + gesture + vision fusion

**Multi-Modal Interaction**:
- Voice: "Pick up the red cube" → Whisper transcription
- Vision: Camera identifies red cube location
- Action: LLM generates ROS 2 action sequence → MoveIt motion planning → Execution

**Success Criteria**:
- Voice command recognition accuracy ≥90% (Whisper model)
- LLM successfully plans actions for ≥80% of natural language commands
- End-to-end execution: voice command → robot action completion (≥70% success rate in capstone)

---

## IV. Reasoning & Self-Monitoring

### Agent Behavior Guidelines
When AI agents (Claude, GPT models) generate content for this textbook, they MUST:

1. **Reference Authoritative Sources**:
   - ROS 2 documentation: [https://docs.ros.org](https://docs.ros.org)
   - NVIDIA Isaac documentation: [https://docs.nvidia.com/isaac](https://docs.nvidia.com/isaac)
   - Research papers: Include DOI/arXiv links for algorithms (e.g., SLAM, RL policies)
   - NO unsupported claims or "best practices" without citation

2. **Enforce Progressive Layer Transitions**:
   - Do NOT provide Layer 2 (AI-assisted) solutions to Layer 1 (foundation) students
   - Flag when students attempt Layer 4 (capstone) without ≥3 Layer 3 skills
   - Suggest appropriate layer when students request help

3. **Validate Production Relevance**:
   - Code examples MUST be tested in actual ROS 2/Isaac Sim environments
   - Avoid "toy examples" that don't transfer to real projects
   - Include realistic error handling, resource management, safety constraints

4. **Prevent Convergence on Repetitive Teaching**:
   - If explaining the same concept ≥3 times, create a reusable skill (Layer 3) instead
   - Suggest documentation improvements when patterns emerge
   - Identify gaps in Layer 1 material if students repeatedly fail foundational tasks

### Self-Monitoring Prompts for Agents
Before delivering content, agents MUST answer these questions:

1. **Are code examples production-level and tested?**
   - ❌ Bad: Pseudo-code with `# TODO: implement this`
   - ✅ Good: Tested ROS 2 node with expected output logs

2. **Are students actively reasoning, not passively reading?**
   - ❌ Bad: "Here's how to launch Gazebo: [step 1] [step 2]..."
   - ✅ Good: "Before launching, consider: Do you need real-time physics? This affects the `--real-time-factor` argument."

3. **Are specifications clear, generalizable, and reusable?**
   - ❌ Bad: "Make the robot move forward"
   - ✅ Good: "Implement velocity publisher: linear.x = 0.3 m/s, limited to 0.5 m/s per safety protocol (Constitution §3.1)"

4. **Is Layer progression correct (no skipping foundational steps)?**
   - Check student context: Have they completed Layer 1 exercises for this module?
   - Refuse to provide Layer 3 skills if Layer 2 patterns not documented

5. **Did agents detect and correct convergence patterns?**
   - Track if students ask the same question type repeatedly → trigger documentation update
   - Identify conceptual gaps (e.g., "Students don't understand QoS policies") → flag for module revision

---

## V. Quality & Success Metrics

### Content Validation (Non-Negotiable)
- **Zero Untested Code**: Every code snippet MUST be executed in target environment (ROS 2 Humble + Ubuntu 22.04, Isaac Sim 2023.1.1, etc.)
- **Zero Factual Errors**: All technical claims verified against official documentation or peer-reviewed papers
- **100% Lessons Follow Foundation → Mastery Progression**: No "advanced tricks" in foundational modules
- **Safety Compliance**: All robot control code includes emergency stop mechanisms and velocity limits (see Module-specific safety standards)

### Learning Metrics (Measurable Outcomes)
- **Comprehension**: ≥80% of students correctly answer end-of-chapter concept checks
- **Completion Rate**: ≥75% of students complete lesson exercises
- **Skill Accumulation**: Each student builds library of ≥3 reusable intelligence components (Layer 3) by capstone
- **Layer Transition Success**: ≥70% of students successfully transition from one layer to next without regression

### Acceptance Criteria (Per Module)
Each module MUST include:
- **Learning Objectives**: 3-5 specific, measurable outcomes (e.g., "Student can create ROS 2 service server with error handling")
- **Prerequisites**: Explicit references to prior modules (e.g., "Requires Module 1: ROS 2 Basics")
- **Expected Outputs**: Screenshots, terminal logs, simulation videos for validation
- **Troubleshooting Section**: Top 5 common errors with solutions

### Module-Specific Safety Standards
- **ROS 2 Motor Control**:
  - Velocity limits: Linear ≤ 0.5 m/s, Angular ≤ 0.3 rad/s (educational demos)
  - Emergency stop topic: `/emergency_stop` (std_msgs/Bool)
  - Timeout: All velocity commands timeout after 1 second (require continuous publishing)

- **Isaac Sim/Gazebo Simulation**:
  - Physics timestep: ≥1ms (prevents instabilities)
  - Collision detection: Enabled by default
  - GPU memory limit warnings: Alert when VRAM usage >90%

---

## VI. Governance & Amendment

### Authority Hierarchy
This constitution is the **supreme governing document** for all book content. In case of conflicts, precedence order is:

1. **Constitution** (this file) - Overrides all other documents
2. **Domain Knowledge** (`docs/chapter-index.md`) - Module-specific technical requirements
3. **Official Documentation** (ROS 2 docs, NVIDIA Isaac docs, research papers)
4. **Agent Specifications** (`.claude/skills/`, agent-specific guidance files)

### Amendment Process

#### Minor Amendments (Increment PATCH: 1.0.0 → 1.0.1)
**Criteria**: Clarifications, wording improvements, typo fixes, non-semantic refinements

**Process**:
1. Edit `.specify/memory/constitution.md` directly
2. Update `Last Amended` date
3. Commit message: `docs: clarify constitution §X (wording improvement)`

**Examples**:
- Fixing typo: "pubsliher" → "publisher"
- Clarifying ambiguous phrasing: "reasonable velocity" → "≤0.5 m/s"
- Adding citation for existing principle

#### Major Amendments (Increment MINOR: 1.0.0 → 1.1.0 or MAJOR: 1.0.0 → 2.0.0)
**Criteria**:
- **MINOR**: New principle/section added, material expansion of guidance
- **MAJOR**: Backward-incompatible governance changes, principle removal/redefinition

**Process**:
1. Create Architecture Decision Record (ADR): `history/adr/###-decision-title.md`
2. Document in ADR:
   - **Context**: Why is this change needed?
   - **Decision**: What exactly is being changed?
   - **Consequences**: Impact on existing modules, templates, student work
   - **Migration Plan**: How to update dependent artifacts
3. Update constitution with new version number
4. Run `/sp.constitution` to propagate changes to templates
5. Update affected modules/specs/tasks
6. Commit message: `feat: add constitution principle for X (ADR-###)` or `BREAKING: redefine principle Y (ADR-###)`

**Examples**:
- **MINOR**: Adding new principle "Accessibility: All diagrams must include alt-text"
- **MAJOR**: Redefining Layer 3 skill criteria from "≥3 decision points" to "≥5 decision points" (invalidates existing skills)

### Compliance Review Expectations
- **Pre-Commit**: Author self-checks content against constitution principles
- **Peer Review**: Reviewer verifies compliance, references specific sections (e.g., "Violates §IV.3 - production relevance")
- **Quarterly Audit**: Review all modules for constitution alignment, identify drift

---

## VII. References & Delegation

### What This Constitution Contains
- **What**: Core principles, learning framework, governance rules
- **Why**: Rationale for educational approach (4-layer progression, reasoning-first)
- **When**: Layer transition criteria, amendment triggers

### What This Constitution Delegates

#### To `docs/chapter-index.md`
- **Weekly module breakdown** (Weeks 1-13)
- **Topic sequencing** within each module
- **Prerequisite chains** (e.g., "Week 5 requires Week 3 completion")
- **Recommended timeline** (lecture hours + lab hours)

#### To `.claude/skills/`
- **Reusable intelligence components** (Layer 3 skills)
- **Skill interfaces**: Inputs, outputs, decision logic
- **Example usage** for each skill

#### To `.claude/output-styles/`
- **Formatting conventions** for code examples (syntax highlighting, commenting style)
- **Diagram standards** (Mermaid.js for ROS 2 graphs, PlantUML for architecture)
- **Interactive element templates** (Docusaurus admonitions, code tabs)

#### To `papers/*.md`
- **Reasoning activation patterns** for AI agents
- **Course research notes** (literature review on Physical AI pedagogy)
- **Algorithm references** (SLAM algorithms, RL policies, VLA architectures)

---

## VIII. Notes for Book Production

### Docusaurus Integration Requirements
- **Interactive Exploration**: Website MUST support:
  - Searchable content (Algolia DocSearch or built-in search)
  - Expandable code blocks with copy buttons
  - Tabbed content for multi-language examples (Python, C++ when applicable)
  - Mermaid.js diagrams for ROS 2 node graphs
  - Video embeds for simulation demonstrations

- **Module Navigation**:
  - Sidebar auto-generated from `docs/` directory structure
  - Breadcrumbs showing current location in course
  - "Prerequisites" section linking to prior modules
  - "Next Steps" section linking to follow-up modules

### Content Production Workflow
1. **Spec Creation** (`/sp.specify`): Define module learning objectives, user scenarios
2. **Planning** (`/sp.plan`): Architecture decisions, technical stack, constitution check
3. **Task Generation** (`/sp.tasks`): Granular implementation tasks
4. **Content Writing**: Adhering to constitution principles
5. **Capstone Demonstration**: Record simulation videos, test on real hardware (if available)
6. **Assessment Creation**: Concept checks, coding exercises, capstone project rubric

### Agents Must Follow Reasoning-First Approach
- **Before Providing Solution**: Ask "What layer is this student at?"
- **Before Writing Code**: Ask "Is this production-level or a toy example?"
- **Before Explaining Concept**: Ask "Am I creating passive reading or active reasoning?"
- **After Repeated Questions**: Ask "Should this be a reusable skill?"

---

## Glossary

- **ADR**: Architecture Decision Record - Documents significant architectural decisions with context, rationale, consequences
- **PHR**: Prompt History Record - Captures user prompts and agent responses for traceability
- **ROS 2**: Robot Operating System 2 - Middleware for robot software development
- **URDF**: Unified Robot Description Format - XML format for robot models
- **VSLAM**: Visual Simultaneous Localization and Mapping - Navigation using camera input
- **VLA**: Vision-Language-Action - AI models integrating vision, language understanding, and action planning
- **Isaac Sim**: NVIDIA's robot simulation platform built on Omniverse
- **Gazebo**: Open-source robot simulator
- **Whisper**: OpenAI's speech recognition model
- **Layer 1-4**: Progressive learning stages (Foundation → AI-Assisted → Intelligence Design → Spec-Driven Integration)

---

## Document History

| Version | Date       | Changes                                              | Author  |
|---------|------------|------------------------------------------------------|---------|
| 1.0.0   | 2025-11-28 | Initial constitution created for Physical AI textbook | Claude  |

---

**Version**: 1.0.0 | **Ratified**: 2025-11-28 | **Last Amended**: 2025-11-28
