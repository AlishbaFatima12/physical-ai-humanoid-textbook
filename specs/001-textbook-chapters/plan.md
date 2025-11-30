# Implementation Plan: Physical AI & Humanoid Robotics Textbook Chapters

**Branch**: `001-textbook-chapters` | **Date**: 2025-11-28 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/001-textbook-chapters/spec.md`

## Summary

Generate 11 comprehensive textbook chapters for "Physical AI & Humanoid Robotics" course following a 4-layer learning framework (Foundation → AI-Assisted → Intelligence Design → Spec-Driven Integration). Each chapter will be deployed as a Docusaurus 3.x MDX file with interactive features, runnable ROS 2 code examples, hands-on exercises, and constitution-aligned quality standards (zero untested code, zero factual errors, safety compliance).

**Technical Approach**:
- **Context-First Methodology**: Create planning documents (`context/XX_chapter-name/readme.md`) before content generation
- **Template-Driven Generation**: Use standardized templates for chapter structure, code examples, and exercises
- **Multi-Stage Validation Pipeline**: Code testing (ROS 2 Humble), citation verification, safety compliance, Docusaurus deployment
- **Layer Progression Enforcement**: Automated checks to prevent students from skipping prerequisite layers

## Technical Context

**Language/Version**:
- Python 3.10+ (ROS 2 rclpy examples)
- C++ (optional advanced examples)
- Markdown/MDX (Docusaurus 3.x compatible)

**Primary Dependencies**:
- **ROS 2**: Humble Hawksbill (LTS until 2027)
- **Simulation**: Gazebo Harmonic, NVIDIA Isaac Sim 2023.1.1, Unity 2022.3 LTS
- **Docusaurus**: 3.x with Mermaid.js plugin, code highlighting, search
- **Python Packages**: rclpy>=3.3.0, numpy>=1.24.0, opencv-python>=4.8.0

**Storage**:
- **Context Planning**: `context/XX_chapter-name/` (planning metadata, outlines, references)
- **Content Delivery**: `docs/docs/` (final Docusaurus MDX files)
- **Assets**: `docs/static/img/chapter-X/` (images, diagrams, screenshots)

**Testing**:
- **Code Validation**: All Python/C++ examples executed in ROS 2 Humble + Ubuntu 22.04
- **Simulation Testing**: Gazebo Harmonic and Isaac Sim 2023.1.1 environments
- **Build Testing**: Docusaurus `npm run build` with zero errors
- **Link Validation**: Automated cross-reference checking

**Target Platform**:
- **Development Environment**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Deployment Platform**: Static Docusaurus website (GitHub Pages, Netlify, or Vercel)
- **Student Environment**: Ubuntu 22.04 + ROS 2 Humble (virtual or physical hardware)

**Project Type**: Educational content generation (Docusaurus-based static site)

**Performance Goals**:
- **Build Time**: Docusaurus build completes in <2 minutes for all 11 chapters
- **Page Load**: Chapter pages load in <1 second on standard broadband
- **Search**: Keyword search returns results in <500ms

**Constraints**:
- **Zero Untested Code**: Every code snippet MUST execute successfully (Constitution §V.1)
- **Zero Factual Errors**: All technical claims verified against official documentation
- **Safety Compliance**: All robot control code includes emergency stops, velocity limits, timeouts (Constitution §V.4)
- **Accessibility**: All images include alt-text (WCAG AA minimum)

**Scale/Scope**:
- **11 Chapters**: 001-011 covering Foundation (1-3), AI-Assisted (4-5), Intelligence Design (6-7), Spec-Driven (8-9), Practice (10-11)
- **33-55 Exercises**: 3-5 exercises per chapter, progressively difficult
- **110-220 Code Examples**: 10-20 examples per chapter (Python/C++/ROS 2)
- **Estimated Content**: 50,000-70,000 words total across all chapters

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Quality Standards (Constitution §V)

✅ **Zero Untested Code** (§V.1)
- **Requirement**: Every code snippet executed in target environment
- **Enforcement**: Validation pipeline (Stage 2) runs all examples in ROS 2 Humble + Ubuntu 22.04
- **Pass Criteria**: 100% code execution success with outputs matching documentation

✅ **Zero Factual Errors** (§V.1)
- **Requirement**: All technical claims verified against official documentation
- **Enforcement**: Citation validation (Stage 2) checks all references link to ROS docs, NVIDIA docs, or peer-reviewed papers
- **Pass Criteria**: 100% citations valid (no broken links, no unsupported claims)

✅ **Layer Progression Enforcement** (§II)
- **Requirement**: Students cannot skip layers; prerequisites enforced
- **Enforcement**:
  - Layer 1 (Ch 1-3): NO AI assistance prompts
  - Layer 2 (Ch 4-5): "Ask AI" prompts for pattern recognition
  - Layer 3 (Ch 6-7): Reference `.claude/skills/` with decision trees
  - Layer 4 (Ch 8-9): Hard prerequisite check (requires ≥3 Layer 3 skills)
- **Pass Criteria**: Each chapter includes appropriate layer-specific content and prerequisites

✅ **Safety Compliance** (§V.4)
- **Requirement**: Robot control code includes safety mechanisms
- **Enforcement**: Validation pipeline checks for:
  - Velocity limits (linear ≤0.5 m/s, angular ≤0.3 rad/s)
  - Emergency stop topic (`/emergency_stop`)
  - Timeout requirements (1 second for velocity commands)
- **Pass Criteria**: 100% robot control code includes safety constraints

✅ **Module Alignment** (§III)
- **Requirement**: Chapters align with module guidelines
- **Modules**:
  - Module 1: The Robotic Nervous System (ROS 2) - Chapters 2-3
  - Module 2: Digital Twin (Gazebo & Unity) - Chapters 4-5
  - Module 3: AI-Robot Brain (NVIDIA Isaac) - Chapters 6-7
  - Module 4: Vision-Language-Action (VLA) - Chapter 8
- **Pass Criteria**: Each chapter references appropriate module and follows module-specific success criteria

### Simplicity Principles

✅ **Smallest Viable Change**
- Generate chapters iteratively (start with Ch 1, validate, then proceed)
- Reuse templates across chapters (don't reinvent structure per chapter)

✅ **No Premature Abstraction**
- Use simple Markdown templates, not complex content generation frameworks
- Manual validation initially, automate only if patterns emerge

✅ **Testable Increments**
- Each chapter is independently deployable and testable
- Can release chapters 1-3 before completing chapters 4-11

### Gates Status

| Gate | Status | Notes |
|------|--------|-------|
| Zero untested code | ✅ PASS | Validation pipeline designed (Stage 2) |
| Zero factual errors | ✅ PASS | Citation verification process defined |
| Layer progression | ✅ PASS | Enforcement rules specified per layer |
| Safety compliance | ✅ PASS | Safety validation checks defined |
| Module alignment | ✅ PASS | Chapters mapped to constitution modules |
| Simplicity | ✅ PASS | Iterative approach, template reuse |

**Re-evaluation Required**: After Phase 1 design (data-model.md, templates created)

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-chapters/
├── spec.md                          # ✅ Feature specification (complete)
├── plan.md                          # ← This file (architectural plan)
├── research.md                      # Phase 0: Technical research and decisions
├── data-model.md                    # Phase 1: Chapter metadata schema
├── contracts/                       # Phase 1: Template contracts
│   ├── context-readme-template.md  # Context planning template
│   ├── outline-template.md         # Chapter outline template
│   ├── chapter-mdx-template.md     # Docusaurus MDX template
│   └── exercise-template.md        # Exercise specification template
├── quickstart.md                    # Phase 1: Quick setup guide for authors
├── checklists/
│   └── requirements.md              # ✅ Spec quality checklist (complete)
└── tasks.md                         # Phase 2: Implementation tasks (NOT created by /sp.plan)
```

### Content Organization (three-tier structure)

#### Tier 1: Context (Planning & Design)

```text
context/
├── 00_constitution/
│   └── readme.md                    # Constitution principles summary
│
├── 01_introduction-physical-ai/
│   ├── readme.md                    # Learning objectives, scope, prerequisites
│   ├── outline.md                   # Detailed section breakdown
│   └── references.md                # Authoritative sources (ROS docs, papers)
│
├── 02_ros2-fundamentals/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 03_ros2-packages-urdf/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 04_gazebo-simulation/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 05_unity-visualization/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 06_isaac-perception/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 07_path-planning-rl/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 08_vla-humanoid/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 09_capstone/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
├── 10_hardware-lab-setup/
│   ├── readme.md
│   ├── outline.md
│   └── references.md
│
└── 11_safety-best-practices/
    ├── readme.md
    ├── outline.md
    └── references.md
```

#### Tier 2: Implementation (Docusaurus Content)

```text
docs/docs/
├── 001-chapter-1-introduction-physical-ai.md
├── 002-chapter-2-ros2-fundamentals.md
├── 003-chapter-3-ros2-packages-urdf.md
├── 004-chapter-4-gazebo-simulation.md
├── 005-chapter-5-unity-visualization.md
├── 006-chapter-6-isaac-perception.md
├── 007-chapter-7-path-planning-rl.md
├── 008-chapter-8-vla-humanoid.md
├── 009-chapter-9-capstone.md
├── 010-chapter-10-hardware-lab-setup.md
└── 011-chapter-11-safety-best-practices.md
```

#### Tier 3: Assets

```text
docs/static/img/
├── chapter-1/                       # Introduction diagrams, flowcharts
├── chapter-2/                       # ROS 2 node graphs, topic visualizations
├── chapter-3/                       # URDF model screenshots, RViz captures
├── chapter-4/                       # Gazebo simulation screenshots
├── chapter-5/                       # Unity rendering examples
├── chapter-6/                       # Isaac Sim perception outputs, VSLAM maps
├── chapter-7/                       # Path planning visualizations, RL training curves
├── chapter-8/                       # VLA architecture diagrams, multimodal fusion
├── chapter-9/                       # Capstone project screenshots, demo videos
├── chapter-10/                      # Hardware setup photos, wiring diagrams
└── chapter-11/                      # Safety protocol flowcharts, risk matrices
```

**Structure Decision**:
- **Three-tier separation** provides clear boundaries between planning (`context/`), implementation (`docs/docs/`), and assets (`docs/static/img/`)
- **Sequential numbering** (001-011) enforces linear prerequisite chain and simplifies sidebar ordering
- **Context-first workflow** enables review and validation before content generation
- **Panaversity pattern alignment** follows proven educational content structure (reference: https://github.com/panaversity/ai-native-software-development/tree/main/context)

**Rationale**: ADR-002 (Context Folder Structure) documents full decision logic.

## Complexity Tracking

> **No violations detected** - Constitution Check passed all gates.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A       | N/A        | N/A                                 |

## Chapter Metadata Table

| # | Chapter Title | Layer | Module | Prerequisites | Duration | Tags |
|---|---------------|-------|--------|---------------|----------|------|
| 1 | Introduction to Physical AI | 1 (Foundation) | Introduction | None | 4h | ["layer1", "introduction", "foundation"] |
| 2 | ROS 2 Fundamentals | 1 (Foundation) | ROS 2 | Ch 1 | 6h | ["layer1", "ros2", "nodes", "topics"] |
| 3 | ROS 2 Packages & URDF | 1 (Foundation) | ROS 2 | Ch 2 | 6h | ["layer1", "ros2", "urdf", "packages"] |
| 4 | Gazebo Simulation | 2 (AI-Assisted) | Digital Twin | Ch 2-3 | 6h | ["layer2", "gazebo", "simulation"] |
| 5 | Unity Visualization | 2 (AI-Assisted) | Digital Twin | Ch 4 | 5h | ["layer2", "unity", "visualization"] |
| 6 | Isaac Perception | 3 (Intelligence Design) | AI-Robot Brain | Ch 4-5 | 8h | ["layer3", "isaac", "perception", "vslam"] |
| 7 | Path Planning & RL | 3 (Intelligence Design) | AI-Robot Brain | Ch 6 | 8h | ["layer3", "navigation", "reinforcement-learning"] |
| 8 | VLA for Humanoids | 4 (Spec-Driven) | VLA | Ch 7 | 10h | ["layer4", "vla", "multimodal", "llm"] |
| 9 | Capstone Project | 4 (Spec-Driven) | Capstone | Ch 8 | 12h | ["layer4", "capstone", "integration"] |
| 10 | Hardware & Lab Setup | Practice | Practice | Ch 9 | 4h | ["hardware", "deployment", "jetson"] |
| 11 | Safety & Best Practices | Practice | Practice | All | 4h | ["safety", "ethics", "best-practices"] |

**Total Course Duration**: 73 hours (lecture + lab)

## Content Generation Workflow

### 5-Stage Pipeline

Each chapter follows this sequential workflow:

#### Stage 1: Context Definition

**Input**: Chapter number, title, layer, module (from metadata table)

**Process**:
1. Create `context/0X_chapter-name/readme.md` using template
2. Author defines:
   - 3-5 measurable learning objectives (Bloom's taxonomy verbs)
   - Prerequisites (links to prior chapters or "None")
   - Scope (in-scope topics, out-of-scope deferrals)
   - Success criteria (student comprehension metrics)
   - Key concepts with definitions and real-world relevance
   - Exercise overview (3-5 exercises: easy → medium → hard)
   - Diagram/visualization placeholders
   - Top 5 anticipated errors for troubleshooting
   - Authoritative references (ROS docs, NVIDIA docs, research papers)
3. Author documents constitution alignment:
   - Layer progression rules
   - Safety compliance requirements (if robot control code)
   - Quality standards application

**Output**: `context/0X_chapter-name/readme.md` (complete metadata)

**Validation**:
- [ ] 3-5 measurable objectives defined
- [ ] Prerequisites explicitly stated
- [ ] Scope clearly bounded (in/out)
- [ ] Constitution alignment documented

**Responsible**: Content author or instructional designer

---

#### Stage 2: Outline Creation

**Input**: Completed `context/0X_chapter-name/readme.md`

**Process**:
1. Create `context/0X_chapter-name/outline.md` using template
2. Author breaks down chapter into sections:
   - Overview (200-300 words introduction)
   - Prerequisites Check (links to prior chapters)
   - Theory sections (H2, H3 headings with key concepts)
   - Hands-On Practice (code example placeholders)
   - Exercises (3-5 exercises with difficulty progression)
   - Troubleshooting (top 5 errors from readme.md)
   - Assessment (concept check questions)
   - Next Steps (link to subsequent chapter)
3. For each code example placeholder:
   - Define purpose (what it demonstrates)
   - Specify expected functionality
   - Note safety constraints (if robot control)
4. For each exercise placeholder:
   - State objective (skill to practice)
   - Define acceptance criteria (pass/fail)
   - Specify expected outputs (screenshots, logs)

**Output**: `context/0X_chapter-name/outline.md` (detailed section breakdown)

**Validation**:
- [ ] All sections from template present
- [ ] Code examples have purpose statements
- [ ] Exercises have clear objectives and acceptance criteria
- [ ] Heading hierarchy consistent (H1 → H2 → H3)

**Responsible**: Content author

---

#### Stage 3: Content Drafting

**Input**:
- Completed `context/0X_chapter-name/outline.md`
- Chapter MDX template (`specs/001-textbook-chapters/contracts/chapter-mdx-template.md`)

**Process**:
1. Create `docs/docs/00X-chapter-X-title.md` using MDX template
2. Generate front matter:
   ```yaml
   ---
   id: chapter-X-slug
   title: "Chapter X: Title"
   sidebar_position: X
   description: |
     1-2 sentence chapter summary
   keywords: [keyword1, keyword2, ...]
   tags: [layerX, module-name, ...]
   ---
   ```
3. Write theory sections:
   - Explain concepts with citations to official docs
   - Include equations using KaTeX syntax: `$$ equation $$`
   - Add Mermaid.js diagrams for ROS 2 graphs
4. Create runnable code examples:
   - Include header comments: Purpose, Prerequisites, Expected Output
   - Add inline comments explaining WHY (rationale), not WHAT (description)
   - Test in ROS 2 Humble + Ubuntu 22.04
   - Include safety constraints (if robot control code)
5. Design 3-5 exercises:
   - Easy: Basic application of concepts
   - Medium: Integration of multiple concepts
   - Hard: Novel problem-solving or debugging
   - Provide expected outputs for self-validation
6. Create troubleshooting section:
   - Expandable `<details>` blocks for each error
   - Include symptoms, root cause, solution, prevention
7. Add assessment questions:
   - Multiple-choice or short-answer
   - Provide answers in expandable blocks

**Output**: `docs/docs/00X-chapter-X-title.md` (complete Docusaurus MDX file)

**Validation**:
- [ ] Front matter valid (all required fields)
- [ ] Code examples include header comments
- [ ] Inline comments explain WHY
- [ ] 3-5 exercises with acceptance criteria
- [ ] Troubleshooting covers top 5 errors
- [ ] Assessment questions included

**Responsible**: Content author with technical expertise in chapter topic

---

#### Stage 4: Quality Validation

**Input**: Draft `docs/docs/00X-chapter-X-title.md`

**Process**: Run 5-stage validation checklist

**Validation Stage 1: Content Validation**
- [ ] All code examples include header comments (Purpose, Prerequisites, Expected Output)
- [ ] Inline comments explain WHY (rationale), not WHAT (description)
- [ ] 3-5 measurable learning objectives stated in overview
- [ ] Consistent heading hierarchy (H1 → H2 → H3, no skips)
- [ ] Prerequisites Check section links to prior chapters
- [ ] Next Steps section links to subsequent chapter

**Validation Stage 2: Technical Accuracy**
- [ ] All code tested in target environment (ROS 2 Humble + Ubuntu 22.04)
- [ ] Code execution outputs match documented expected results
- [ ] All technical claims cite official documentation (ROS docs, NVIDIA docs, papers with DOI/arXiv)
- [ ] Version compatibility noted (ROS 2 Humble, Isaac Sim 2023.1.1, Gazebo Harmonic)
- [ ] No unsupported claims or "best practices" without citation

**Validation Stage 3: Safety Compliance** (for robot control code)
- [ ] Velocity limits documented (linear ≤0.5 m/s, angular ≤0.3 rad/s)
- [ ] Emergency stop topic defined (`/emergency_stop`)
- [ ] Timeout requirements specified (1 second for velocity commands)
- [ ] Safety constraints included in code comments

**Validation Stage 4: Accessibility & Usability**
- [ ] All images include alt-text
- [ ] Cross-references use relative paths and resolve correctly
- [ ] Troubleshooting section covers top 5 errors
- [ ] Expected outputs provided (screenshots, terminal logs, simulation videos)
- [ ] Exercises include self-validation instructions

**Validation Stage 5: Layer Progression Enforcement**
- [ ] **Layer 1 (Ch 1-3)**: NO AI assistance prompts present
- [ ] **Layer 2 (Ch 4-5)**: "Ask AI" prompts for pattern recognition included
- [ ] **Layer 3 (Ch 6-7)**: References to `.claude/skills/` with decision trees
- [ ] **Layer 4 (Ch 8-9)**: Hard prerequisite check (requires ≥3 Layer 3 skills)

**Output**: Validation report with pass/fail status per checklist item

**Responsible**: Technical reviewer or QA engineer

---

#### Stage 5: Docusaurus Integration & Deployment

**Input**: Validated `docs/docs/00X-chapter-X-title.md`

**Process**:
1. **Build Test**:
   ```bash
   cd docs/
   npm run build
   ```
   - Must complete with zero errors
   - Check build output for warnings

2. **Link Validation**:
   - Verify all internal cross-references resolve
   - Check external links (ROS docs, NVIDIA docs, papers)
   - Confirm relative paths correct (e.g., `../002-chapter-2-ros2-fundamentals.md`)

3. **Interactive Features Test**:
   - Code copy buttons functional
   - Mermaid.js diagrams render correctly
   - Expandable `<details>` blocks work
   - Tabbed content switches properly (if multi-language examples)

4. **Search Indexing**:
   - Run local Docusaurus dev server: `npm run start`
   - Test keyword search (e.g., "ROS 2 nodes", "VSLAM", "velocity limits")
   - Verify search returns relevant chapter sections

5. **Visual QA**:
   - Sidebar shows correct sequence (1-11)
   - Breadcrumbs display current location
   - Images load and display correctly
   - Mobile responsiveness check

**Output**: Chapter ready for deployment

**Validation**:
- [ ] Build succeeds with zero errors
- [ ] All cross-references resolve
- [ ] Interactive features functional
- [ ] Search returns relevant results
- [ ] Visual rendering correct

**Responsible**: DevOps or deployment engineer

---

### Workflow Coordination: Context → Content

```
┌─────────────────────────────────────────────────────────────┐
│ Stage 1: Context Definition                                 │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Author creates context/0X_chapter-name/readme.md        │ │
│ │ (Learning objectives, scope, prerequisites, references) │ │
│ └─────────────────────────────────────────────────────────┘ │
│                          ↓                                   │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Review: Objectives align with constitution?             │ │
│ │ Pass → Continue | Fail → Revise readme.md               │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 2: Outline Creation                                   │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Author creates context/0X_chapter-name/outline.md       │ │
│ │ (Section breakdown, code placeholders, exercises)       │ │
│ └─────────────────────────────────────────────────────────┘ │
│                          ↓                                   │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Review: Section flow logical? Exercise progression OK?  │ │
│ │ Pass → Continue | Fail → Revise outline.md              │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 3: Content Drafting                                   │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Author generates docs/docs/00X-chapter-X-title.md       │ │
│ │ (Full MDX with theory, code, exercises, troubleshooting)│ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 4: Quality Validation (5-stage checklist)             │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ 1. Content Validation    2. Technical Accuracy          │ │
│ │ 3. Safety Compliance     4. Accessibility               │ │
│ │ 5. Layer Progression                                    │ │
│ └─────────────────────────────────────────────────────────┘ │
│                          ↓                                   │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ All checks pass? Yes → Continue | No → Revise content   │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│ Stage 5: Docusaurus Integration                             │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Build test → Link validation → Interactive features     │ │
│ │ Search indexing → Visual QA                             │ │
│ └─────────────────────────────────────────────────────────┘ │
│                          ↓                                   │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Deployment ready? Yes → Deploy | No → Fix issues        │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                           ↓
                  ✅ Chapter Complete
```

**Multi-Author Collaboration Model** (optional):
- **Instructional Designer**: Creates context/readme.md (learning objectives, scope)
- **Subject Matter Expert**: Creates context/outline.md and drafts content
- **Technical Reviewer**: Runs Stage 4 validation checklist
- **DevOps Engineer**: Performs Stage 5 Docusaurus integration

**Single-Author Model** (default):
- One author progresses sequentially through all 5 stages
- Self-review at each stage before proceeding

## Layer Progression Enforcement Mechanism

### Layer 1: Foundation – Independent Execution (Chapters 1-3)

**Enforcement Rules**:
1. **NO AI Assistance Prompts**
   - Content MUST NOT include phrases like "Ask AI to..." or "Use ChatGPT for..."
   - Exercises designed for independent completion
   - Validation: Automated grep check for AI-related keywords

2. **Self-Validation Outputs**
   - Every exercise includes expected outputs (screenshots, terminal logs)
   - Students compare their results to expected outputs
   - Example: "Your output should show: `[INFO] [minimal_publisher]: Publishing: 'Hello World: 0'`"

3. **Comprehensive Troubleshooting**
   - Top 5 errors documented with solutions
   - Common pitfalls explained (e.g., "forgot to source ROS 2 environment")
   - Validation: Checklist confirms troubleshooting section present

4. **≥75% Success Rate Target**
   - Exercises designed for independent completion
   - Clear acceptance criteria for each exercise
   - Example: "[ ] Node subscribes to `/topic`"

**Validation Checklist** (Stage 4.5):
- [ ] Grep check: No AI assistance keywords found
- [ ] All exercises include expected outputs
- [ ] Troubleshooting section covers ≥5 errors
- [ ] Acceptance criteria testable without external help

---

### Layer 2: AI-Assisted – Workflow Acceleration (Chapters 4-5)

**Enforcement Rules**:
1. **Include "Ask AI" Prompts**
   - Content includes strategic AI assistance prompts
   - Example: "Ask AI: 'What are the tradeoffs between Gazebo Classic and Gazebo Harmonic for humanoid simulation?'"
   - Validation: At least 3 "Ask AI" prompts per chapter

2. **Pattern Documentation Requirement**
   - Students document ≥5 reusable workflow patterns
   - Example: "Pattern: Configure Gazebo world with custom gravity"
   - Validation: Chapter includes section "Patterns to Document" with ≥5 examples

3. **Explain WHY, Not Just Execute**
   - Exercises require students to explain AI suggestions
   - Example: "After AI suggests optimization, explain in your own words why this improves performance."
   - Validation: Exercise acceptance criteria include "Explanation provided"

4. **Workflow Optimization Focus**
   - Chapters emphasize reducing time-to-completion for repetitive tasks
   - Example: "Use AI to generate boilerplate ROS 2 launch file, then customize"
   - Validation: Chapter objectives include "Accelerate workflow using AI assistance"

**Validation Checklist** (Stage 4.5):
- [ ] ≥3 "Ask AI" prompts present
- [ ] "Patterns to Document" section with ≥5 examples
- [ ] Exercises require explanation of AI suggestions
- [ ] Objectives mention workflow acceleration

---

### Layer 3: Intelligence Design – Reusable Skills (Chapters 6-7)

**Enforcement Rules**:
1. **Reference `.claude/skills/` Directory**
   - Content references existing reusable skills
   - Example: "Use the `debug-ros2-node` skill to troubleshoot communication issues."
   - Validation: Chapter includes section "Reusable Skills" with ≥2 skill references

2. **≥5 Decision Points in Workflows**
   - Exercises involve workflows with complex decision trees
   - Example: "Debug failing ROS 2 node" requires:
     1. Check dependencies installed?
     2. Analyze log output → error type?
     3. Inspect network configuration → namespace issues?
     4. Validate QoS policies → compatibility?
     5. Test with minimal example → isolate problem?
   - Validation: Exercise describes decision tree or flowchart

3. **Create ≥3 Reusable Intelligence Components**
   - Chapter capstone: Students create skills for `.claude/skills/`
   - Each skill includes: purpose, inputs, outputs, decision tree, example usage
   - Validation: Chapter includes "Skill Creation Exercise" with template

4. **Production-Grade Skill Interfaces**
   - Skills testable and composable
   - Example:
     ```yaml
     Skill: debug-ros2-node
     Inputs: [node_name, expected_topics]
     Outputs: [diagnosis, suggested_fix]
     Decision_Tree: [dependency_check, log_analysis, network_inspection, config_validation]
     ```
   - Validation: Skill template includes all required fields

**Validation Checklist** (Stage 4.5):
- [ ] "Reusable Skills" section references ≥2 existing skills
- [ ] Exercises describe ≥5 decision point workflows
- [ ] "Skill Creation Exercise" present with template
- [ ] Skill interfaces include purpose, inputs, outputs, decision tree

---

### Layer 4: Spec-Driven Integration – Capstone (Chapters 8-9)

**Enforcement Rules**:
1. **Hard Prerequisite Check**
   - Chapter 8 opening includes warning box:
     ```markdown
     :::danger Prerequisites
     **ONLY proceed if you have created ≥3 reusable intelligence components from Chapters 6-7.**

     Required skills library:
     - [ ] Skill 1: [e.g., VSLAM navigation]
     - [ ] Skill 2: [e.g., Object detection]
     - [ ] Skill 3: [e.g., Path planning]

     If you have <3 skills, return to [Chapter 6](../006-chapter-6-isaac-perception.md).
     :::
     ```
   - Validation: Warning box present at chapter start

2. **Specification-First Methodology**
   - Exercises require students to create `spec.md` BEFORE coding
   - Example: "Create a specification for voice-controlled object pickup. Include:
     - User scenarios (voice command → action)
     - Functional requirements (Whisper transcription, LLM planning, MoveIt execution)
     - Success criteria (≥70% task completion)"
   - Validation: Exercise includes "Create spec.md" step

3. **≥3 Skill Integration**
   - Capstone project integrates multiple Layer 3 skills
   - Example: Voice command ("Pick up red cube") requires:
     - Skill 1: Whisper voice transcription
     - Skill 2: Vision (camera identifies red cube location)
     - Skill 3: LLM action planning
     - Skill 4: MoveIt motion planning
   - Validation: Chapter identifies required skill integrations

4. **≥70% Success Rate in Simulation**
   - Capstone project tested in Isaac Sim or Gazebo
   - Success metric: Voice command → robot action execution ≥70% of trials
   - Validation: Chapter defines success metrics and testing protocol

**Validation Checklist** (Stage 4.5):
- [ ] Hard prerequisite warning box present (≥3 skills required)
- [ ] Exercises require spec.md creation before coding
- [ ] Chapter identifies ≥3 skill integrations for capstone
- [ ] Success metrics defined (≥70% task completion)

---

### Automated Enforcement (Future Enhancement)

**Potential automation** (not implemented in initial version):
1. **CI/CD Pipeline**:
   - Pre-commit hook: Grep check for AI keywords in Layer 1 chapters
   - Build-time validation: Parse MDX front matter tags, verify layer matches content
   - Example: If `tags: ["layer1"]`, assert no "Ask AI" prompts in content

2. **Student Progress Tracking** (out of scope for initial release):
   - Quiz system validates Layer 1 completion before unlocking Layer 2
   - Skill library checker: Blocks Layer 4 access if <3 skills created in Layer 3

**Decision**: Start with manual validation (Stage 4 checklist), automate if patterns emerge.

## Templates & Contracts

### Template 1: Context README (`context/XX_chapter-name/readme.md`)

Location: `specs/001-textbook-chapters/contracts/context-readme-template.md`

```markdown
# Chapter X: [Title]

## Metadata
- **ID**: `chapter-X-slug`
- **Sidebar Position**: X
- **Layer**: [1 | 2 | 3 | 4]
- **Module**: [ROS 2 | Digital Twin | AI-Robot Brain | VLA | Practice]
- **Estimated Duration**: X hours (lecture + lab)
- **Prerequisites**: [Chapter Y, Z] or "None"
- **Tags**: ["layerX", "module-name", "topic1", "topic2"]
- **Keywords**: ["keyword1", "keyword2", ...] (5-10 terms for search)

## Learning Objectives
1. [Use Bloom's taxonomy verbs: analyze, design, implement, evaluate, etc.]
2. [Specific, measurable outcome: "Create a ROS 2 publisher node with error handling"]
3. [Another testable objective]
4. [Optional 4th-5th objectives]

## Scope
**In Scope:**
- [Topic A that this chapter covers]
- [Concept B explained here]
- [Skill C students will practice]

**Out of Scope** (deferred to other chapters):
- [Advanced topic X → Chapter Y]
- [Related concept Z → Chapter W]
- [Alternative approach Q → Out of course scope]

## Key Concepts
- **[Term 1]**: Definition and why it matters for Physical AI & Humanoid Robotics
  - Real-world example: [Humanoid robot scenario]
- **[Term 2]**: Definition with robotics context
  - Analogy: [Help bridge digital AI knowledge to embodied systems]

## Code Examples Overview
1. **[Example Name]**: [What it demonstrates] (Language: Python | C++)
   - Purpose: [Educational goal - why this example exists]
   - Expected Output: [What student should see in terminal/simulation]
   - Safety Constraints: [If robot control: velocity limits, emergency stops, timeouts]

2. **[Another Example]**: [Different concept demonstration]
   - Purpose: [...]
   - Expected Output: [...]

[Aim for 10-20 code examples total]

## Exercises Overview
1. **[Exercise Name]** (Difficulty: Easy)
   - Objective: [Skill to practice]
   - Acceptance Criteria: [How to verify success]
   - Expected Output: [Screenshot, terminal log, or simulation behavior]

2. **[Exercise Name]** (Difficulty: Medium)
   - Objective: [Integration of multiple concepts]
   - Acceptance Criteria: [...]
   - Expected Output: [...]

3. **[Exercise Name]** (Difficulty: Hard)
   - Objective: [Novel problem-solving or debugging challenge]
   - Acceptance Criteria: [...]
   - Expected Output: [...]

[Aim for 3-5 exercises total, progressively difficult]

## Diagrams & Visualizations
- **Diagram 1**: [ROS 2 node graph showing publisher-subscriber communication] (Format: Mermaid.js)
- **Figure 2**: [URDF humanoid model visualization] (Format: Screenshot from RViz, stored in `/static/img/chapter-X/`)
- **Figure 3**: [Architecture diagram for perception pipeline] (Format: SVG or PNG, max 1200px width)

[List all visual assets needed]

## Troubleshooting (Top 5 Anticipated Errors)
1. **Error**: [Common mistake students make]
   - Symptoms: [What they see in terminal or simulation]
   - Root Cause: [Why error occurs]
   - Solution: [Step-by-step fix]
   - Prevention: [How to avoid in future]

2. **Error**: [Second most common issue]
   - Symptoms: [...]
   - Root Cause: [...]
   - Solution: [...]
   - Prevention: [...]

[Continue for 5 errors total]

## References
- [Official ROS 2 Documentation - Specific Topic](https://docs.ros.org/en/humble/...)
- [NVIDIA Isaac Documentation - Feature X](https://docs.nvidia.com/isaac/...)
- [Research Paper Title, Authors (Year)](https://doi.org/... or https://arxiv.org/...)
- [ROS 2 Design Article](https://design.ros2.org/...)

[Cite all authoritative sources for technical claims]

## Constitution Alignment
- **Layer Progression**: This chapter enforces Layer [X] rules:
  - [Layer 1]: NO AI assistance prompts; students self-validate outputs
  - [Layer 2]: Include "Ask AI" prompts for pattern recognition; document ≥5 patterns
  - [Layer 3]: Reference `.claude/skills/` with decision trees; create ≥3 reusable components
  - [Layer 4]: Require spec.md before coding; verify ≥3 Layer 3 skills exist

- **Safety Compliance** (if robot control code):
  - All velocity commands include limits (linear ≤0.5 m/s, angular ≤0.3 rad/s)
  - Emergency stop topic (`/emergency_stop`) defined
  - Timeout requirements (1 second for velocity commands)
  - Safety constraints documented in code comments

- **Quality Standards**:
  - Zero untested code: All examples tested in ROS 2 Humble + Ubuntu 22.04
  - Zero factual errors: All claims cited with official docs or peer-reviewed papers
  - Accessibility: All images include alt-text

## Success Criteria
- Students complete exercises with ≥[X]% success rate (Layer 1: ≥75%, Layer 2-3: ≥70%, Layer 4: ≥70%)
- Students demonstrate [specific skill from objectives] in capstone integration
- Students can troubleshoot [top 3 errors] independently after completing chapter

---

**Template Version**: 1.0.0
**Last Updated**: 2025-11-28
**Next Review**: After Chapter 1 pilot test
```

**Usage**: Copy template to `context/XX_chapter-name/readme.md`, fill all placeholders, validate against checklist.

---

### Template 2: Chapter Outline (`context/XX_chapter-name/outline.md`)

Location: `specs/001-textbook-chapters/contracts/outline-template.md`

```markdown
# Chapter X: [Title] - Detailed Outline

## Front Matter (MDX Header)
```yaml
---
id: chapter-X-slug
title: "Chapter X: [Title]"
sidebar_position: X
description: |
  [1-2 sentence summary from readme.md]
keywords:
  - [keyword1 from readme.md]
  - [keyword2]
  - [keyword3]
tags:
  - layerX
  - module-name
  - topic1
---
```

## Section Breakdown

### Overview (200-300 words)

**Content**:
- Introduce chapter relevance to Physical AI & Humanoid Robotics
- Bridge from previous chapter (if applicable)
- Preview key concepts and skills students will learn
- State learning objectives (from readme.md)

**Visual Assets**: None

---

### Prerequisites Check

**Content**:
```markdown
:::info Prerequisites
This chapter assumes you have completed:
- [Chapter Y: Title](../00Y-chapter-Y-slug.md)
- [Chapter Z: Title](../00Z-chapter-Z-slug.md)

Or equivalent knowledge of:
- [Concept A]
- [Concept B]
:::
```

**Purpose**: Enforce prerequisite chain, redirect students if needed

---

### Theory Section 1: [Key Concept 1]

**Content**:
- Explain concept with citations to official docs
- Include real-world robotics example
- Use analogies to bridge digital AI knowledge

**Code Example Placeholder**:
- **Name**: [e.g., "Basic ROS 2 Publisher Node"]
- **Purpose**: Demonstrate [specific learning objective]
- **Language**: Python | C++
- **Expected Functionality**: [What the code should do]
- **Safety Constraints**: [If robot control: velocity limits, emergency stop]

**Visual Assets**:
- **Diagram 1**: [ROS 2 node graph] (Mermaid.js)
  ```mermaid
  graph TD
    A[Publisher Node] --> B[Topic: /example]
    B --> C[Subscriber Node]
  ```

**Equations** (if applicable):
- Use KaTeX syntax: `$$ equation $$`
- Example: `$$ v_{linear} \leq 0.5 \, \text{m/s} $$`

---

### Theory Section 2: [Key Concept 2]

[Repeat structure from Section 1]

---

### Hands-On Practice

**Code Example 1**: [Name]

**Placeholder Details**:
```python
# Purpose: [What this demonstrates]
# Prerequisites: ROS 2 Humble installed, rclpy package
# Expected Output: [What student should see]

# [Code structure outline]
# - Imports
# - Class definition
# - Main function
# - Safety constraints (if robot control)
```

**Expected Terminal Output**:
```
[INFO] [node_name]: [Expected message]
```

**Code Example 2**: [Name]

[Repeat structure]

---

### Exercises

**Exercise 1**: [Name] (Difficulty: Easy)

**Objective**: [From readme.md]

**Instructions**:
1. [Step-by-step task]
2. [Another step]
3. [Final step]

**Acceptance Criteria**:
- [ ] [Testable criterion 1]
- [ ] [Testable criterion 2]
- [ ] [Testable criterion 3]

**Self-Validation**:
- Expected terminal output: `[OUTPUT]`
- Expected RViz visualization: [Screenshot description]
- Expected simulation behavior: [Gazebo/Isaac Sim behavior]

**Exercise 2**: [Name] (Difficulty: Medium)

[Repeat structure with integration of multiple concepts]

**Exercise 3**: [Name] (Difficulty: Hard)

[Repeat structure with novel problem-solving challenge]

---

### Troubleshooting

**Error 1**: [From readme.md]

```markdown
<details>
<summary>[Error message or symptom]</summary>

**Symptoms**: [What student sees]

**Root Cause**: [Why error occurs]

**Solution**:
1. [Step 1 to fix]
2. [Step 2]
3. [Verify fix worked]

**Prevention**: [How to avoid in future]
</details>
```

[Repeat for Errors 2-5]

---

### Assessment

**Concept Check Questions**:

```markdown
:::tip Self-Check Questions
1. [Question about key concept 1]
2. [Question about application of concept 2]
3. [Question requiring synthesis of multiple concepts]

<details>
<summary>Answers</summary>

1. [Answer with brief explanation]
2. [Answer with rationale]
3. [Answer demonstrating understanding]
</details>
:::
```

---

### Next Steps

**Content**:
```markdown
:::note What's Next?
Continue to [Chapter X+1: Title](../00X-chapter-X+1-slug.md) to learn about [next topic].

**Prerequisites for Next Chapter:**
- Completion of exercises in this chapter
- Understanding of [key concepts from this chapter]
:::
```

---

## Notes for Content Author

- **Code Examples**: Will be written in full during Stage 3 (Content Drafting)
- **Diagrams**: Mermaid.js code can be refined during Stage 3; placeholders OK for now
- **Exercises**: Ensure progressive difficulty (easy → medium → hard)
- **Troubleshooting**: Based on anticipated errors from readme.md; may refine after pilot testing

---

**Outline Version**: 1.0.0
**Last Updated**: 2025-11-28
**Status**: Template - to be customized per chapter
```

**Usage**: Copy template to `context/XX_chapter-name/outline.md`, fill section placeholders, validate structure before Stage 3.

---

### Template 3: Docusaurus MDX Chapter (`docs/docs/00X-chapter-X-title.md`)

Location: `specs/001-textbook-chapters/contracts/chapter-mdx-template.md`

**(Full template provided in separate file due to length - see contracts/chapter-mdx-template.md)**

**Key Sections**:
1. Front matter (YAML)
2. Overview with learning objectives
3. Prerequisites Check (info box)
4. Theory sections with citations
5. Hands-On Practice (runnable code examples)
6. Exercises with acceptance criteria
7. Troubleshooting (expandable details blocks)
8. Assessment (self-check questions with answers)
9. Next Steps (link to subsequent chapter)

**Usage**: Copy template during Stage 3, fill with content from outline.md, add code examples and diagrams.

---

### Template 4: Exercise Specification

Location: `specs/001-textbook-chapters/contracts/exercise-template.md`

```markdown
## Exercise [N]: [Name]

**Difficulty**: [Easy | Medium | Hard]

**Objective**: [Specific skill to practice - from readme.md]

**Estimated Time**: [X] minutes

### Instructions

1. [Clear step-by-step task]
2. [Another step]
3. [Final step - may include "verify your solution" checkpoint]

### Acceptance Criteria

Test your solution against these criteria:

- [ ] [Testable criterion 1 - must be verifiable without external help]
- [ ] [Testable criterion 2]
- [ ] [Testable criterion 3]

### Expected Output

**Terminal Output**:
```
[Exact or representative output student should see]
```

**Visual Output** (if applicable):
- **RViz**: [Screenshot description or link to expected visualization]
- **Gazebo/Isaac Sim**: [Expected robot behavior]

**File Output** (if applicable):
- **Created Files**: [List files student should have created]
- **Modified Files**: [List files student should have edited]

### Self-Validation

Compare your results to the expected output above. If your output matches:
- ✅ **All criteria met**: Proceed to next exercise
- ⚠️ **Partial match**: Review steps, check for common errors in Troubleshooting section
- ❌ **No match**: Re-read instructions, verify prerequisites (ROS 2 environment sourced, dependencies installed)

### Hints (Optional - for Medium/Hard exercises)

<details>
<summary>Hint 1: [Topic hint relates to]</summary>

[Guidance without giving away solution]
</details>

<details>
<summary>Hint 2: [Another aspect]</summary>

[Another guiding question or debugging approach]
</details>

### Common Mistakes

- **Mistake**: [Common error students make]
  - **Fix**: [How to correct]

### Extension Challenge (Optional - for Advanced Students)

[Additional challenge building on exercise - no acceptance criteria provided, encourages exploration]

---

**Exercise Version**: 1.0.0
**Last Updated**: 2025-11-28
**Tested**: [Yes/No - update after pilot test]
```

**Usage**: Embed in chapter MDX files during Stage 3, customize per exercise difficulty and learning objective.

## Architectural Decision Records (ADRs)

The following ADRs will be created to document significant architectural decisions:

### ADR-001: Sequential Chapter Numbering vs Module-Based Folders

**Status**: Proposed
**Decision**: Use sequential numbering (001-011) instead of module-based folder hierarchy
**Context**: Need to organize 11 chapters for navigation while maintaining clear prerequisite chain

**Options Considered**:
1. **Sequential numbering**: `001-chapter-1-introduction-physical-ai.md`
   - **Pros**: Simple sidebar ordering, clear linear progression, enforces prerequisite chain
   - **Cons**: All chapters at same directory level (no module grouping)

2. **Module-based folders**:
   ```
   docs/docs/
   ├── 01-foundation/
   │   ├── chapter-1-introduction.md
   │   ├── chapter-2-ros2-fundamentals.md
   │   └── chapter-3-ros2-packages-urdf.md
   ├── 02-ai-assisted/
   │   ├── chapter-4-gazebo-simulation.md
   │   └── chapter-5-unity-visualization.md
   ...
   ```
   - **Pros**: Clear module grouping, logical organization
   - **Cons**: Complex sidebar configuration, ambiguous prerequisite order within modules

3. **Hybrid**: Sequential numbering with module tags in front matter
   - **Pros**: Simple sidebar + module metadata
   - **Cons**: Module grouping only visible in metadata, not file structure

**Decision**: **Option 3 (Hybrid)** - Sequential numbering (001-011) with module/layer tags in front matter

**Rationale**:
- Docusaurus sidebar auto-sorts by `sidebar_position` (sequential)
- Cross-references simple: `../002-chapter-2-ros2-fundamentals.md` (predictable pattern)
- Module grouping preserved in `tags: ["layer1", "ros2"]` for filtering/search
- Aligns with constitution's linear prerequisite chain (§II Layer progression)

**Consequences**:
- ✅ Sidebar ordering trivial (position 1-11)
- ✅ Clear prerequisite chain (Chapter 3 requires Chapter 2)
- ✅ Module metadata available via tags
- ⚠️ No visual folder-based module grouping (acceptable tradeoff)

**Alternatives Rejected**:
- **Module folders** rejected because: Complicates cross-references, requires manual sidebar configuration, obscures linear progression

---

### ADR-002: Context Folder Structure Separate from Content

**Status**: Proposed
**Decision**: Create `context/` directory for planning, separate from `docs/docs/` content
**Context**: Need planning layer before content generation, enable review process

**Options Considered**:
1. **Context-first (Panaversity pattern)**:
   ```
   context/00_constitution/readme.md
   context/01_introduction-physical-ai/readme.md
   docs/docs/001-chapter-1-introduction-physical-ai.md
   ```
   - **Pros**: Clear separation (planning vs delivery), aligns with proven pattern, enables review before writing
   - **Cons**: Dual directory structure (more files)

2. **All-in-one**:
   ```
   docs/docs/001-chapter-1-introduction-physical-ai/
   ├── index.md (chapter content)
   ├── planning.md (context)
   └── outline.md
   ```
   - **Pros**: Single location per chapter
   - **Cons**: Mixes planning artifacts with deployment files, clutters docs/ directory

3. **Inline front matter**:
   - Store all context (objectives, scope, references) in MDX front matter
   - **Pros**: Single file per chapter
   - **Cons**: Front matter becomes massive, difficult to review planning separately

**Decision**: **Option 1 (Context-first)**

**Rationale**:
- **Proven Pattern**: Panaversity AI Native Software Development uses `context/` successfully (https://github.com/panaversity/ai-native-software-development/tree/main/context)
- **Review Workflow**: Instructional designer reviews `context/readme.md` before author writes full content
- **Separation of Concerns**: Planning (`context/`) vs delivery (`docs/`) vs assets (`static/img/`)
- **Reusability**: Context files can inform multiple output formats (Docusaurus, PDF, slides)

**Consequences**:
- ✅ Clear review gates (context → outline → content)
- ✅ Planning artifacts version-controlled
- ✅ Easier to update objectives without touching content
- ⚠️ More files to maintain (acceptable - worth the clarity)

**Alternatives Rejected**:
- **All-in-one** rejected because: Clutters docs/ directory, makes planning hard to find
- **Inline front matter** rejected because: YAML front matter not designed for large metadata

---

### ADR-003: Mermaid.js for ROS 2 Diagrams vs Static Images

**Status**: Proposed
**Decision**: Use Mermaid.js for ROS 2 node graphs, PNG/SVG for screenshots
**Context**: Need diagrams that are maintainable, version-controllable, and Docusaurus-compatible

**Options Considered**:
1. **Mermaid.js (code-based diagrams)**:
   ```markdown
   ```mermaid
   graph TD
     A[Publisher Node] --> B[Topic: /example]
     B --> C[Subscriber Node]
   ```
   - **Pros**: Version-controlled (text), easy to update, renders in Docusaurus
   - **Cons**: Limited styling, not suitable for complex architectures

2. **Static images (PNG/SVG)**:
   - Create in Draw.io, export to PNG/SVG
   - **Pros**: Full design control, professional appearance
   - **Cons**: Binary files (harder to version control), requires external tool to edit

3. **PlantUML**:
   - **Pros**: Code-based like Mermaid, supports complex diagrams
   - **Cons**: Requires external renderer, less Docusaurus integration

**Decision**: **Hybrid Approach**
- **Mermaid.js**: ROS 2 node graphs, simple flowcharts, state diagrams
- **PNG/SVG**: Screenshots (RViz, Gazebo, Isaac Sim), complex architecture diagrams

**Rationale**:
- **Mermaid.js strengths**: Perfect for ROS 2 node graphs (simple topology), easy to update when code changes
- **Static image strengths**: Better for screenshots (must capture actual output), complex multi-layer architectures
- **Docusaurus support**: Mermaid plugin official, images simple to embed

**Consequences**:
- ✅ ROS 2 diagrams live in markdown (easy to maintain)
- ✅ Screenshots realistic (captured from actual simulations)
- ⚠️ Mixed formats require guidelines (documented in quickstart.md)

**Alternatives Rejected**:
- **All Mermaid** rejected because: Cannot represent complex perception pipelines, no screenshot capability
- **All static images** rejected because: Harder to maintain when ROS 2 code changes, larger repo size

---

### ADR-004: Layer Progression Enforcement (Manual vs Automated)

**Status**: Proposed
**Decision**: Manual validation (Stage 4 checklist) initially, automate if patterns emerge
**Context**: Must ensure students don't skip layers, but avoid premature tooling

**Options Considered**:
1. **Manual validation**:
   - Reviewer checks Layer 1 chapters for AI keywords (grep)
   - Checklist confirms "Ask AI" prompts in Layer 2
   - Manual verification of prerequisite checks
   - **Pros**: Simple, flexible, human judgment
   - **Cons**: Relies on reviewer diligence, potential for oversight

2. **Automated CI/CD pipeline**:
   - Pre-commit hook: Grep for AI keywords in Layer 1 chapters
   - Build-time check: Parse MDX front matter tags, assert layer rules
   - Example: If `tags: ["layer1"]`, fail build if "Ask AI" found
   - **Pros**: Consistent, catches errors early
   - **Cons**: Requires tooling setup, may be brittle (false positives)

3. **LMS integration** (student-side enforcement):
   - Quiz gate: Students must pass Layer 1 quiz before accessing Layer 2
   - Skill library checker: Layer 4 locked until ≥3 skills created
   - **Pros**: Prevents students from skipping
   - **Cons**: Out of scope (no LMS in initial release), complex to implement

**Decision**: **Option 1 (Manual validation) with Option 2 as future enhancement**

**Rationale**:
- **Iterative Approach**: Start simple (manual checklist), measure overhead
- **Constitution Principle**: "Smallest viable change" - don't automate until proven necessary
- **Reviewer Expertise**: Human judgment catches nuanced violations (e.g., implicit AI assistance)
- **Future Path**: If manual validation becomes bottleneck, implement CI/CD checks

**Consequences**:
- ✅ Simple to implement (checklist already defined)
- ✅ Flexible (reviewer can use judgment)
- ⚠️ Relies on reviewer diligence (mitigated by checklist)
- 🔄 Revisit after 3-5 chapters: If manual errors frequent, automate

**Alternatives Rejected**:
- **Full automation** rejected because: Premature - unknown if manual validation will fail
- **LMS integration** rejected because: Out of scope for content generation phase

---

### ADR-005: Chapter Authoring Workflow (Single-Author vs Collaborative)

**Status**: Proposed
**Decision**: Support both single-author and multi-author workflows
**Context**: Uncertain how chapters will be authored (one person vs team)

**Workflows**:

**Single-Author Workflow**:
1. Author creates `context/readme.md`
2. Author creates `context/outline.md`
3. Author drafts `docs/docs/00X-chapter-X-title.md`
4. Author self-validates (Stage 4 checklist)
5. Author deploys (Stage 5)

**Multi-Author Workflow**:
1. **Instructional Designer** creates `context/readme.md` (learning objectives, scope)
2. **Subject Matter Expert** creates `context/outline.md` and drafts content
3. **Technical Reviewer** runs Stage 4 validation
4. **DevOps Engineer** performs Stage 5 deployment

**Decision**: **Templates support both workflows**
- **Default**: Single-author (simpler)
- **Optional**: Multi-author (if team available)

**Rationale**:
- **Flexibility**: Don't mandate team structure
- **Templates**: Designed for either workflow (clear stage boundaries)
- **Review Gates**: Built into both workflows (self-review or peer review)

**Consequences**:
- ✅ Works for solo content creators or teams
- ✅ Clear handoff points in multi-author case
- ⚠️ Documentation must explain both workflows (covered in quickstart.md)

---

## Phase 0: Research & Technical Decisions

**Objective**: Resolve all NEEDS CLARIFICATION items from Technical Context

### Research Tasks

#### Research Task 1: Docusaurus Plugin Configuration

**Question**: Which Docusaurus plugins are required for interactive features (Mermaid diagrams, code highlighting, search)?

**Investigation**:
1. Review Docusaurus 3.x documentation: https://docusaurus.io/docs/
2. Identify required plugins:
   - **Mermaid.js**: @docusaurus/theme-mermaid
   - **Code highlighting**: Built-in (Prism.js with Python/C++ support)
   - **Search**: @docusaurus/preset-classic (includes local search)
3. Test plugin installation in minimal Docusaurus project

**Decision Criteria**:
- Plugins must be official Docusaurus plugins (not third-party)
- Must support offline functionality (no external API calls)
- Must work with static site generation

**Output**: Document in `research.md`:
```markdown
### Docusaurus Plugin Stack

**Required Plugins**:
- `@docusaurus/theme-mermaid@3.x`: Mermaid.js diagram rendering
- `@docusaurus/preset-classic@3.x`: Includes code highlighting, search, blog (optional)
- `@docusaurus/plugin-pwa@3.x`: Offline support (optional future enhancement)

**Installation**:
```bash
npm install --save @docusaurus/theme-mermaid
```

**Configuration** (docusaurus.config.js):
```javascript
module.exports = {
  themes: ['@docusaurus/theme-mermaid'],
  markdown: {
    mermaid: true,
  },
};
```

**Tested**: Yes - sample Mermaid diagram renders correctly in local dev server

**Alternatives Considered**:
- PlantUML plugin: Rejected (requires external renderer)
- Custom diagram component: Rejected (unnecessary complexity)
```

---

#### Research Task 2: ROS 2 Code Testing Environment

**Question**: How to validate all code examples execute successfully in ROS 2 Humble + Ubuntu 22.04?

**Investigation**:
1. Identify options:
   - **Local VM**: Ubuntu 22.04 VM with ROS 2 Humble installed
   - **Docker container**: Official ROS 2 Docker image
   - **GitHub Actions**: CI/CD pipeline with ROS 2 environment
2. Test code execution in each environment
3. Measure setup time, maintenance overhead

**Decision Criteria**:
- Must exactly match student environment (Ubuntu 22.04 + ROS 2 Humble)
- Must be reproducible (not "works on my machine")
- Must be practical for author workflow (not too slow)

**Output**: Document in `research.md`:
```markdown
### ROS 2 Code Testing Strategy

**Chosen Approach**: Docker container (recommended) + local VM (optional)

**Docker Setup**:
```bash
docker pull ros:humble-ros-base-jammy
docker run -it --rm ros:humble-ros-base-jammy /bin/bash
# Inside container: test code examples
```

**Advantages**:
- ✅ Exact environment match (Ubuntu 22.04 + ROS 2 Humble)
- ✅ Reproducible (Dockerfile version-controlled)
- ✅ Fast setup (<5 minutes)

**Disadvantages**:
- ⚠️ GUI applications (RViz) require X11 forwarding
- ⚠️ Gazebo/Isaac Sim require GPU passthrough (complex)

**Recommendation**:
- **Python/C++ ROS 2 code**: Test in Docker container
- **Gazebo/Isaac Sim**: Test in local VM or real hardware

**Alternative**: GitHub Actions CI/CD
- Future enhancement: Automate code testing on every commit
- Initial version: Manual Docker testing (simpler)

**Testing Checklist** (per chapter):
- [ ] All Python examples execute without errors
- [ ] Expected outputs match documentation
- [ ] No undocumented dependencies (all pip packages listed in requirements.txt)
```

---

#### Research Task 3: Citation Management Strategy

**Question**: How to ensure all technical claims cite authoritative sources?

**Investigation**:
1. Identify authoritative sources:
   - ROS 2 docs: https://docs.ros.org
   - NVIDIA Isaac docs: https://docs.nvidia.com/isaac
   - Research papers: DOI or arXiv links
2. Research citation formats for Markdown
3. Identify link validation tools (prevent broken links)

**Decision Criteria**:
- Citations must be verifiable (no "common knowledge" claims)
- Links must not break over time (prefer DOI over URLs)
- Format must be consistent across chapters

**Output**: Document in `research.md`:
```markdown
### Citation Standards

**Format**:
- **Official Documentation**: `[Topic - ROS 2 Humble Docs](https://docs.ros.org/en/humble/Tutorials/...)`
- **Research Papers**: `[Paper Title, Authors (Year)](https://doi.org/10.1234/...)` or `[arXiv:XXXX.XXXXX](https://arxiv.org/abs/...)`
- **Design Articles**: `[ROS 2 Design - Feature](https://design.ros2.org/articles/...)`

**Link Validation**:
- Tool: `markdown-link-check` (npm package)
- Run before deployment: `npx markdown-link-check docs/**/*.md`
- Catch 404 errors, redirect chains

**Authoritative Sources** (priority order):
1. Official ROS 2 Humble documentation
2. NVIDIA Isaac Sim 2023.1.1 documentation
3. Peer-reviewed papers (IEEE, arXiv)
4. ROS 2 Design documents
5. Gazebo documentation

**Forbidden Sources**:
- ❌ Blog posts (unless official ROS/NVIDIA blog)
- ❌ Stack Overflow (can reference for common errors, but not as technical authority)
- ❌ YouTube tutorials (can link for supplementary viewing, not primary source)

**Example**:
> ROS 2 uses DDS (Data Distribution Service) for inter-process communication, enabling real-time data exchange with configurable Quality of Service policies ([ROS 2 Concepts - DDS](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)).
```

---

#### Research Task 4: Exercise Difficulty Calibration

**Question**: How to ensure exercises progress from easy → medium → hard appropriately?

**Investigation**:
1. Review educational literature on exercise design (Bloom's taxonomy)
2. Analyze example exercises from existing ROS 2 tutorials
3. Define criteria for each difficulty level

**Decision Criteria**:
- **Easy**: Single concept application, clear instructions
- **Medium**: Integration of 2-3 concepts, some problem-solving
- **Hard**: Novel scenario, debugging required, multiple valid solutions

**Output**: Document in `research.md`:
```markdown
### Exercise Difficulty Guidelines

**Easy (Layer 1 target: ≥75% student success)**:
- **Scope**: Single concept from chapter
- **Instructions**: Step-by-step (5-8 steps)
- **Acceptance Criteria**: 3-4 checkboxes, all testable
- **Example**: "Create a ROS 2 publisher node that publishes 'Hello World' to `/example` topic"
- **Bloom's Taxonomy Level**: Remember, Understand, Apply

**Medium (Layer 2-3 target: ≥70% student success)**:
- **Scope**: Integration of 2-3 concepts from chapter
- **Instructions**: High-level goals (3-5 steps), student determines details
- **Acceptance Criteria**: 4-5 checkboxes, includes "explain your approach"
- **Example**: "Create publisher-subscriber pair where subscriber filters messages based on keyword, republishes to new topic"
- **Bloom's Taxonomy Level**: Apply, Analyze

**Hard (Layer 3-4 target: ≥70% student success with hints)**:
- **Scope**: Novel problem requiring synthesis of concepts
- **Instructions**: Problem statement only (1-2 paragraphs)
- **Acceptance Criteria**: Success metrics (e.g., "≥90% message delivery rate"), student designs solution
- **Example**: "Debug a failing ROS 2 multi-node system (provided broken code). Identify root cause, propose fix, verify solution."
- **Bloom's Taxonomy Level**: Analyze, Evaluate, Create
- **Hints**: 2-3 expandable hints available (but not required for completion)

**Validation**:
- Pilot test exercises with 3-5 students per chapter
- Track success rates, adjust difficulty if <70% (hard) or <75% (easy)
- Iterate after first cohort completes course
```

---

### Research Summary Document

Location: `specs/001-textbook-chapters/research.md`

**Contents**:
1. Docusaurus plugin stack (Mermaid, code highlighting, search)
2. ROS 2 code testing strategy (Docker + local VM)
3. Citation management standards (format, link validation, authoritative sources)
4. Exercise difficulty calibration (easy/medium/hard criteria)

**Status**: To be completed in Phase 0

---

## Phase 1: Design & Contracts

### Artifact 1: Chapter Metadata Schema (`data-model.md`)

**Purpose**: Define structured metadata for each chapter to enable consistent planning and validation

**Schema**:
```yaml
Chapter:
  id: string                          # Unique identifier (e.g., "chapter-1-introduction-physical-ai")
  number: integer                     # Sequential number (1-11)
  title: string                       # Full chapter title
  sidebar_position: integer           # Docusaurus sidebar ordering (1-11)
  description: string                 # 1-2 sentence summary (max 200 chars)
  keywords: array<string>             # 5-10 search terms
  tags: array<string>                 # ["layerX", "module-name", "topic1", ...]

  layer: enum                         # [1, 2, 3, 4] - Learning framework layer
  module: enum                        # ["ROS 2", "Digital Twin", "AI-Robot Brain", "VLA", "Practice"]
  prerequisites: array<string>        # ["chapter-1", "chapter-2"] or ["None"]
  estimated_duration: integer         # Hours (lecture + lab)

  learning_objectives: array<string>  # 3-5 measurable outcomes
  scope_in: array<string>             # Topics covered in this chapter
  scope_out: array<string>            # Topics deferred to other chapters

  key_concepts: array<Concept>        # Definitions and real-world examples
  code_examples: array<CodeExample>   # Code snippets with metadata
  exercises: array<Exercise>          # Hands-on practice tasks
  diagrams: array<Diagram>            # Visual assets
  troubleshooting: array<Error>       # Top 5 anticipated errors
  references: array<Reference>        # Authoritative sources

  constitution_alignment:
    layer_progression_rules: string   # How chapter enforces layer rules
    safety_compliance: string         # Safety mechanisms (if robot control)
    quality_standards: string         # Zero untested code, zero factual errors

  success_criteria: array<string>     # Student comprehension metrics

Concept:
  term: string
  definition: string
  robotics_context: string

CodeExample:
  name: string
  purpose: string
  language: enum                      # ["Python", "C++"]
  expected_output: string
  safety_constraints: string          # (optional)

Exercise:
  name: string
  difficulty: enum                    # ["Easy", "Medium", "Hard"]
  objective: string
  instructions: array<string>
  acceptance_criteria: array<string>
  expected_output: string

Diagram:
  type: enum                          # ["Mermaid", "PNG", "SVG"]
  description: string
  filepath: string                    # (if PNG/SVG)

Error:
  message: string
  symptoms: string
  root_cause: string
  solution: string
  prevention: string

Reference:
  title: string
  url: string
  source_type: enum                   # ["Official Docs", "Research Paper", "Design Article"]
```

**Location**: `specs/001-textbook-chapters/data-model.md`

**Usage**:
- Context readme templates enforce this schema
- Validation scripts check for missing required fields
- Can export to JSON for programmatic processing (future enhancement)

---

### Artifact 2: Template Contracts (`contracts/` directory)

**Created in Phase 1**:
1. `context-readme-template.md` ✅ (defined above in Templates section)
2. `outline-template.md` ✅ (defined above)
3. `chapter-mdx-template.md` (full MDX template with front matter, sections, code blocks, exercises)
4. `exercise-template.md` ✅ (defined above)

**Location**: `specs/001-textbook-chapters/contracts/`

**Purpose**: Standardize chapter structure, reduce authoring variability, enable validation

---

### Artifact 3: Quickstart Guide (`quickstart.md`)

**Purpose**: Onboard content authors to the chapter generation workflow

**Contents**:
1. **Prerequisites**:
   - Ubuntu 22.04 VM or Docker
   - ROS 2 Humble installed
   - Docusaurus 3.x project setup
   - Text editor with Markdown/MDX support

2. **Workflow Overview**:
   - Diagram showing 5-stage pipeline (Context → Outline → Draft → Validate → Deploy)

3. **Step-by-Step Instructions**:
   - **Stage 1**: Copy `context-readme-template.md`, fill metadata
   - **Stage 2**: Copy `outline-template.md`, break down sections
   - **Stage 3**: Copy `chapter-mdx-template.md`, write full content
   - **Stage 4**: Run validation checklist (5 stages)
   - **Stage 5**: Build and deploy (`npm run build`)

4. **Code Testing Guide**:
   - Docker setup commands
   - How to run Python/C++ ROS 2 examples
   - Expected output validation

5. **Citation Guidelines**:
   - How to cite ROS 2 docs, NVIDIA docs, research papers
   - Link format standards
   - Link validation with `markdown-link-check`

6. **Diagram Creation**:
   - Mermaid.js syntax examples (ROS 2 node graphs)
   - Image export from Gazebo/RViz (screenshot tools)
   - Image storage (`/static/img/chapter-X/`)

7. **Troubleshooting**:
   - Common authoring errors (broken links, malformed front matter, failed builds)
   - How to get help (review process, contact info)

**Location**: `specs/001-textbook-chapters/quickstart.md`

---

### Artifact 4: Agent Context Update

**Action**: Run `.specify/scripts/bash/update-agent-context.sh claude`

**Purpose**: Add new technologies from this plan to agent-specific context file

**Updates**:
- Add Docusaurus 3.x to known technologies
- Add ROS 2 Humble to robotics stack
- Add Mermaid.js to diagram tools
- Preserve manual additions between markers

**Location**: Agent-specific file (e.g., `.claude/context.md`)

---

## Phase 2: Task Generation (NOT in this plan)

**Next Command**: `/sp.tasks`

**Purpose**: Break down architectural plan into granular implementation tasks

**Expected Output**: `specs/001-textbook-chapters/tasks.md` with:
- **Task 1**: Set up Docusaurus project with plugins
- **Task 2**: Create `context/` directory structure
- **Task 3**: Generate Chapter 1 context (readme.md, outline.md, references.md)
- **Task 4**: Draft Chapter 1 content (001-chapter-1-introduction-physical-ai.md)
- **Task 5**: Validate Chapter 1 (Stage 4 checklist)
- **Task 6**: Deploy Chapter 1 (Docusaurus build test)
- **Task 7-66**: Repeat for Chapters 2-11
- **Task 67**: Create consolidated bibliography
- **Task 68**: Final deployment and QA

**Note**: Tasks will be created by `/sp.tasks` command, not by `/sp.plan`.

---

## Summary

**Plan Completion Status**: ✅ Ready for implementation

**Key Deliverables**:
1. ✅ Three-tier folder structure designed (context/, docs/, static/img/)
2. ✅ 5-stage content generation workflow defined
3. ✅ Chapter metadata table with all 11 chapters
4. ✅ Layer progression enforcement mechanism specified
5. ✅ Quality validation pipeline (5-stage checklist)
6. ✅ Templates created (context readme, outline, MDX chapter, exercise)
7. ✅ 5 ADRs proposed for architectural decisions
8. ✅ Research tasks identified (Docusaurus plugins, ROS 2 testing, citations, exercise calibration)

**Constitution Compliance**: ✅ All gates passed

**Next Steps**:
1. **Immediate**: Run Phase 0 research tasks, create `research.md`
2. **Phase 1**: Create contracts (templates), generate `data-model.md`, `quickstart.md`
3. **Phase 2**: Run `/sp.tasks` to generate implementation task list
4. **Implementation**: Begin with Chapter 1 (pilot test workflow)

**Risks**:
1. **Code Testing Overhead**: ROS 2 code validation may be time-consuming
   - **Mitigation**: Start with Docker setup, automate if becomes bottleneck
2. **Content Author Expertise**: Requires deep ROS 2 + Isaac Sim knowledge
   - **Mitigation**: SME review at each stage, reference official docs
3. **Layer Progression Enforcement**: Manual validation may miss violations
   - **Mitigation**: Detailed checklist, consider automation after pilot

**Success Metrics** (from spec.md):
- ✅ SC-005: All 11 chapters deploy to Docusaurus with zero build errors (validated in Stage 5)
- ✅ SC-006: 100% code execution success (validated in Stage 4.2)
- ✅ SC-007: 100% citations verified (validated in Stage 4.2)
- ✅ SC-011: All robot control code includes safety mechanisms (validated in Stage 4.3)

---

**Plan Version**: 1.0.0
**Last Updated**: 2025-11-28
**Ready for Phase 0 Research**: ✅ Yes
