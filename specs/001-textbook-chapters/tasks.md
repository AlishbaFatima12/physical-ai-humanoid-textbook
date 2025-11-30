# Tasks: Physical AI & Humanoid Robotics Textbook Chapters

**Input**: Design documents from `/specs/001-textbook-chapters/`
**Prerequisites**: plan.md (complete), spec.md (complete)

**Tests**: Not explicitly requested - focusing on content validation via manual testing

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create three-tier folder structure (context/, docs/docs/, docs/static/img/)
- [X] T002 [P] Create template files in specs/001-textbook-chapters/contracts/
- [ ] T003 [P] Create research.md documenting Phase 0 research findings
- [ ] T004 [P] Create data-model.md with chapter metadata schema
- [ ] T005 Create quickstart.md guide for content authors

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY chapter can be created

**⚠️ CRITICAL**: No chapter work can begin until this phase is complete

- [X] T006 Verify Docusaurus 3.x installation with Mermaid.js plugin
- [ ] T007 [P] Set up Docker environment for ROS 2 Humble + Ubuntu 22.04 code testing
- [X] T008 [P] Create constitution summary in context/00_constitution/readme.md
- [X] T009 Configure docs/docusaurus.config.js for Mermaid, search, code highlighting
- [X] T010 Create image directories docs/static/img/chapter-1/ through chapter-11/

**Checkpoint**: Foundation ready - chapter generation can now begin

---

## Phase 3: User Story 5 - Instructor Deploying Course Content (Priority: P1) 🎯 MVP

**Goal**: Ensure Docusaurus infrastructure works and can deploy chapters with zero build errors

**Independent Test**: Run `npm run build` in docs/ directory and verify zero errors, all interactive features work

### Implementation for User Story 5

- [ ] T011 [US5] Test Docusaurus build with minimal chapter (create 001-chapter-1-introduction-physical-ai.md stub)
- [ ] T012 [US5] Verify Mermaid.js diagrams render correctly
- [ ] T013 [US5] Verify code copy buttons function correctly
- [ ] T014 [US5] Test local search functionality with sample keywords
- [ ] T015 [US5] Validate sidebar shows correct sequence (position 1-11)

**Checkpoint**: Docusaurus deployment pipeline validated - ready for real content

---

## Phase 4: User Story 1 - Student Learning Foundation Concepts (Priority: P1)

**Goal**: Create Chapters 1-3 (Layer 1 - Foundation) with independent execution focus

**Independent Test**: Student completes Ch 1-3 exercises without AI assistance, achieves ≥75% task completion

### Chapter 1: Introduction to Physical AI

- [ ] T016 [P] [US1] Create context/01_introduction-physical-ai/readme.md using context-readme-template.md
- [ ] T017 [P] [US1] Create context/01_introduction-physical-ai/outline.md using outline-template.md
- [ ] T018 [P] [US1] Create context/01_introduction-physical-ai/references.md with authoritative sources
- [ ] T019 [US1] Draft docs/docs/001-chapter-1-introduction-physical-ai.md using chapter-mdx-template.md
- [ ] T020 [US1] Validate Chapter 1 content (Stage 4 checklist: content, accuracy, accessibility, layer progression)
- [ ] T021 [US1] Test Docusaurus build with Chapter 1, verify zero errors

### Chapter 2: ROS 2 Fundamentals

- [ ] T022 [P] [US1] Create context/02_ros2-fundamentals/readme.md
- [ ] T023 [P] [US1] Create context/02_ros2-fundamentals/outline.md
- [ ] T024 [P] [US1] Create context/02_ros2-fundamentals/references.md
- [ ] T025 [US1] Draft docs/docs/002-chapter-2-ros2-fundamentals.md with 10-20 code examples
- [ ] T026 [US1] Test all Chapter 2 code examples in Docker ROS 2 Humble environment
- [ ] T027 [US1] Validate Chapter 2 (Stage 4 checklist + code execution 100% success)
- [ ] T028 [US1] Test Docusaurus build with Chapters 1-2

### Chapter 3: ROS 2 Packages & URDF

- [ ] T029 [P] [US1] Create context/03_ros2-packages-urdf/readme.md
- [ ] T030 [P] [US1] Create context/03_ros2-packages-urdf/outline.md
- [ ] T031 [P] [US1] Create context/03_ros2-packages-urdf/references.md
- [ ] T032 [US1] Draft docs/docs/003-chapter-3-ros2-packages-urdf.md with URDF examples
- [ ] T033 [US1] Test all Chapter 3 code examples in Docker + validate URDF in RViz
- [ ] T034 [US1] Validate Chapter 3 (Stage 4 checklist)
- [ ] T035 [US1] Test Docusaurus build with Chapters 1-3, verify cross-references resolve

**Checkpoint**: Layer 1 (Foundation) complete - students can execute ROS 2 basics independently

---

## Phase 5: User Story 2 - Student Using AI-Assisted Workflows (Priority: P2)

**Goal**: Create Chapters 4-5 (Layer 2 - AI-Assisted) with pattern documentation

**Independent Test**: Student documents ≥5 workflow patterns per chapter using AI assistance

### Chapter 4: Gazebo Simulation

- [ ] T036 [P] [US2] Create context/04_gazebo-simulation/readme.md
- [ ] T037 [P] [US2] Create context/04_gazebo-simulation/outline.md
- [ ] T038 [P] [US2] Create context/04_gazebo-simulation/references.md
- [ ] T039 [US2] Draft docs/docs/004-chapter-4-gazebo-simulation.md with ≥3 "Ask AI" prompts
- [ ] T040 [US2] Test Gazebo simulation examples in local VM (requires GPU)
- [ ] T041 [US2] Validate Chapter 4 (Stage 4.5: verify ≥3 AI prompts, pattern documentation section)
- [ ] T042 [US2] Test Docusaurus build with Chapters 1-4

### Chapter 5: Unity Visualization

- [ ] T043 [P] [US2] Create context/05_unity-visualization/readme.md
- [ ] T044 [P] [US2] Create context/05_unity-visualization/outline.md
- [ ] T045 [P] [US2] Create context/05_unity-visualization/references.md
- [ ] T046 [US2] Draft docs/docs/005-chapter-5-unity-visualization.md with Unity-ROS 2 integration
- [ ] T047 [US2] Test Unity visualization examples (Unity 2022.3 LTS + ROS 2 bridge)
- [ ] T048 [US2] Validate Chapter 5 (Stage 4.5: ≥3 AI prompts, ≥5 patterns to document)
- [ ] T049 [US2] Test Docusaurus build with Chapters 1-5

**Checkpoint**: Layer 2 (AI-Assisted) complete - students accelerate workflows with AI

---

## Phase 6: User Story 3 - Student Building Reusable Intelligence Components (Priority: P3)

**Goal**: Create Chapters 6-7 (Layer 3 - Intelligence Design) with skill creation focus

**Independent Test**: Student creates ≥3 reusable skills with interfaces and decision trees

### Chapter 6: Isaac Perception

- [ ] T050 [P] [US3] Create context/06_isaac-perception/readme.md
- [ ] T051 [P] [US3] Create context/06_isaac-perception/outline.md
- [ ] T052 [P] [US3] Create context/06_isaac-perception/references.md
- [ ] T053 [US3] Draft docs/docs/006-chapter-6-isaac-perception.md with VSLAM workflows
- [ ] T054 [US3] Test Isaac Sim perception examples (Isaac Sim 2023.1.1 + VSLAM validation)
- [ ] T055 [US3] Validate Chapter 6 (Stage 4.5: ≥2 skill references, skill creation exercise)
- [ ] T056 [US3] Test Docusaurus build with Chapters 1-6

### Chapter 7: Path Planning & RL

- [ ] T057 [P] [US3] Create context/07_path-planning-rl/readme.md
- [ ] T058 [P] [US3] Create context/07_path-planning-rl/outline.md
- [ ] T059 [P] [US3] Create context/07_path-planning-rl/references.md
- [ ] T060 [US3] Draft docs/docs/007-chapter-7-path-planning-rl.md with RL training workflows
- [ ] T061 [US3] Test RL training examples (Isaac Sim + stable baselines3)
- [ ] T062 [US3] Validate Chapter 7 (Stage 4.5: ≥5 decision point workflows, skill creation)
- [ ] T063 [US3] Test Docusaurus build with Chapters 1-7

**Checkpoint**: Layer 3 (Intelligence Design) complete - students create production-grade skills

---

## Phase 7: User Story 4 - Student Completing Spec-Driven Capstone (Priority: P4)

**Goal**: Create Chapters 8-9 (Layer 4 - Spec-Driven) with hard prerequisite checks

**Independent Test**: Student creates spec.md before coding, integrates ≥3 skills, achieves ≥70% success rate

### Chapter 8: VLA for Humanoids

- [ ] T064 [P] [US4] Create context/08_vla-humanoid/readme.md
- [ ] T065 [P] [US4] Create context/08_vla-humanoid/outline.md
- [ ] T066 [P] [US4] Create context/08_vla-humanoid/references.md
- [ ] T067 [US4] Draft docs/docs/008-chapter-8-vla-humanoid.md with hard prerequisite warning (≥3 skills)
- [ ] T068 [US4] Test VLA integration examples (Whisper + LLM + MoveIt)
- [ ] T069 [US4] Validate Chapter 8 (Stage 4.5: prerequisite warning box, spec.md requirement)
- [ ] T070 [US4] Test Docusaurus build with Chapters 1-8

### Chapter 9: Capstone Project

- [ ] T071 [P] [US4] Create context/09_capstone/readme.md
- [ ] T072 [P] [US4] Create context/09_capstone/outline.md
- [ ] T073 [P] [US4] Create context/09_capstone/references.md
- [ ] T074 [US4] Draft docs/docs/009-chapter-9-capstone.md with multi-skill integration
- [ ] T075 [US4] Test capstone project example (voice command → robot action in Isaac Sim)
- [ ] T076 [US4] Validate Chapter 9 (Stage 4.5: ≥3 skill integrations, success metrics ≥70%)
- [ ] T077 [US4] Test Docusaurus build with Chapters 1-9

**Checkpoint**: Layer 4 (Spec-Driven) complete - students ready for professional robotics development

---

## Phase 8: User Story 6 - Educator Validating Content Quality (Priority: P2)

**Goal**: Ensure all chapters meet constitution standards (zero untested code, zero factual errors)

**Independent Test**: All code executes successfully, all citations verified, safety compliance validated

### Content Quality Validation

- [ ] T078 [P] [US6] Run markdown-link-check on all chapter files to catch broken links
- [ ] T079 [P] [US6] Verify 100% code execution success across all chapters in Docker/VM
- [ ] T080 [P] [US6] Verify all technical claims cite official docs (ROS 2, NVIDIA, DOI/arXiv)
- [ ] T081 [P] [US6] Verify all robot control code includes safety mechanisms (velocity limits, emergency stop, timeouts)
- [ ] T082 [US6] Run full Docusaurus build test with all 11 chapters
- [ ] T083 [US6] Test search functionality returns relevant results for key terms
- [ ] T084 [US6] Validate all images have alt-text (accessibility check)

**Checkpoint**: Content quality validated - ready for student deployment

---

## Phase 9: Practice Chapters (10-11)

**Goal**: Add hardware deployment and safety chapters

**Independent Test**: Chapters provide standalone value for hardware setup and safety protocols

### Chapter 10: Hardware & Lab Setup

- [ ] T085 [P] Create context/10_hardware-lab-setup/readme.md
- [ ] T086 [P] Create context/10_hardware-lab-setup/outline.md
- [ ] T087 [P] Create context/10_hardware-lab-setup/references.md
- [ ] T088 Draft docs/docs/010-chapter-10-hardware-lab-setup.md with Jetson Orin setup
- [ ] T089 Validate Chapter 10 (Stage 4 checklist)
- [ ] T090 Test Docusaurus build with Chapters 1-10

### Chapter 11: Safety & Best Practices

- [ ] T091 [P] Create context/11_safety-best-practices/readme.md
- [ ] T092 [P] Create context/11_safety-best-practices/outline.md
- [ ] T093 [P] Create context/11_safety-best-practices/references.md
- [ ] T094 Draft docs/docs/011-chapter-11-safety-best-practices.md with safety protocols
- [ ] T095 Validate Chapter 11 (Stage 4 checklist + safety compliance)
- [ ] T096 Test Docusaurus build with all 11 chapters

**Checkpoint**: All 11 chapters complete

---

## Phase 10: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple chapters

- [ ] T097 [P] Validate all cross-references resolve correctly (grep check)
- [ ] T098 [P] Create consolidated bibliography of all references
- [ ] T099 Test final Docusaurus deployment (build + visual QA)
- [ ] T100 [P] Run constitution compliance check (all gates from plan.md)
- [ ] T101 Create PHR for task completion

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all chapter work
- **US5 Deployment (Phase 3)**: Depends on Foundational - validates infrastructure
- **US1 Foundation Chapters (Phase 4)**: Depends on US5 validation
- **US2 AI-Assisted Chapters (Phase 5)**: Depends on US1 completion (prerequisites)
- **US3 Intelligence Chapters (Phase 6)**: Depends on US2 completion
- **US4 Capstone Chapters (Phase 7)**: Depends on US3 completion (hard prerequisite: ≥3 skills)
- **US6 Quality Validation (Phase 8)**: Can run in parallel with chapter creation
- **Practice Chapters (Phase 9)**: Depends on US4 completion
- **Polish (Phase 10)**: Depends on all chapters complete

### User Story Dependencies

- **US5 (Deployment)**: No chapter dependencies - infrastructure validation
- **US1 (Ch 1-3)**: Sequential (Ch 2 requires Ch 1, Ch 3 requires Ch 2)
- **US2 (Ch 4-5)**: Depends on US1 (prerequisite knowledge), Ch 5 depends on Ch 4
- **US3 (Ch 6-7)**: Depends on US2, Ch 7 depends on Ch 6
- **US4 (Ch 8-9)**: Hard dependency on US3 (requires ≥3 skills), Ch 9 depends on Ch 8
- **US6 (Quality)**: Can start early and run continuously

### Within Each Chapter

- Context (readme.md, outline.md, references.md) before draft
- Draft before validation
- Validation before integration
- Code testing before quality validation
- Chapter complete before moving to next

### Parallel Opportunities

- All Setup tasks (T001-T005) can run in parallel
- All Foundational tasks marked [P] can run in parallel (T007-T008, T010)
- Within each chapter: readme/outline/references can be created in parallel
- US6 quality validation tasks (T078-T084) can run in parallel
- Different chapters within same layer can be worked on by different authors (if team capacity allows)

---

## Parallel Example: Chapter 1

```bash
# Context creation for Chapter 1 (parallel):
Task: "Create context/01_introduction-physical-ai/readme.md"
Task: "Create context/01_introduction-physical-ai/outline.md"
Task: "Create context/01_introduction-physical-ai/references.md"

# Sequential after context:
Task: "Draft docs/docs/001-chapter-1-introduction-physical-ai.md"
Task: "Validate Chapter 1 content"
Task: "Test Docusaurus build with Chapter 1"
```

---

## Implementation Strategy

### MVP First (Deploy Infrastructure + Chapter 1)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T010) - CRITICAL
3. Complete Phase 3: US5 Deployment validation (T011-T015)
4. Complete Chapter 1 only from Phase 4 (T016-T021)
5. **STOP and VALIDATE**: Test Chapter 1 independently
6. Deploy/demo if ready

### Incremental Delivery (Layer by Layer)

1. Complete Setup + Foundational + Deployment → Infrastructure ready
2. Add Layer 1 (Ch 1-3) → Test independently → Deploy/Demo (Foundation MVP!)
3. Add Layer 2 (Ch 4-5) → Test independently → Deploy/Demo
4. Add Layer 3 (Ch 6-7) → Test independently → Deploy/Demo
5. Add Layer 4 (Ch 8-9) → Test independently → Deploy/Demo
6. Add Practice (Ch 10-11) → Final polish → Full deployment

### Parallel Team Strategy

With multiple content authors:

1. Team completes Setup + Foundational together (T001-T010)
2. Team validates Deployment infrastructure (T011-T015)
3. Once infrastructure validated:
   - Author A: Chapter 1 (T016-T021)
   - Author B: Chapter 2 (T022-T028) after Ch 1 complete
   - Quality Reviewer: US6 validation tasks (T078-T084) continuously
4. Continue sequential chapter creation respecting prerequisites

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each chapter follows 5-stage workflow (Context → Outline → Draft → Validate → Deploy)
- Constitution compliance critical: zero untested code, zero factual errors, safety compliance
- Layer progression enforced: L1 (no AI) → L2 (AI-assisted) → L3 (skills) → L4 (spec-driven)
- Commit after each chapter validation
- Stop at checkpoints to validate layer completion
- Avoid: skipping validation, broken cross-references, untested code, missing citations
