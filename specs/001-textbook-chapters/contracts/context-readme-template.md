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
