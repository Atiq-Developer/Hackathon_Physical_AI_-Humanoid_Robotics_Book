<!--
Sync Impact Report:
Version change: 0.0.0 → 1.0.0
Modified principles:
- [PRINCIPLE_1_NAME] → I. Technical Accuracy
- [PRINCIPLE_2_NAME] → II. Engineering Clarity
- [PRINCIPLE_3_NAME] → III. Reproducibility
- [PRINCIPLE_4_NAME] → IV. Embodiment-First Thinking
Added sections:
- Key Standards
- Constraints and Success Criteria
Removed sections:
- [PRINCIPLE_5_NAME]
- [PRINCIPLE_6_NAME]
Templates requiring updates:
- ✅ .specify/templates/plan-template.md
- ✅ .specify/templates/spec-template.md
- ✅ .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation — Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Technical Accuracy
All robotics, AI, and simulation claims must align with official documentation or peer-reviewed research.

### II. Engineering Clarity
Written for senior undergraduate, graduate, and professional AI/robotics learners. The writing clarity target is a Flesch-Kincaid grade level of 11–13.

### III. Reproducibility
All system designs, pipelines, and workflows must be conceptually reproducible.

### IV. Embodiment-First Thinking
AI must always be grounded in physical constraints, including physics, sensors, latency, and actuation.

## Key Standards

- All factual claims must be traceable to:
  - Official docs (ROS 2, NVIDIA Isaac, Gazebo, Unity)
  - Peer-reviewed robotics / AI research
- Citation format: APA
- Source mix:
  - ≥50% peer-reviewed papers
  - Remaining from official SDK or platform documentation
- Code examples must be conceptually correct and idiomatic.

## Constraints and Success Criteria

### Constraints
- Total length: One full instructional book
- Format:
  - Docusaurus-compatible Markdown
  - Chapter-based structure
- Code language:
  - Python (ROS 2, rclpy)
  - YAML (ROS configs)
- Diagrams: described textually (ASCII or Mermaid-compatible)

### Success Criteria
- Readers can:
  - Explain Physical AI and embodied intelligence
  - Build and simulate a humanoid robot control pipeline
  - Connect LLMs to real robotic action
- All modules map cleanly to weekly curriculum.
- Zero hallucinated APIs or fake citations.

## Governance

This constitution is the authoritative guide for the project. All specifications, plans, and artifacts must comply with these principles. Amendments require review and approval, and the version number must be updated according to semantic versioning.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16