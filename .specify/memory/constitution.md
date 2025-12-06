<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0 (minor bump for new structural rule on docs folder and sidebar)
List of modified principles: Structural Rules
Added sections: None
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - CLAUDE.md: ✅ updated (this file itself)
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Accuracy
All technical content, code snippets, diagrams, and robotics explanations MUST be factually correct and validated against authoritative sources (ROS2, Gazebo, NVIDIA Isaac, OpenAI docs).

### Clarity
Writing MUST be professional, precise, and understandable by students with AI/robotics background.

### Reproducibility
All examples, simulations, and exercises MUST be executable in real or simulated environments.

### Rigor
Use reliable sources: research papers, official SDK documentation, technical manuals.

## Content and Structural Guidelines

### Content Rules
- **Citation & References:** All factual statements MUST include a source. Prefer APA style.
- **Source Quality:** Minimum 50% of references MUST be peer-reviewed papers, official SDK documentation, or authoritative whitepapers.
- **Originality:** No plagiarism; all content MUST be original or properly cited.
- **Readability:** Maintain Flesch-Kincaid grade 10–12 level for technical content.
- **Diagram Accuracy:** All diagrams (robot kinematics, URDF, VLA workflows, sensor setups) MUST reflect real-world robotics and physics principles.
- **Code Validation:** All ROS2, Gazebo, and Isaac code snippets MUST be syntactically correct, executable, and align with described functionality.
- **Interactive Content:** RAG chatbot and personalization modules MUST accurately retrieve or manipulate content based on the book’s sections.

### Structural Rules
- **Modular Chapters:** Divide book into modules corresponding to course structure:
  - ROS2 Fundamentals
  - Robot Simulation (Gazebo & Unity)
  - AI-Robot Brain (NVIDIA Isaac)
  - Vision-Language-Action (VLA)
  - Capstone Project (Autonomous Humanoid)
- **Chapter Length:** Each module SHOULD contain 1,500–3,000 words with structured subsections, diagrams, and exercises.
- **Consistency:** Terminology, formatting, and style MUST remain consistent across all chapters and modules.
- **Translation & Accessibility:** Optional content (e.g., Urdu translation, interactive explanations) MUST remain faithful to the original technical meaning.
- **Docs Folder & Sidebar:** All markdown files for chapters MUST be placed in `/docs`. The `sidebar.js` MUST include all modules and chapters with **clickable routes**, starting from **0 index** for your book modules after any existing tutorial sections.

## Technical, Hardware, and RAG Guidelines

### Technical & Hardware Guidelines
- **Simulation First:** Ensure all robotics examples CAN be simulated using Gazebo, Isaac Sim, or Unity before deploying to real hardware.
- **Edge Deployment:** Edge AI kits (Jetson Orin Nano/NX, RealSense sensors) MUST run ROS2 nodes correctly for physical deployment exercises.
- **Cloud Use:** If using cloud simulation, ensure all downloadable models and instructions are provided for local deployment to mitigate latency issues.
- **Safety & Feasibility:** All physical AI exercises MUST be safe, feasible, and clearly documented for students with minimal physical robot experience.

### Interactive RAG Guidelines
- **Data Sources:** RAG chatbot MUST retrieve answers only from book content and explicitly selected sections.
- **Embeddings & Queries:** Embeddings SHOULD accurately represent textual content; vector database queries MUST reflect intended sections.
- **Endpoints & Integration:** FastAPI endpoints MUST securely handle book content queries without introducing errors or conflicts in other modules.

## Agents & Skills Integration

### Agents
- Orchestrator-Agent.md
- Writing-Sub-Agent.md
- Research-Sub-Agent.md
- Editing-Sub-Agent.md
- ROS2-Code-Agent.md
- Gazebo-Simulation-Agent.md
- NVIDIA-Isaac-Agent.md
- VLA-Agent.md
- Capstone-Project-Agent.md
- RAG-Chatbot-Agent.md

### Skills
- topic-breakdown/SKILL.md
- chapter-outline/SKILL.md
- paragraph-writer/SKILL.md
- robotics-terminology-guardian/SKILL.md
- physics-formula-validator/SKILL.md
- diagram-description-enhancer/SKILL.md
- grammar-and-style-pro/SKILL.md
- educational-simplifier/SKILL.md
- section-summarizer-pro/SKILL.md
- chapter-translation-urdu-pro/SKILL.md
- consistency-and-coherence-auditor/SKILL.md
- RAG-master-skill/SKILL.md

## Governance
This constitution supersedes all other practices; Amendments require documentation, approval, and a migration plan. All PRs/reviews MUST verify compliance. Complexity MUST be justified.

**Version**: 1.1.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04

This constitution ensures **all chapters, agents, and skills are modular, reproducible, and compliant**, with **docs folder structure, sidebar routing, and clickable module routes fully enforced**.