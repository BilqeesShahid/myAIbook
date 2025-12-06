---
id: 0001
title: Define Physical AI Humanoid Robotics Constitution
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["project-setup", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

```text
# /sp.constitution

## Project: Physical AI & Humanoid Robotics

### Core Principles
- **Accuracy:** All technical content, code snippets, diagrams, and robotics explanations must be factually correct and validated against authoritative sources (ROS2, Gazebo, NVIDIA Isaac, OpenAI docs).
- **Clarity:** Writing must be professional, precise, and understandable by students with AI/robotics background.
- **Reproducibility:** All examples, simulations, and exercises must be executable in real or simulated environments.
- **Rigor:** Use reliable sources: research papers, official SDK documentation, technical manuals.

---

### Content Rules
- **Citation & References:** All factual statements must include a source. Prefer APA style.
- **Source Quality:** Minimum 50% of references must be peer-reviewed papers, official SDK documentation, or authoritative whitepapers.
- **Originality:** No plagiarism; all content must be original or properly cited.
- **Readability:** Maintain Flesch-Kincaid grade 10–12 level for technical content.
- **Diagram Accuracy:** All diagrams (robot kinematics, URDF, VLA workflows, sensor setups) must reflect real-world robotics and physics principles.
- **Code Validation:** All ROS2, Gazebo, and Isaac code snippets must be syntactically correct, executable, and align with described functionality.
- **Interactive Content:** RAG chatbot and personalization modules must accurately retrieve or manipulate content based on the book’s sections.

---

### Structural Rules
- **Modular Chapters:** Divide book into modules corresponding to course structure:
  - ROS2 Fundamentals
  - Robot Simulation (Gazebo & Unity)
  - AI-Robot Brain (NVIDIA Isaac)
  - Vision-Language-Action (VLA)
  - Capstone Project (Autonomous Humanoid)
- **Chapter Length:** Each module should contain 1,500–3,000 words with structured subsections, diagrams, and exercises.
- **Consistency:** Terminology, formatting, and style must remain consistent across all chapters and modules.
- **Translation & Accessibility:** Optional content (e.g., Urdu translation, interactive explanations) must remain faithful to the original technical meaning.

---

### Technical & Hardware Guidelines
- **Simulation First:** Ensure all robotics examples can be simulated using Gazebo, Isaac Sim, or Unity before deploying to real hardware.
- **Edge Deployment:** Edge AI kits (Jetson Orin Nano/NX, RealSense sensors) must run ROS2 nodes correctly for physical deployment exercises.
- **Cloud Use:** If using cloud simulation, ensure all downloadable models and instructions are provided for local deployment to mitigate latency issues.
- **Safety & Feasibility:** All physical AI exercises must be safe, feasible, and clearly documented for students with minimal physical robot experience.

---

### Interactive RAG Guidelines
- **Data Sources:** RAG chatbot must retrieve answers only from book content and explicitly selected sections.
- **Embeddings & Queries:** Embeddings should accurately represent textual content; vector database queries must reflect intended sections.
- **Endpoints & Integration:** FastAPI endpoints must securely handle book content queries without introducing errors or conflicts in other modules.

---

### Agents & Skills Integration
- **Agents:**
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

- **Skills:**
  - topic-breakdown/SKILL.md
  - chapter-outline/SKILL.md
  - paragraph-writer/SKILL.md
  - robotics-terminology-guardian/SKILL.md
  - physics-formula-validator/SKILL.md
  - diagram-description-enhancer/SKILL.md
  - grammar-and-style-pro/SKILL.md
  - educational-simplifier/SKILL.md
  - se
```

## Response snapshot

```text
The project constitution has been updated to **Version 1.0.0**. This is a major version bump as the constitution has been fully defined, replacing the initial placeholder template.

The following templates require manual follow-up to ensure alignment with the new constitution:
- `.specify/templates/plan-template.md`
- `.specify/templates/spec-template.md`
- `.specify/templates/tasks-template.md`

Suggested commit message:
`docs: define project constitution to v1.0.0 (initial definition)`
```

## Outcome

- ✅ Impact: Project constitution defined and updated.
- 🧪 Tests: None required for this task.
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Review pending template updates.
- 🧠 Reflection: Successfully updated constitution and generated PHR.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
