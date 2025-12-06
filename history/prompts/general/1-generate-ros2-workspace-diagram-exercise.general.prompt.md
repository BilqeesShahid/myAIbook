---
id: 1
title: Generate ROS2 Workspace Diagram and Exercise
stage: general
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: 001-robotics-book
user: user
command: Generate a Mermaid diagram that conceptually illustrates the typical directory structure of a ROS2 workspace, including `src`, `build`, `install`, and `log` directories. Also, generate a simple exercise for Chapter 2: ROS2 Workspaces and Your First Package, asking the reader to create a new workspace and a simple Python package within it, and verify its creation. Provide the output in markdown format, ready to be appended to the chapter file. Ensure the diagram is clear and easy to understand for beginners.
labels: ["ROS2", "workspace", "diagram", "exercise", "documentation"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- history/prompts/general/1-generate-ros2-workspace-diagram-exercise.general.prompt.md
tests:

---

## Prompt

Generate a Mermaid diagram that conceptually illustrates the typical directory structure of a ROS2 workspace, including `src`, `build`, `install`, and `log` directories. Also, generate a simple exercise for Chapter 2: ROS2 Workspaces and Your First Package, asking the reader to create a new workspace and a simple Python package within it, and verify its creation. Provide the output in markdown format, ready to be appended to the chapter file. Ensure the diagram is clear and easy to understand for beginners.

## Response snapshot

Generated a Mermaid diagram depicting a ROS2 workspace structure (ros2_ws with src, build, install, log directories) and a detailed exercise for creating a ROS2 workspace and a Python package, including steps for creation, building, sourcing, and verification using bash commands, formatted as markdown.

## Outcome

- ✅ Impact: Generated content for documentation
- 🧪 Tests: No tests were run as part of this task.
- 📁 Files: No files were created or modified by the agent.
- 🔁 Next prompts: User to review and potentially append the content to a chapter file.
- 🧠 Reflection: Successfully generated a Mermaid diagram and a practical exercise for a ROS2 chapter, adhering to the requested format.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
