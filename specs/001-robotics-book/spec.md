## Project: Physical AI & Humanoid Robotics

### Overview
This project delivers a **modular, interactive, AI-augmented book** using Docusaurus.
Navigation will include a custom **MyBook section (replacing Tutorials)**, and its main index page will contain **responsive UI cards** representing modules and assessments.
Once the book is complete and deployed, a **Qdrant-powered RAG chatbot** will use the hosted pages as the knowledge base.

---

### Functional Requirements

#### 1. Docusaurus Navigation & UI Requirements

##### 🔹 Navbar
- Replace **Tutorials → MyBook**
- Keep other items (`MySite`, `Blog`) unchanged.

##### 🔹 Sidebar Structure
- `tutorial-info` renamed to **book title page**
- Sidebar indices for modules start at **0**
- Folder-based structure remains:
  ```
  /docs/<module>/<chapter>.md
  ```

##### 🔹 Book Index Page (Previous `tutorial-info`)
This page must be a custom UI page (JSX allowed & styled).
It must include **6 responsive card components (2-per-row flex)**:

| Card | Content |
|------|---------|
| 1–5 | Each card shows: Module #, Module Name & bullet chapter list |
| 6 | **Assessments & Quiz Strategy** (quiz plan per module) |

UI Rules:
- Flex layout: **3 rows × 2 columns**
- Cards include:
  - Module number heading
  - Module title
  - Bullet list of chapters from `/docs/<module>/`
  - Assessment card lists quiz features:
    - **MCQs + Code Completion + Simulation Tasks**
    - One quiz per module

#### 2. Book Structure & Required Content

Modules (MANDATORY CONTENT):
1. **ROS2 Fundamentals**
2. **Robot Simulation (Gazebo & Unity)**
3. **AI-Robot Brain (NVIDIA Isaac)**
4. **Vision-Language-Action (VLA)**
5. **Capstone Project (Autonomous Humanoid)**

##### Folder Requirements
```
/docs/0-ros2/<chapters>.md
/docs/1-simulation/<chapters>.md
/docs/2-isaac/<chapters>.md
/docs/3-vla/<chapters>.md
/docs/4-capstone/<chapters>.md
```

##### Sidebar Requirements
- Every module & every chapter MUST be clickable.
- Titles must match the book pages (not folder names).
- No empty links or commented items allowed.

---

### **Phased Workflow Requirement (MANDATORY)**

#### 📌 Phase 1 — Content Creation (Claude-only)
- Claude agents generate **all modules, chapters, quizzes, diagrams, exercises, Urdu translations**
- Must follow constitution rules (accuracy, robotics validation, reproducibility)
- **No RAG, no backend, no embeddings until book is complete**
- On completion: deploy on **GitHub Pages**

#### 📌 Phase 2 — RAG Chatbot Integration
- Once book is deployed, extract content using its public URLs
- Create embeddings using **Qdrant Cloud Free Tier**
- Use **OpenAI Agents SDK + FastAPI backend**
- Must support two endpoints:
  - `POST /ask`
  - `POST /ask/selected-text`
- Retrieval rules:
  - Only answer from deployed content
  - Prioritize selected text if provided
- Supports **multi-turn context and Urdu output**

---

### Backend + Chatbot Requirements

- **FastAPI** backend
- **Vector DB:** Qdrant Cloud
- **Agents SDK:** OpenAI Agents
- **Skills:** summarization, translation, retrieval, ranking, ROS code check, simulation validation.

---

### Non-Functional Requirements
- Deployment first, then RAG
- Sidebar reliable on all devices
- API latency < **500ms**
- Secure access (no untrusted prompts)
- Maintainable folder separation:
  - `/docs (book)`
  - `/backend (API + RAG)`
  - `/frontend (optional chat UI)`

---

### Success Criteria
✔ MyBook replaces Tutorials
✔ Book index = 6-card responsive UI
✔ Modules & chapters fully generated before RAG
✔ GitHub Pages deployment successful
✔ Qdrant embeddings from deployed URLs
✔ `/ask` and `/ask/selected-text` return accurate answers
✔ Responses are explainable, validated, optionally in Urdu
