# 📌 Phase 2 — RAG Chatbot & OpenAI Agents Integration (Updated v2)

## 1. Overview

Integrate a **RAG (Retrieval-Augmented Generation) system** into the deployed book to provide an **interactive AI chatbot**.

**New Features:**

- Book content is dynamically read from the **`docs` folder** in VS Code
- Automatic content chunking for embeddings
- FastAPI endpoint supports **selected text functionality**, with frontend mechanism moving selected text into chatbot input
- Multi-turn conversation and Urdu output supported

---

## 2. Book Content Extraction & Embeddings

- **Source of Content:**
  - Local `docs/` folder in VS Code
  - Includes all Markdown files organized by chapters and sections

- **Chunking & Embedding Script:**
  - Read Markdown content and split into **logical chunks** (300–500 words or paragraph-level)
  - Generate **Gemini embeddings** for each chunk
  - Store embeddings in **Qdrant Cloud Free Tier** with metadata:
    - `chapter_number`
    - `section_title`
    - `source_file_path`

- **Query Rules:**
  - Answer **only from book content**
  - Prioritize **selected text** if user provides
  - Support multi-turn context

---

## 3. Backend Architecture

- **Framework:** FastAPI
- **Endpoints:**
  1. `POST /ask` → Retrieve answers from book content
  2. `POST /ask/selected-text` → Retrieve answers **prioritizing selected text**

- **Selected Text Flow:**
  - User highlights text on frontend
  - A **"Ask Chatbot" button appears**
  - Clicking the button sends selected text to `/ask/selected-text`
  - Backend uses this text as **priority context** for response generation

- **Security & Performance:**
  - Validate all prompts
  - API latency < **500ms**
  - Prevent untrusted prompts from affecting system

- **Skills Supported:**
  - Summarization
  - Translation (Urdu/English)
  - Retrieval & ranking
  - ROS code check & validation
  - Simulation guidance / verification

---

## 4. Frontend Integration

- **Framework:** Optional React / Docusaurus page
- **Chat UI:**
  - **ChatKit JS** for frontend
  - Connects to **ChatKit Python backend via FastAPI**
  - Features:
    - Multi-turn conversation
    - Urdu text input/output
    - **Selected text injection:** highlight → button → auto-fills chatbot input
    - Context-aware suggestions

---

## 5. OpenAI Agents SDK Integration

- **Agents SDK Tasks:**
  - Orchestrate multiple skills (summarization, translation, code check, simulation)
  - Manage multi-step reasoning
  - Fetch relevant context from **Qdrant embeddings**
  - Handle both English and Urdu content

---

## 6. Deployment Guidelines

- **Phase 1:** Deploy Docusaurus book (already done)
- **Phase 2:**
  - Build **FastAPI backend** with Qdrant integration
  - Implement **selected text functionality**
  - Connect **ChatKit JS frontend**
- **Folder structure:**
