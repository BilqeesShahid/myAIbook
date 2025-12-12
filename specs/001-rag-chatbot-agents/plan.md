# Implementation Plan: RAG Chatbot & OpenAI Agents Integration

**Branch**: `001-rag-chatbot-agents` | **Date**: 2025-12-07 | **Spec**: specs/001-rag-chatbot-agents/spec.md
**Input**: Feature specification from `/specs/001-rag-chatbot-agents/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a RAG (Retrieval-Augmented Generation) system for the deployed book with an interactive AI chatbot. The system will extract content from the `docs` folder, create embeddings using Gemini, store them in Qdrant Cloud, and provide FastAPI endpoints for querying with selected text prioritization. The frontend will use ChatKit JS to connect with the backend, supporting multi-turn conversations and Urdu output.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend
**Primary Dependencies**: FastAPI, Qdrant, Google Generative AI SDK (for Gemini), ChatKit, OpenAI Agents SDK, Pydantic, Markdown processing libraries
**Storage**: Qdrant Cloud Free Tier for vector embeddings, local file system for source content (docs folder)
**Testing**: pytest for backend, Jest for frontend components
**Target Platform**: Linux server (backend), Web browser (frontend)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: API latency < 500ms for response generation, handle multi-turn conversations efficiently
**Constraints**: <500ms p95 response time, secure prompt validation, support for Urdu text processing
**Scale/Scope**: Single user focused with potential for multi-user scaling, handle book-sized content (likely 100+ pages)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution:
- Content accuracy: RAG system must retrieve answers only from book content (✓ Compliant)
- Reproducibility: All examples must be executable in real or simulated environments (✓ Compliant)
- Code validation: All code snippets will be syntactically correct and aligned with functionality (✓ Compliant)
- Interactive content: RAG chatbot must accurately retrieve content based on book sections (✓ Compliant)
- Data sources: RAG chatbot retrieves only from book content and selected sections (✓ Compliant)
- Embeddings and queries: Must accurately represent textual content (✓ Compliant)
- Endpoints and integration: FastAPI endpoints must securely handle book content queries (✓ Compliant)

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-agents/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── embedding.py      # Embedding data models
│   │   ├── query.py          # Query request/response models
│   │   └── document.py       # Document chunk models
│   ├── services/
│   │   ├── embedding_service.py     # Embedding generation and storage
│   │   ├── retrieval_service.py     # Content retrieval from Qdrant
│   │   ├── rag_service.py           # RAG orchestration
│   │   ├── document_processor.py    # Markdown processing and chunking
│   │   └── chat_service.py          # Chat session management
│   ├── api/
│   │   ├── main.py           # FastAPI app
│   │   ├── routes/
│   │   │   ├── ask.py        # /ask endpoint
│   │   │   └── selected_text.py  # /ask/selected-text endpoint
│   │   └── middleware/
│   │       └── security.py   # Prompt validation middleware
│   └── utils/
│       ├── constants.py      # Configuration constants
│       └── helpers.py        # Utility functions
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── src/
│   ├── components/
│   │   ├── ChatInterface.jsx    # Main chat UI component
│   │   ├── Message.jsx          # Individual message component
│   │   ├── InputArea.jsx        # Text input with Urdu support
│   │   └── SelectionHandler.jsx # Selected text integration
│   ├── services/
│   │   ├── api.js              # API client for backend communication
│   │   └── chatManager.js      # Chat session management
│   ├── hooks/
│   │   └── useChat.js          # Chat state management
│   └── utils/
│       ├── textProcessor.js    # Text processing utilities
│       └── constants.js        # Frontend constants
└── tests/
    ├── unit/
    └── integration/

scripts/
├── setup/
│   └── embed_content.py        # Script to process docs folder and create embeddings
└── utils/
    └── validate_setup.py       # Setup validation script
```

**Structure Decision**: Web application structure chosen with separate backend (FastAPI) and frontend (React-based) to allow independent scaling and clear separation of concerns. The backend handles RAG processing and API endpoints while the frontend provides the chat interface with selected text functionality.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multi-component architecture | Required for clear separation of RAG processing (backend) and user interface (frontend) | Single component would mix concerns and reduce maintainability |
| Multiple dependencies (Qdrant, Gemini, ChatKit) | Needed to implement full RAG system with proper vector storage and AI capabilities | Simpler alternatives would not meet functional requirements for content retrieval |
