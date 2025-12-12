# Research Summary: RAG Chatbot & OpenAI Agents Integration

## 1. Technology Decisions

### 1.1 Embedding Technology - Gemini vs Alternatives
- **Decision**: Use Google Generative AI SDK with Gemini for embeddings
- **Rationale**:
  - Strong performance for text embeddings
  - Good integration with other Google AI services
  - Supports multiple languages including Urdu
  - Cost-effective for the project scope
- **Alternatives considered**:
  - OpenAI embeddings: More expensive, less language support for Urdu
  - Sentence Transformers: Self-hosted option but requires more infrastructure
  - Cohere embeddings: Good performance but less integration with other services

### 1.2 Vector Database - Qdrant Cloud
- **Decision**: Use Qdrant Cloud Free Tier for vector storage
- **Rationale**:
  - Efficient similarity search for RAG applications
  - Managed service reduces operational overhead
  - Good performance for the expected content size
  - Free tier sufficient for initial development
- **Alternatives considered**:
  - Pinecone: More expensive, similar functionality
  - Weaviate: Open source but requires self-hosting
  - FAISS: Self-hosted option with more setup required

### 1.3 Backend Framework - FastAPI
- **Decision**: Use FastAPI for backend API
- **Rationale**:
  - Automatic API documentation with Swagger/OpenAPI
  - Fast performance with ASGI
  - Excellent validation with Pydantic
  - Good async support for I/O operations
- **Alternatives considered**:
  - Flask: More familiar but slower, less automatic documentation
  - Django: Overkill for API-only application
  - Express.js: Node.js option but Python preferred for ML integration

### 1.4 Frontend Integration - ChatKit JS
- **Decision**: Use ChatKit JS for frontend chat interface
- **Rationale**:
  - Pre-built chat UI components
  - Easy integration with custom backends
  - Supports multi-turn conversations
  - Extensible for custom features like Urdu input
- **Alternatives considered**:
  - Custom React chat components: More control but more development time
  - Socket.io with custom UI: More complex for basic chat functionality
  - Third-party chat widgets: Less customization options

## 2. Architecture Decisions

### 2.1 Content Processing Pipeline
- **Decision**: Process content from `docs/` folder with automatic chunking
- **Rationale**:
  - Leverages existing Docusaurus content structure
  - Automatic updates when docs are modified
  - Logical chunking (300-500 words) preserves context
- **Implementation approach**:
  - Read all Markdown files from docs folder
  - Parse and extract text content
  - Split into semantically coherent chunks
  - Generate embeddings for each chunk
  - Store with metadata (chapter, section, file path)

### 2.2 Selected Text Priority Mechanism
- **Decision**: Implement dual endpoint approach with priority context
- **Rationale**:
  - Separate endpoints for different use cases
  - Maintains clean API design
  - Allows different processing logic for selected vs general queries
- **Approach**:
  - `POST /ask`: General content queries
  - `POST /ask/selected-text`: Prioritizes selected text in context

### 2.3 Multi-turn Conversation Support
- **Decision**: Implement session-based conversation tracking
- **Rationale**:
  - Enables context-aware responses
  - Supports complex queries that span multiple interactions
  - Improves user experience
- **Implementation**:
  - Session IDs to track conversation history
  - Context window management
  - History summarization for long conversations

## 3. Language Support - Urdu
- **Decision**: Implement Urdu text processing capabilities
- **Rationale**:
  - Requirement specified in feature spec
  - Important for target audience
  - Gemini supports Urdu for generation
- **Considerations**:
  - Text encoding and processing
  - Frontend input/output handling
  - Tokenization differences from English

## 4. Security & Performance
- **Decision**: Implement prompt validation and performance monitoring
- **Rationale**:
  - Prevents injection attacks
  - Maintains response quality
  - Meets 500ms latency requirement
- **Approach**:
  - Input validation middleware
  - Rate limiting
  - Response time monitoring
  - Sanitization of outputs

## 5. OpenAI Agents SDK Integration
- **Decision**: Use OpenAI Agents SDK for skill orchestration
- **Rationale**:
  - Handles multi-step reasoning
  - Manages complex workflows
  - Integrates well with RAG system
- **Skills to implement**:
  - Summarization
  - Translation (Urdu/English)
  - Content retrieval and ranking
  - ROS code validation
  - Simulation guidance