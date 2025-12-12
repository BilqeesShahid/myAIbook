# Data Model: RAG Chatbot & OpenAI Agents Integration

## 1. Core Entities

### 1.1 DocumentChunk
Represents a chunk of content from the book that has been processed for RAG.

- **id**: string (UUID) - Unique identifier for the chunk
- **content**: string - The actual text content of the chunk
- **chapter_number**: integer - Chapter number from the source document
- **section_title**: string - Title of the section this chunk belongs to
- **source_file_path**: string - Path to the original Markdown file
- **embedding_vector**: array<float> - The Gemini embedding vector
- **chunk_index**: integer - Position of this chunk within the document
- **metadata**: object - Additional metadata (word count, headings, etc.)

### 1.2 ChatSession
Represents a user's conversation session with the chatbot.

- **session_id**: string (UUID) - Unique identifier for the session
- **user_id**: string (UUID) - Identifier for the user (optional for anonymous)
- **created_at**: datetime - Timestamp when the session was created
- **updated_at**: datetime - Timestamp of last activity
- **title**: string - Generated title for the conversation
- **is_active**: boolean - Whether the session is currently active

### 1.3 ChatMessage
Represents a single message in a conversation.

- **message_id**: string (UUID) - Unique identifier for the message
- **session_id**: string (UUID) - Reference to the parent session
- **role**: string (enum: "user", "assistant", "system") - The role of the message sender
- **content**: string - The text content of the message
- **timestamp**: datetime - When the message was created
- **language**: string - Language of the message (e.g., "en", "ur")
- **context_chunks**: array<string> - IDs of document chunks used to generate this response

### 1.4 QueryRequest
Represents a request made to the RAG system.

- **query_id**: string (UUID) - Unique identifier for the query
- **session_id**: string (UUID) - Reference to the session (if applicable)
- **query_text**: string - The original query text
- **selected_text**: string (optional) - Text selected by the user (for /ask/selected-text)
- **query_type**: string (enum: "general", "selected_text") - Type of query
- **timestamp**: datetime - When the query was made
- **user_context**: object - Additional context from the user session

### 1.5 QueryResponse
Represents a response from the RAG system.

- **response_id**: string (UUID) - Unique identifier for the response
- **query_id**: string (UUID) - Reference to the original query
- **response_text**: string - The generated response text
- **source_chunks**: array<string> - IDs of chunks used to generate the response
- **confidence_score**: float - Confidence score for the response (0-1)
- **timestamp**: datetime - When the response was generated
- **language**: string - Language of the response (e.g., "en", "ur")

## 2. Relationships

### 2.1 Session-Message Relationship
- One ChatSession can have many ChatMessages
- Foreign key: ChatMessage.session_id references ChatSession.session_id
- Cardinality: One-to-Many

### 2.2 Query-Response Relationship
- One QueryRequest has one QueryResponse
- Foreign key: QueryResponse.query_id references QueryRequest.query_id
- Cardinality: One-to-One

### 2.3 Message-Chunk Relationship
- One ChatMessage can reference many DocumentChunks (via context_chunks)
- Many ChatMessages can reference the same DocumentChunks
- Cardinality: Many-to-Many (through context reference)

## 3. Validation Rules

### 3.1 DocumentChunk Validation
- content: Required, minimum 10 characters, maximum 10000 characters
- chapter_number: Required, must be positive integer
- section_title: Required, maximum 200 characters
- source_file_path: Required, valid file path format
- embedding_vector: Required, must have consistent dimensions
- chunk_index: Required, must be non-negative integer

### 3.2 ChatSession Validation
- session_id: Required, valid UUID format
- created_at: Required, must be in the past
- updated_at: Required, must be equal to or after created_at
- title: Optional, maximum 200 characters

### 3.3 ChatMessage Validation
- session_id: Required, valid UUID format
- role: Required, must be one of allowed values
- content: Required, minimum 1 character, maximum 10000 characters
- timestamp: Required, must be in the past
- language: Required, valid language code format

### 3.4 QueryRequest Validation
- query_text: Required, minimum 1 character, maximum 1000 characters
- selected_text: Optional, maximum 5000 characters
- timestamp: Required, must be in the past

### 3.5 QueryResponse Validation
- response_text: Required, minimum 1 character, maximum 10000 characters
- confidence_score: Required, between 0 and 1
- timestamp: Required, must be in the past
- language: Required, valid language code format

## 4. State Transitions

### 4.1 ChatSession States
- **Active**: New session created, ready for messages
- **Inactive**: No activity for a period (e.g., 24 hours)
- **Archived**: Session completed and moved for storage

Transition rules:
- Active → Inactive: After 24 hours of inactivity
- Inactive → Active: New message added
- Inactive → Archived: After 30 days of inactivity

## 5. Indexes

### 5.1 DocumentChunk Indexes
- Primary: id (UUID)
- Secondary: source_file_path (for content updates)
- Vector: embedding_vector (for similarity search)
- Composite: (chapter_number, chunk_index) for content ordering

### 5.2 ChatSession Indexes
- Primary: session_id (UUID)
- Secondary: user_id (for user session retrieval)
- Secondary: updated_at (for session ordering)

### 5.3 ChatMessage Indexes
- Primary: message_id (UUID)
- Secondary: session_id (for session queries)
- Secondary: timestamp (for chronological ordering)