# API Contract: RAG Chatbot System

## Overview
This document defines the API contracts for the RAG (Retrieval-Augmented Generation) chatbot system. The API allows users to query book content with support for selected text prioritization.

## Base URL
`http://localhost:8000` (development)
`https://[deployed-url]` (production)

## Common Headers
- `Content-Type: application/json`
- `Accept: application/json`

## Endpoints

### 1. General Content Query
#### `POST /ask`

Query the RAG system for information from the book content.

##### Request
```json
{
  "query": "What are the main concepts in chapter 3?",
  "session_id": "unique-session-id",
  "language": "en",
  "context_window": 5
}
```

**Request Parameters:**
- `query` (string, required): The question or query text
- `session_id` (string, optional): Session identifier for conversation history
- `language` (string, optional): Preferred response language (default: "en")
- `context_window` (integer, optional): Number of previous messages to include in context (default: 5)

##### Response
```json
{
  "response_id": "response-uuid",
  "response": "The main concepts in chapter 3 include...",
  "source_chunks": [
    {
      "chunk_id": "chunk-uuid",
      "chapter_number": 3,
      "section_title": "Key Concepts",
      "source_file_path": "docs/chapter3.md",
      "confidence_score": 0.85
    }
  ],
  "session_id": "unique-session-id",
  "timestamp": "2025-12-07T10:30:00Z",
  "language": "en"
}
```

**Response Parameters:**
- `response_id` (string): Unique identifier for the response
- `response` (string): The generated response text
- `source_chunks` (array): List of source chunks used to generate the response
- `session_id` (string): Session identifier
- `timestamp` (string): ISO 8601 timestamp of response generation
- `language` (string): Language of the response

##### Error Responses
- `400 Bad Request`: Invalid request parameters
- `422 Unprocessable Entity`: Validation error
- `500 Internal Server Error`: Processing error

### 2. Selected Text Query
#### `POST /ask/selected-text`

Query the RAG system with priority given to selected text from the book.

##### Request
```json
{
  "query": "Explain this concept further",
  "selected_text": "The concept of artificial intelligence involves...",
  "session_id": "unique-session-id",
  "language": "en",
  "context_window": 5
}
```

**Request Parameters:**
- `query` (string, required): The question or query text
- `selected_text` (string, required): The text selected by the user
- `session_id` (string, optional): Session identifier for conversation history
- `language` (string, optional): Preferred response language (default: "en")
- `context_window` (integer, optional): Number of previous messages to include in context (default: 5)

##### Response
```json
{
  "response_id": "response-uuid",
  "response": "Based on the selected text about artificial intelligence...",
  "source_chunks": [
    {
      "chunk_id": "chunk-uuid",
      "chapter_number": 2,
      "section_title": "Introduction to AI",
      "source_file_path": "docs/chapter2.md",
      "confidence_score": 0.92
    },
    {
      "chunk_id": "chunk-uuid-2",
      "chapter_number": 2,
      "section_title": "AI Concepts",
      "source_file_path": "docs/chapter2.md",
      "confidence_score": 0.88,
      "is_selected_text": true
    }
  ],
  "session_id": "unique-session-id",
  "timestamp": "2025-12-07T10:30:00Z",
  "language": "en"
}
```

**Response Parameters:**
- `response_id` (string): Unique identifier for the response
- `response` (string): The generated response text
- `source_chunks` (array): List of source chunks used to generate the response (with priority given to selected text)
- `session_id` (string): Session identifier
- `timestamp` (string): ISO 8601 timestamp of response generation
- `language` (string): Language of the response

##### Error Responses
- `400 Bad Request`: Invalid request parameters
- `422 Unprocessable Entity`: Validation error
- `500 Internal Server Error`: Processing error

### 3. Session Management
#### `POST /session/new`

Create a new chat session.

##### Request
```json
{
  "user_id": "optional-user-id",
  "session_title": "Initial session title"
}
```

**Request Parameters:**
- `user_id` (string, optional): User identifier
- `session_title` (string, optional): Initial title for the session

##### Response
```json
{
  "session_id": "new-session-uuid",
  "created_at": "2025-12-07T10:30:00Z",
  "title": "Generated session title"
}
```

### 4. Get Session History
#### `GET /session/{session_id}/history`

Retrieve the conversation history for a session.

##### Path Parameters
- `session_id` (string): The session identifier

##### Response
```json
{
  "session_id": "session-uuid",
  "messages": [
    {
      "message_id": "message-uuid",
      "role": "user",
      "content": "What are the main concepts in chapter 3?",
      "timestamp": "2025-12-07T10:25:00Z"
    },
    {
      "message_id": "message-uuid-2",
      "role": "assistant",
      "content": "The main concepts in chapter 3 include...",
      "timestamp": "2025-12-07T10:25:05Z",
      "source_chunks": ["chunk-uuid"]
    }
  ]
}
```

## Data Models

### QueryRequest
```json
{
  "query": "string",
  "session_id": "string (optional)",
  "selected_text": "string (optional)",
  "language": "string (optional, default: 'en')",
  "context_window": "integer (optional, default: 5)"
}
```

### QueryResponse
```json
{
  "response_id": "string",
  "response": "string",
  "source_chunks": [
    {
      "chunk_id": "string",
      "chapter_number": "integer",
      "section_title": "string",
      "source_file_path": "string",
      "confidence_score": "number",
      "is_selected_text": "boolean (optional)"
    }
  ],
  "session_id": "string",
  "timestamp": "string (ISO 8601)",
  "language": "string"
}
```

### ChatMessage
```json
{
  "message_id": "string",
  "session_id": "string",
  "role": "string (user|assistant|system)",
  "content": "string",
  "timestamp": "string (ISO 8601)",
  "language": "string",
  "context_chunks": ["string"]
}
```

## Error Format
All error responses follow this format:
```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "object (optional)"
  }
}
```

## Authentication
This API does not require authentication for basic functionality, but may require API keys for production deployment.

## Rate Limiting
The API implements rate limiting to prevent abuse. Standard limits apply per IP address.