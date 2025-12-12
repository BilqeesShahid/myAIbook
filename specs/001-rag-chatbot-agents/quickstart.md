# Quickstart Guide: RAG Chatbot & OpenAI Agents Integration

## Overview
This guide will help you set up and run the RAG chatbot system with OpenAI Agents integration. The system extracts content from your `docs/` folder, creates embeddings, and provides an interactive chatbot interface.

## Prerequisites
- Python 3.11 or higher
- Node.js 16 or higher (for frontend, if applicable)
- Qdrant Cloud account (Free Tier)
- Google AI API key for Gemini embeddings
- Access to the `docs/` folder with your book content

## Installation

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd <repo-name>
```

### 2. Set up Backend Environment
```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file in the backend directory:

```env
QDRANT_URL=<your-qdrant-cluster-url>
QDRANT_API_KEY=<your-qdrant-api-key>
GOOGLE_API_KEY=<your-google-ai-api-key>
FASTAPI_ENV=development
```

## Content Processing

### 1. Prepare Your Book Content
Ensure your book content is in the `docs/` folder in Markdown format.

### 2. Generate Embeddings
Run the content embedding script to process your `docs/` folder:

```bash
python scripts/setup/embed_content.py
```

This script will:
- Read all Markdown files from the `docs/` folder
- Chunk the content (300-500 words per chunk)
- Generate Gemini embeddings for each chunk
- Store the embeddings in Qdrant with metadata

## Running the Application

### 1. Start the Backend Server
```bash
cd backend
source venv/bin/activate  # If not already activated
uvicorn src.api.main:app --reload --port 8000
```

The API will be available at `http://localhost:8000`.

### 2. API Endpoints
- `POST /ask` - General content queries
- `POST /ask/selected-text` - Queries prioritizing selected text
- `GET /docs` - API documentation

### 3. Frontend Setup (Optional)
If using the frontend interface:

```bash
cd frontend
npm install
npm start
```

## Using the RAG System

### 1. Making Queries
To ask a question about your book content:

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the main concepts in chapter 3?",
    "session_id": "unique-session-id"
  }'
```

### 2. Using Selected Text Feature
To prioritize selected text in your query:

```bash
curl -X POST http://localhost:8000/ask/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain this concept further",
    "selected_text": "The concept of artificial intelligence involves...",
    "session_id": "unique-session-id"
  }'
```

## OpenAI Agents Integration

The system supports various skills through OpenAI Agents:

- **Summarization**: Summarize book sections
- **Translation**: Translate between English and Urdu
- **Content Retrieval**: Find specific information
- **ROS Code Validation**: Check ROS code snippets
- **Simulation Guidance**: Provide simulation guidance

## Testing

### Backend Tests
```bash
cd backend
python -m pytest tests/
```

### Frontend Tests (if applicable)
```bash
cd frontend
npm test
```

## Configuration Options

### Environment Variables
- `QDRANT_URL`: Qdrant cluster URL
- `QDRANT_API_KEY`: Qdrant API key
- `GOOGLE_API_KEY`: Google AI API key
- `EMBEDDING_DIMENSION`: Dimension of embeddings (default: 768)
- `CHUNK_SIZE`: Size of text chunks (default: 400 words)
- `MAX_CONTEXT_LENGTH`: Maximum context length for responses

## Troubleshooting

### Common Issues

1. **Embedding Generation Fails**
   - Check that Google API key is valid
   - Verify internet connectivity
   - Ensure `docs/` folder contains valid Markdown files

2. **API Returns Empty Results**
   - Verify content was properly embedded
   - Check Qdrant connection
   - Confirm query matches content in your book

3. **Slow Response Times**
   - Check Qdrant cluster performance
   - Verify Google API quota
   - Review system resources

### Useful Commands
```bash
# Check if Qdrant is accessible
curl -X GET <your-qdrant-url>/collections

# Validate embeddings exist
python scripts/utils/validate_setup.py
```

## Next Steps
1. Customize the frontend interface to match your book's style
2. Add additional skills to the OpenAI Agents
3. Implement user authentication for session management
4. Set up production deployment