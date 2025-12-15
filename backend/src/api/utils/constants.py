import os

# Configuration constants for the RAG system

# Qdrant Configuration
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_embeddings")

# Google Generative AI Configuration
GOOGLE_API_KEY = os.getenv("GOOGLE_API_KEY")

# Local Embedding Configuration - Use 768 to match existing collection
EMBEDDING_DIMENSION = int(os.getenv("EMBEDDING_DIMENSION", "768"))

# Document Processing Configuration
DEFAULT_CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "400"))  # words per chunk
DEFAULT_OVERLAP = 50  # words overlap between chunks
MAX_CHUNK_SIZE = 1000  # maximum words per chunk
MIN_CHUNK_SIZE = 100  # minimum words per chunk

# API Configuration
DEFAULT_CONTEXT_WINDOW = 5  # number of previous messages to include in context
MAX_CONTEXT_LENGTH = int(os.getenv("MAX_CONTEXT_LENGTH", "1000"))  # maximum context length in tokens
MAX_QUERY_LENGTH = 1000  # maximum query length in characters

# Performance Configuration
MAX_RESPONSE_TIME = 0.5  # maximum response time in seconds (500ms)
DEFAULT_TIMEOUT = 30  # default timeout in seconds

# Language Configuration
SUPPORTED_LANGUAGES = ["en", "ur"]  # English and Urdu
DEFAULT_LANGUAGE = "en"

# Collection Configuration
VECTOR_COLLECTION_NAME = QDRANT_COLLECTION_NAME
FASTAPI_ENV = os.getenv("FASTAPI_ENV", "development")