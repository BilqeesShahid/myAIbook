#!/usr/bin/env python3
"""
Script to process a few small chunks and create embeddings for the RAG system.
This will help avoid API quota issues while verifying the system works.
"""

import sys
import os
import uuid
from pathlib import Path
import logging

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from backend.src.services.document_processor import document_processor
from backend.src.models.document import DocumentProcessingRequest
from backend.src.services.embedding_service import embedding_service
from backend.src.models.embedding import DocumentChunk


def process_few_chunks(docs_path: str = "docs"):
    """
    Process only a few chunks to avoid API quota issues.
    """
    logger.info(f"Starting to process a few chunks from: {docs_path}")

    # First, let's find a small file to process
    import os
    content_dirs = ["0-ros2", "1-simulation", "2-isaac", "3-vla", "4-capstone"]

    content_file = None
    for dir_name in content_dirs:
        dir_path = os.path.join(docs_path, dir_name)
        if os.path.exists(dir_path):
            files = [f for f in os.listdir(dir_path) if f.endswith('.md')]
            if files:
                content_file = os.path.join(dir_path, files[0])  # Use first file
                logger.info(f"Found content file: {content_file}")
                break

    if not content_file:
        logger.error(f"No markdown content files found in {docs_path}")
        return False

    # Process just this single file with small chunk size to create multiple small chunks
    processing_request = DocumentProcessingRequest(
        source_path=content_file,
        chunk_size=100,  # Smaller chunks to create more of them
        overlap=20       # Smaller overlap
    )

    logger.info("Processing document...")
    result = document_processor.process_document_request(processing_request)

    if result.status != "success":
        logger.error(f"Error processing document: {result.status}")
        return False

    logger.info(f"Processed {result.total_chunks} chunks from {content_file}")

    if result.total_chunks == 0:
        logger.warning("No chunks were created from the document")
        return True  # Not an error, just no content to process

    # Take only the first 3 chunks to avoid quota issues
    chunks_to_process = result.processed_chunks[:3]
    logger.info(f"Processing first {len(chunks_to_process)} chunks to avoid API quota limits...")

    # Create DocumentChunk objects and store embeddings
    chunks_to_store = []
    for processed_chunk in chunks_to_process:
        chunk = DocumentChunk(
            id=str(uuid.uuid4()),  # Generate a unique ID for each chunk
            content=processed_chunk.content[:500],  # Limit content size to first 500 chars to ensure variety
            chapter_number=processed_chunk.metadata.get("chapter_number", 0),
            section_title=processed_chunk.metadata.get("section_title", "Unknown"),
            source_file_path=processed_chunk.metadata.get("source_file_path", "Unknown"),
            embedding_vector=[],  # This will be populated by the embedding service
            chunk_index=processed_chunk.chunk_index,
            metadata=processed_chunk.metadata
        )
        chunks_to_store.append(chunk)

    logger.info(f"Storing {len(chunks_to_store)} chunks in Qdrant...")

    # Store embeddings in Qdrant
    try:
        stored_ids = embedding_service.store_embeddings_batch(chunks_to_store)
        logger.info(f"Successfully stored {len(stored_ids)} chunks in Qdrant")
        logger.info("Embedding process completed successfully!")
        return True
    except Exception as e:
        logger.error(f"Error storing embeddings: {e}")
        return False


def validate_setup():
    """
    Validate that the setup is properly configured.
    """
    logger.info("Validating setup...")

    # Check if required environment variables are set
    import backend.src.utils.constants as constants
    if not constants.QDRANT_URL or not constants.QDRANT_URL.startswith("https://"):
        logger.warning("Qdrant URL not configured properly")
        return False

    if not constants.GOOGLE_API_KEY or len(constants.GOOGLE_API_KEY) < 10:
        logger.warning("Google API key not configured properly")
        return False

    logger.info("Setup validation passed!")
    return True


if __name__ == "__main__":
    # Check if docs folder exists
    docs_path = os.path.join(os.path.dirname(__file__), "..", "..", "docs")

    if not os.path.exists(docs_path):
        logger.error(f"Error: docs folder not found at {docs_path}")
        sys.exit(1)

    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    success = process_few_chunks(docs_path)

    if not success:
        logger.error("Document embedding process failed.")
        sys.exit(1)
    else:
        logger.info("Document embedding process completed successfully!")
        logger.info("✅ Check your Qdrant dashboard - you should now see chunks stored in the book_embeddings collection!")