#!/usr/bin/env python3
"""
Script to process the docs folder and create embeddings for the RAG system.
This script reads all markdown files from the docs folder, chunks them,
generates local embeddings, and stores them in Qdrant.
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
from backend.src.services.local_embedding_service import local_embedding_service
from backend.src.models.embedding import DocumentChunk


def process_docs_folder(docs_path: str = "docs"):
    """
    Process all markdown files in the docs folder and create local embeddings.
    """
    logger.info(f"Starting to process documents from: {docs_path}")

    # Process documents from the docs folder
    processing_request = DocumentProcessingRequest(
        source_path=docs_path,
        chunk_size=400,  # words per chunk
        overlap=50       # overlap between chunks
    )

    logger.info("Processing documents...")
    result = document_processor.process_document_request(processing_request)

    if result.status != "success":
        logger.error(f"Error processing documents: {result.status}")
        return False

    logger.info(f"Processed {result.total_chunks} chunks from {docs_path}")

    # Create DocumentChunk objects and store embeddings
    chunks_to_store = []
    for processed_chunk in result.processed_chunks:
        chunk = DocumentChunk(
            id=str(uuid.uuid4()),  # Generate a unique ID for each chunk
            content=processed_chunk.content,
            chapter_number=processed_chunk.metadata.get("chapter_number", 0),
            section_title=processed_chunk.metadata.get("section_title", "Unknown"),
            source_file_path=processed_chunk.metadata.get("source_file_path", "Unknown"),
            embedding_vector=[],  # This will be populated by the embedding service
            chunk_index=processed_chunk.chunk_index,
            metadata=processed_chunk.metadata
        )
        chunks_to_store.append(chunk)

    logger.info(f"Storing {len(chunks_to_store)} chunks in Qdrant...")

    # Store embeddings in Qdrant using local embeddings
    try:
        stored_ids = local_embedding_service.store_embeddings_batch(chunks_to_store)
        logger.info(f"Successfully stored {len(stored_ids)} chunks in Qdrant")
        logger.info("Local embedding process completed successfully!")
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

    success = process_docs_folder(docs_path)

    if not success:
        logger.error("Document local embedding process failed.")
        sys.exit(1)
    else:
        logger.info("Document local embedding process completed successfully!")