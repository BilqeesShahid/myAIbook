#!/usr/bin/env python3
"""
Test script to process a single markdown file and create embeddings for the RAG system.
This script reads a single markdown file, chunks it, generates embeddings, and stores them in Qdrant.
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


def process_single_file(file_path: str):
    """
    Process a single markdown file and create embeddings.
    """
    logger.info(f"Starting to process document: {file_path}")

    # Process the single file
    processing_request = DocumentProcessingRequest(
        source_path=file_path,
        chunk_size=400,  # words per chunk
        overlap=50       # overlap between chunks
    )

    logger.info("Processing document...")
    result = document_processor.process_document_request(processing_request)

    if result.status != "success":
        logger.error(f"Error processing document: {result.status}")
        return False

    logger.info(f"Processed {result.total_chunks} chunks from {file_path}")

    if result.total_chunks == 0:
        logger.warning("No chunks were created from the document")
        return True  # Not an error, just no content to process

    # Create DocumentChunk objects and store embeddings
    chunks_to_store = []
    for processed_chunk in result.processed_chunks[:3]:  # Only process first 3 chunks to avoid quota
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
    # Check if docs folder exists and get a small file
    docs_path = os.path.join(os.path.dirname(__file__), "..", "..", "docs")

    # Try to find a small markdown file to test with
    intro_file = os.path.join(docs_path, "intro.md")

    if os.path.exists(intro_file):
        file_to_process = intro_file
        logger.info(f"Found intro.md file to process: {intro_file}")
    else:
        # If intro.md doesn't exist, look for any small markdown file
        md_files = []
        for root, dirs, files in os.walk(docs_path):
            for file in files:
                if file.lower().endswith('.md'):
                    md_files.append(os.path.join(root, file))

        if md_files:
            file_to_process = md_files[0]  # Use the first markdown file found
            logger.info(f"Using first markdown file found: {file_to_process}")
        else:
            logger.error(f"No markdown files found in {docs_path}")
            sys.exit(1)

    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    success = process_single_file(file_to_process)

    if not success:
        logger.error("Document embedding process failed.")
        sys.exit(1)
    else:
        logger.info("Document embedding process completed successfully!")