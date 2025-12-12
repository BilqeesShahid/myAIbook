#!/usr/bin/env python3
"""
Script to process Urdu docs folder and create embeddings for the RAG system.
This script reads all markdown files from the i18n folder, chunks them,
generates embeddings, and stores them in Qdrant.
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


def process_urdu_docs_folder(docs_path: str, language: str = "ur"):
    """
    Process all markdown files in the Urdu folder and create embeddings.
    """
    logger.info(f"Starting to process {language} documents from: {docs_path}")

    # Process documents from the folder
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
        # Add language information to metadata
        chunk.metadata["language"] = language
        chunks_to_store.append(chunk)

    logger.info(f"Storing {len(chunks_to_store)} chunks in Qdrant...")

    # Store embeddings in Qdrant
    try:
        stored_ids = embedding_service.store_embeddings_batch(chunks_to_store)
        logger.info(f"Successfully stored {len(stored_ids)} chunks in Qdrant")
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
    # Check if Urdu i18n folder exists
    i18n_path = os.path.join(os.path.dirname(__file__), "..", "..", "i18n", "ur-UR", "docusaurus-plugin-content-docs", "current")

    if not os.path.exists(i18n_path):
        logger.error(f"Error: Urdu i18n folder not found at {i18n_path}")
        sys.exit(1)

    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    # Process Urdu documents
    logger.info("Processing Urdu documents...")
    urdu_success = process_urdu_docs_folder(i18n_path, "ur")

    if not urdu_success:
        logger.error("Urdu document embedding process failed.")
        sys.exit(1)
    else:
        logger.info("Urdu document embedding process completed successfully!")