#!/usr/bin/env python3
"""
Script to clear all existing embeddings and re-embed documents with the new cleaning functionality.
"""

import sys
import os
import logging

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from backend.src.api.services.tfidf_embedding_service import tfidf_embedding_service
from backend.src.api.services.document_processor import document_processor
from backend.src.api.models.document import DocumentProcessingRequest
from backend.src.api.models.embedding import DocumentChunk
import uuid


def clear_and_reembed():
    """Clear all embeddings and re-embed documents with new cleaning."""

    logger.info("Starting to clear and re-embed documents...")

    # Delete all existing embeddings
    logger.info("Deleting all existing embeddings from Qdrant...")
    try:
        tfidf_embedding_service.delete_all_embeddings()
        logger.info("All existing embeddings deleted successfully.")
    except Exception as e:
        logger.error(f"Error deleting embeddings: {e}")
        return False

    # Process documents from the docs folder with new cleaning
    docs_path = os.path.join(os.path.dirname(__file__), "docs")

    if not os.path.exists(docs_path):
        logger.error(f"Error: docs folder not found at {docs_path}")
        return False

    logger.info(f"Processing documents from: {docs_path}")

    processing_request = DocumentProcessingRequest(
        source_path=docs_path,
        chunk_size=400,  # words per chunk
        overlap=50       # overlap between chunks
    )

    logger.info("Processing documents with new cleaning functionality...")
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

    # Store embeddings in Qdrant using TF-IDF embeddings
    try:
        stored_ids = tfidf_embedding_service.store_embeddings_batch(chunks_to_store)
        logger.info(f"Successfully stored {len(stored_ids)} chunks in Qdrant")
        logger.info("Re-embedding process completed successfully!")
        return True
    except Exception as e:
        logger.error(f"Error storing embeddings: {e}")
        return False


if __name__ == "__main__":
    success = clear_and_reembed()

    if not success:
        logger.error("Clear and re-embed process failed.")
        sys.exit(1)
    else:
        logger.info("Clear and re-embed process completed successfully!")