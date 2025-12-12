#!/usr/bin/env python3
"""
Script to create and store a single test chunk to verify the RAG system works.
This will help confirm the system is properly set up without hitting API quotas.
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

from backend.src.services.embedding_service import embedding_service
from backend.src.models.embedding import DocumentChunk


def create_test_chunk():
    """
    Create and store a single test chunk to verify the system works.
    """
    logger.info("Creating a test chunk to verify the RAG system...")

    # Create a test chunk with sample content
    test_chunk = DocumentChunk(
        id=str(uuid.uuid4()),
        content="This is a test chunk to verify that the RAG system is working properly. The Robotics and AI educational system covers fundamental concepts in robotics, artificial intelligence, ROS2 framework, simulation environments, and practical applications.",
        chapter_number=1,
        section_title="Introduction to Robotics",
        source_file_path="test-content.md",
        embedding_vector=[],  # This will be populated by the embedding service
        chunk_index=0,
        metadata={
            "source_file_path": "test-content.md",
            "chapter_number": 1,
            "section_title": "Introduction to Robotics",
            "chunk_index": 0
        }
    )

    logger.info("Storing test chunk in Qdrant...")

    try:
        # Store the single chunk in Qdrant
        stored_id = embedding_service.store_embedding(test_chunk)
        logger.info(f"Successfully stored test chunk with ID: {stored_id}")
        logger.info("✅ RAG system verification successful!")
        logger.info("💡 The system is properly configured. To process all your content, you need to upgrade your Google API plan.")
        return True
    except Exception as e:
        logger.error(f"Error storing test chunk: {e}")
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
    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    success = create_test_chunk()

    if not success:
        logger.error("Test chunk creation failed.")
        sys.exit(1)
    else:
        logger.info("Test chunk verification completed successfully!")
        logger.info("\nTo process all your content:")
        logger.info("1. Upgrade your Google AI Studio plan to get higher quotas")
        logger.info("2. Or wait until your quota resets (usually daily)")
        logger.info("3. Then run: python scripts/setup/embed_complete_content.py")