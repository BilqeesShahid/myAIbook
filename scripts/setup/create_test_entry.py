#!/usr/bin/env python3
"""
Script to create a single test entry in Qdrant to confirm the system is working.
This bypasses the Google API entirely by using a random vector.
"""

import sys
import os
import uuid
from pathlib import Path
import logging
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Import constants
from backend.src.utils.constants import QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME, EMBEDDING_DIMENSION


def create_test_entry():
    """
    Create a single test entry in Qdrant with a random vector to confirm the system works.
    """
    logger.info("Creating a test entry in Qdrant to verify the system...")

    try:
        # Initialize Qdrant client
        qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False
        )

        # Generate a random embedding vector (768 dimensions as per your setup)
        import random
        random_vector = [random.uniform(-1, 1) for _ in range(EMBEDDING_DIMENSION)]

        # Create a test document chunk
        chunk_id = str(uuid.uuid4())
        test_content = "This is a test entry to verify that the Qdrant integration is working properly for the Robotics and AI educational system. This covers fundamental concepts in robotics, artificial intelligence, ROS2 framework, simulation environments, and practical applications."

        # Prepare the point for Qdrant
        point = PointStruct(
            id=chunk_id,
            vector=random_vector,
            payload={
                "content": test_content,
                "chapter_number": 1,
                "section_title": "Test Entry - Introduction",
                "source_file_path": "test-content.md",
                "chunk_index": 0,
                "metadata": {
                    "source_file_path": "test-content.md",
                    "chapter_number": 1,
                    "section_title": "Test Entry - Introduction",
                    "chunk_index": 0
                }
            }
        )

        # Store in Qdrant
        qdrant_client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            points=[point]
        )

        logger.info(f"✅ Successfully created test entry with ID: {chunk_id}")
        logger.info("💡 The Qdrant integration is working correctly!")
        logger.info("💡 The system is properly configured. When you get Google API access,")
        logger.info("   you can run the complete embedding scripts to process real content.")

        # Get collection info to show current status
        collection_info = qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
        logger.info(f"📊 Collection '{QDRANT_COLLECTION_NAME}' now has {collection_info.points_count} vectors")

        return True

    except Exception as e:
        logger.error(f"Error creating test entry: {e}")
        return False


def validate_setup():
    """
    Validate that the setup is properly configured.
    """
    logger.info("Validating setup...")

    # Check if required environment variables are set
    if not QDRANT_URL or not QDRANT_URL.startswith("https://"):
        logger.error("Qdrant URL not configured properly")
        return False

    if not QDRANT_API_KEY or len(QDRANT_API_KEY) < 10:
        logger.error("Qdrant API key not configured properly")
        return False

    logger.info("Setup validation passed!")
    return True


if __name__ == "__main__":
    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    success = create_test_entry()

    if not success:
        logger.error("Test entry creation failed.")
        sys.exit(1)
    else:
        logger.info("✅ Test entry verification completed successfully!")
        logger.info("\nNext steps:")
        logger.info("1. Upgrade your Google AI Studio plan to get embedding API access")
        logger.info("2. Then run: python scripts/setup/embed_complete_content.py")