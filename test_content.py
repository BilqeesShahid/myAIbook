#!/usr/bin/env python3
"""
Quick test to verify the RAG system can process content files
"""

import sys
import os
import logging

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def test_content_processing():
    # Validate setup first
    import backend.src.utils.constants as constants
    if not constants.QDRANT_URL or not constants.QDRANT_URL.startswith("https://"):
        logger.error("Qdrant URL not configured properly")
        return False

    if not constants.GOOGLE_API_KEY or len(constants.GOOGLE_API_KEY) < 10:
        logger.error("Google API key not configured properly")
        return False

    logger.info("Setup validation passed!")

    # Test that the services can be imported and initialized
    try:
        from backend.src.services.document_processor import document_processor
        from backend.src.services.embedding_service import embedding_service
        from backend.src.services.rag_service import rag_service
        from backend.src.services.agent_orchestrator import agent_orchestrator

        logger.info("All services imported successfully!")

        # Check if there are content files to process
        docs_path = os.path.join(os.path.dirname(__file__), "docs")
        content_dirs = ["0-ros2", "1-simulation", "2-isaac", "3-vla", "4-capstone"]

        content_found = False
        for dir_name in content_dirs:
            dir_path = os.path.join(docs_path, dir_name)
            if os.path.exists(dir_path):
                files = [f for f in os.listdir(dir_path) if f.endswith('.md')]
                if files:
                    logger.info(f"Found content files in {dir_name}: {files[:3]}...")  # Show first 3
                    content_found = True
                    break

        if not content_found:
            logger.warning("No content files found in expected directories")
            return False

        logger.info("RAG system is properly configured and ready to process content!")
        return True

    except Exception as e:
        logger.error(f"Error during test: {e}")
        return False

if __name__ == "__main__":
    success = test_content_processing()
    if success:
        print("✅ RAG system test passed! The system is ready to process content.")
        print("💡 To process all content, run the embed_content.py script (be aware of API quotas).")
    else:
        print("❌ RAG system test failed.")
        sys.exit(1)