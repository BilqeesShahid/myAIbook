#!/usr/bin/env python3
"""
Script to process ONE complete module (e.g., 0-ros2) without creating embeddings.
This script processes content but does not perform embedding generation or storage.
"""

import sys
import os
import time
from pathlib import Path
import logging

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

from backend.src.services.document_processor import document_processor
from backend.src.models.document import DocumentProcessingRequest


def process_one_module(module_dir: str = "0-ros2", docs_path: str = "docs"):
    """
    Process ONE complete module without creating embeddings.
    """
    logger.info(f"Starting to process module: {module_dir}")

    module_path = os.path.join(docs_path, module_dir)

    if not os.path.exists(module_path):
        logger.error(f"Module directory does not exist: {module_path}")
        return False

    # Get all markdown files in this module
    module_files = []
    for file in os.listdir(module_path):
        if file.lower().endswith('.md'):
            module_files.append(os.path.join(module_path, file))

    if not module_files:
        logger.error(f"No markdown files found in {module_path}")
        return False

    logger.info(f"Found {len(module_files)} files in module {module_dir}")

    total_processed = 0

    # Process files one by one without API delays since no embeddings are created
    for i, file_path in enumerate(module_files):
        logger.info(f"Processing file {i+1}/{len(module_files)}: {os.path.basename(file_path)}")

        # Process this single file
        processing_request = DocumentProcessingRequest(
            source_path=file_path,
            chunk_size=50,  # Chunk size (though chunking is disabled)
            overlap=10      # Overlap (though chunking is disabled)
        )

        try:
            logger.info(f"Processing document: {os.path.basename(file_path)}")
            result = document_processor.process_document_request(processing_request)

            if result.status != "success":
                logger.error(f"Error processing document {file_path}: {result.status}")
                continue

            logger.info(f"Processed {result.total_chunks} document units from {os.path.basename(file_path)}")

            if result.total_chunks == 0:
                logger.warning(f"No content was extracted from {file_path}")
                continue

            total_processed += result.total_chunks

        except Exception as e:
            logger.error(f"Unexpected error processing {file_path}: {e}")
            continue

    logger.info(f"Module processing completed! Total processed: {total_processed}")
    return total_processed > 0  # Return True if at least one document was processed


def validate_setup():
    """
    Validate that the setup is properly configured.
    """
    logger.info("Validating setup...")

    # No external dependencies to validate now
    logger.info("Setup validation passed!")
    return True


if __name__ == "__main__":
    # Check if docs folder exists
    docs_path = os.path.join(os.path.dirname(__file__), "..", "..", "docs")
    module_to_process = "0-ros2"  # Process the first module

    if not os.path.exists(docs_path):
        logger.error(f"Error: docs folder not found at {docs_path}")
        sys.exit(1)

    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    logger.info(f"Starting to process module {module_to_process} (without embeddings)...")

    success = process_one_module(module_dir=module_to_process, docs_path=docs_path)

    if not success:
        logger.error("Module processing failed or no content was extracted.")
        sys.exit(1)
    else:
        logger.info(f"✅ Module {module_to_process} processing completed successfully!")
        logger.info("💡 Content has been processed (without embeddings).")