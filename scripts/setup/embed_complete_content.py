#!/usr/bin/env python3
"""
Script to process ALL content from the docs folder without creating embeddings.
This script processes content but does not perform embedding generation or storage.
"""

import sys
import os
import time
from pathlib import Path
import logging
from typing import List

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

from backend.src.services.document_processor import document_processor
from backend.src.models.document import DocumentProcessingRequest


def process_content_without_embeddings(docs_path: str = "docs"):
    """
    Process all content from docs folder without creating embeddings.
    """
    logger.info(f"Starting to process ALL content from: {docs_path}")

    # Get all markdown files from all subdirectories
    all_md_files = []
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.lower().endswith('.md'):
                all_md_files.append(os.path.join(root, file))

    logger.info(f"Found {len(all_md_files)} markdown files to process")

    if not all_md_files:
        logger.error(f"No markdown files found in {docs_path}")
        return False

    total_chunks_processed = 0

    for i, file_path in enumerate(all_md_files):
        logger.info(f"Processing file {i+1}/{len(all_md_files)}: {file_path}")

        # Process this single file
        processing_request = DocumentProcessingRequest(
            source_path=file_path,
            chunk_size=200,  # Moderate chunk size (though chunking is disabled)
            overlap=30       # Moderate overlap (though chunking is disabled)
        )

        try:
            logger.info(f"Processing document: {os.path.basename(file_path)}")
            result = document_processor.process_document_request(processing_request)

            if result.status != "success":
                logger.error(f"Error processing document {file_path}: {result.status}")
                continue

            logger.info(f"Processed {result.total_chunks} document units from {file_path}")
            total_chunks_processed += result.total_chunks

            if result.total_chunks == 0:
                logger.warning(f"No content was extracted from {file_path}")
                continue

        except Exception as e:
            logger.error(f"Unexpected error processing {file_path}: {e}")
            continue

    logger.info(f"Processing completed! Total document units processed: {total_chunks_processed}")
    return True


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

    if not os.path.exists(docs_path):
        logger.error(f"Error: docs folder not found at {docs_path}")
        sys.exit(1)

    # Validate setup first
    if not validate_setup():
        logger.error("Setup validation failed.")
        sys.exit(1)

    logger.info("Starting complete content processing (without embeddings)...")

    success = process_content_without_embeddings(docs_path=docs_path)

    if not success:
        logger.error("Document processing failed.")
        sys.exit(1)
    else:
        logger.info("✅ Document processing completed successfully!")
        logger.info("💡 All content from your book modules has been processed (without embeddings).")