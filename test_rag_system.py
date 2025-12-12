#!/usr/bin/env python3
"""
Test script to verify the RAG system is working with local embeddings.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.src.services.retrieval_service import retrieval_service
from backend.src.models.query import QueryRequest

def test_retrieval():
    """
    Test the retrieval functionality with a sample query.
    """
    print("Testing RAG system with local embeddings...")

    # Create a sample query
    sample_query = "What is ROS2?"

    try:
        # Test basic retrieval
        print(f"Query: {sample_query}")
        source_chunks = retrieval_service.retrieve_chunks(sample_query, top_k=3)

        print(f"Found {len(source_chunks)} relevant chunks:")
        for i, chunk in enumerate(source_chunks):
            print(f"  {i+1}. Chapter {chunk.chapter_number}, Section '{chunk.section_title}'")
            print(f"     Confidence: {chunk.confidence_score:.2f}")
            print(f"     Path: {chunk.source_file_path}")
            print()

        if len(source_chunks) == 0:
            print("No chunks found - this might indicate an issue with the stored data")
        else:
            print("SUCCESS: Retrieval test completed successfully!")

        return len(source_chunks) > 0

    except Exception as e:
        print(f"ERROR: Error during retrieval test: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_query_with_content():
    """
    Test with a more specific query that should match content in the docs.
    """
    print("\nTesting with more specific query...")

    # Try a query that might match content in the robotics book
    sample_query = "robot operating system"

    try:
        print(f"Query: {sample_query}")
        source_chunks = retrieval_service.retrieve_chunks(sample_query, top_k=3)

        print(f"Found {len(source_chunks)} relevant chunks:")
        for i, chunk in enumerate(source_chunks):
            print(f"  {i+1}. Chapter {chunk.chapter_number}, Section '{chunk.section_title}'")
            content_preview = chunk.source_file_path  # Using file path as preview
            print(f"     Preview: {content_preview}")
            print()

        return len(source_chunks) > 0

    except Exception as e:
        print(f"ERROR: Error during specific query test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Starting RAG system verification tests...\n")

    success1 = test_retrieval()
    success2 = test_query_with_content()

    if success1 or success2:
        print("\nSUCCESS: At least one test was successful - RAG system appears to be working!")
        print("  Content has been stored in Qdrant and can be retrieved.")
    else:
        print("\nERROR: Tests failed - there may be an issue with the RAG system.")

    print("\nVerification complete.")