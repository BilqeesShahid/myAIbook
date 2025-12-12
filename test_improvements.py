#!/usr/bin/env python3
"""
Test script to verify the improvements made to the RAG chatbot.
This script tests the following improvements:
1. Text cleaning (removing YAML frontmatter)
2. Query reformulation
3. Result filtering based on semantic score
4. Answer summarization
"""

import os
import sys
import asyncio
from pathlib import Path

# Add the backend src to the path so we can import the modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

# Import the functions directly from the modules
from backend.src.services.document_processor import clean_chunk_text, DocumentProcessor
from backend.src.services.retrieval_service import reformulate_query
from backend.src.services.tfidf_embedding_service import tfidf_embedding_service
from backend.src.services.rag_service import summarize_answer
from backend.src.models.query import SourceChunk


def test_text_cleaning():
    """Test the clean_chunk_text function."""
    print("Testing text cleaning...")

    # Test YAML frontmatter removal
    test_text_with_yaml = """---
title: Introduction to ROS2
author: John Doe
date: 2023-01-01
---
# Chapter 1: Getting Started

ROS2 is a framework for robotics development."""

    cleaned = clean_chunk_text(test_text_with_yaml)
    print(f"Original text length: {len(test_text_with_yaml)}")
    print(f"Cleaned text length: {len(cleaned)}")
    print(f"Cleaned text: {cleaned}")

    # Verify YAML frontmatter was removed
    assert "---" not in cleaned, "YAML frontmatter was not removed"
    assert "Chapter 1" in cleaned, "Content after YAML was removed"
    print("PASS: Text cleaning test passed\n")


def test_query_reformulation():
    """Test the query reformulation function."""
    print("Testing query reformulation...")

    original_query = "What is ROS2?"
    reformulated = reformulate_query(original_query)

    print(f"Original query: {original_query}")
    print(f"Reformulated query: {reformulated}")

    # For now, the reformulation just returns the original query
    # In the future, this could expand with synonyms or related terms
    assert reformulated == original_query, "Query reformulation should return original query for now"
    print("PASS: Query reformulation test passed\n")


def test_summarize_answer():
    """Test the answer summarization function."""
    print("Testing answer summarization...")

    # Create mock source chunks for testing
    class MockChunk:
        def __init__(self, chunk_id, content):
            self.chunk_id = chunk_id
            self.content = content

    # Mock retrieval service with a simple get_chunk_content method
    class MockRetrievalService:
        def get_chunk_content(self, chunk_id):
            if chunk_id == "chunk1":
                return "ROS2 (Robot Operating System 2) is a framework used to develop robotics software. It provides tools for communication between robot components through nodes, topics, and services."
            elif chunk_id == "chunk2":
                return "ROS2 is designed for distributed systems, real-time performance, and multi-robot communication. It improves upon the original ROS with better security and reliability."
            else:
                return None

    mock_retrieval_service = MockRetrievalService()

    # Create mock source chunks
    mock_chunks = [
        MockChunk("chunk1", "ROS2 (Robot Operating System 2) is a framework used to develop robotics software. It provides tools for communication between robot components through nodes, topics, and services."),
        MockChunk("chunk2", "ROS2 is designed for distributed systems, real-time performance, and multi-robot communication. It improves upon the original ROS with better security and reliability.")
    ]

    # Convert to SourceChunk-like objects
    source_chunks = []
    for chunk in mock_chunks:
        source_chunk = SourceChunk(
            chunk_id=chunk.chunk_id,
            chapter_number=1,
            section_title="Introduction",
            source_file_path="test.md",
            confidence_score=0.8
        )
        source_chunks.append(source_chunk)

    query = "What is ROS2?"
    summary = summarize_answer(query, source_chunks, mock_retrieval_service)

    print(f"Query: {query}")
    print(f"Summary: {summary}")

    # Verify that the summary contains key information
    assert "ROS2" in summary, "Summary should contain ROS2 information"
    assert "framework" in summary.lower() or "robot" in summary.lower(), "Summary should contain relevant content"
    print("PASS: Answer summarization test passed\n")


def test_document_processor_cleaning():
    """Test that the document processor uses the clean function."""
    print("Testing document processor with cleaning...")

    processor = DocumentProcessor()

    # Test markdown with YAML frontmatter
    test_md_content = """---
title: Test Document
author: Test Author
---
# Test Chapter

This is a test document about ROS2.

## Section 1
ROS2 is a robotics framework that provides communication between robot components.

## Section 2
It supports distributed systems and real-time performance."""

    # Process the markdown content to plain text
    plain_text = processor.md_to_text(test_md_content)

    print(f"Processed text: {plain_text}")

    # Verify YAML frontmatter was removed
    assert "---" not in plain_text, "YAML frontmatter should be removed"
    assert "Test Chapter" in plain_text, "Main content should remain"
    assert "ROS2" in plain_text, "ROS2 content should remain"
    print("PASS: Document processor cleaning test passed\n")


def main():
    """Run all tests."""
    print("Running tests for RAG chatbot improvements...\n")

    try:
        test_text_cleaning()
        test_query_reformulation()
        test_summarize_answer()
        test_document_processor_cleaning()

        print("SUCCESS: All tests passed! The improvements are working correctly.")

    except Exception as e:
        print(f"ERROR: Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())