#!/usr/bin/env python3
"""
Test script to verify RAG chatbot API functionality with Qdrant chunks.
"""
import asyncio
import sys
import os

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend', 'src'))

from backend.src.api.services.rag_service import rag_service
from backend.src.api.models.query import QueryRequest

async def test_rag_functionality():
    """
    Test the RAG functionality by querying the system with a sample question.
    """
    print("Testing RAG functionality with Qdrant chunks...")

    # Test query about ROS2 (should match content in the book)
    test_query = "What is ROS2?"

    # Create a query request
    request = QueryRequest(
        query=test_query,
        language="en"
    )

    try:
        # Get response from RAG service
        response_text, source_chunks = await rag_service.get_response(request)

        print(f"\nQuery: {test_query}")
        print(f"Response: {response_text}")
        print(f"Number of source chunks retrieved: {len(source_chunks)}")

        if source_chunks:
            print("\nSource chunks details:")
            for i, chunk in enumerate(source_chunks[:3]):  # Show first 3 chunks
                print(f"  Chunk {i+1}: Chapter {chunk.chapter_number}, Section '{chunk.section_title}'")
                print(f"    Confidence: {chunk.confidence_score}")
                print(f"    Chunk ID: {chunk.chunk_id}")

        print("\nSUCCESS: RAG functionality test completed successfully!")
        return True

    except Exception as e:
        print(f"\nERROR: Error during RAG functionality test: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_selected_text_functionality():
    """
    Test the selected text functionality.
    """
    print("\nTesting selected text functionality...")

    # Test query with selected text
    test_query = "Explain this concept"
    selected_text = "ROS2 (Robot Operating System 2) is a flexible framework for writing robot software."

    # Create a query request with selected text
    request = QueryRequest(
        query=test_query,
        selected_text=selected_text,
        language="en"
    )

    try:
        # Get response from RAG service with selected text
        response_text, source_chunks = await rag_service.get_response_with_selected_text(request)

        print(f"\nQuery: {test_query}")
        print(f"Selected text: {selected_text}")
        print(f"Response: {response_text}")
        print(f"Number of source chunks retrieved: {len(source_chunks)}")

        if source_chunks:
            print("\nSource chunks details:")
            for i, chunk in enumerate(source_chunks[:3]):  # Show first 3 chunks
                print(f"  Chunk {i+1}: Chapter {chunk.chapter_number}, Section '{chunk.section_title}'")
                print(f"    Is selected text: {chunk.is_selected_text}")
                print(f"    Confidence: {chunk.confidence_score}")

        print("\nSUCCESS: Selected text functionality test completed successfully!")
        return True

    except Exception as e:
        print(f"\nERROR: Error during selected text functionality test: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    print("Starting RAG Chatbot API verification tests...\n")

    # Test regular RAG functionality
    rag_success = await test_rag_functionality()

    # Test selected text functionality
    selected_text_success = await test_selected_text_functionality()

    print(f"\n{'='*60}")
    print("SUMMARY:")
    print(f"  Regular RAG functionality: {'PASS' if rag_success else 'FAIL'}")
    print(f"  Selected text functionality: {'PASS' if selected_text_success else 'FAIL'}")

    if rag_success and selected_text_success:
        print("\nAll RAG functionality tests passed!")
        print("The RAG chatbot API is working correctly with Qdrant chunks.")
        return 0
    else:
        print("\nSome tests failed.")
        return 1

if __name__ == "__main__":
    exit_code = asyncio.run(main())
    sys.exit(exit_code)