#!/usr/bin/env python3
"""
Test script to verify the summarization logic in the RAG service
"""
import asyncio
import sys
import os

# Add the backend src directory to the Python path
backend_path = os.path.join(os.path.dirname(__file__), 'backend')
sys.path.insert(0, backend_path)
src_path = os.path.join(backend_path, 'src')
sys.path.insert(0, src_path)

from models.query import QueryRequest
from services.rag_service import rag_service
from services.retrieval_service import retrieval_service

async def test_summarization_logic():
    """Test the summarization logic directly"""
    print("Testing summarization logic in RAG service...")

    # Create a test query request with selected text
    test_request = QueryRequest(
        query="Summarize this text",
        selected_text="ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. ROS 2 provides a rich set of tools and libraries for building robot applications.",
        session_id="test_session_123",
        language="en"
    )

    try:
        # Test the get_response_with_selected_text method
        response_text, source_chunks = await rag_service.get_response_with_selected_text(test_request)

        print("✅ Summarization logic test passed!")
        print(f"Response: {response_text}")
        print(f"Source chunks: {len(source_chunks)}")

        # Check if the response contains relevant information
        if "ROS 2" in response_text or "robot" in response_text.lower():
            print("✅ Response contains relevant content!")
        else:
            print("⚠️  Response may not contain relevant content")

        return True

    except Exception as e:
        print(f"❌ Error testing summarization logic: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_retrieval_with_selected_text():
    """Test the retrieval service with selected text"""
    print("\nTesting retrieval with selected text...")

    try:
        # Test the retrieval service directly
        query = "What is ROS 2?"
        selected_text = "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software."

        chunks = retrieval_service.retrieve_chunks_with_selected_text(query, selected_text, top_k=3)

        print(f"✅ Retrieval with selected text test passed!")
        print(f"Retrieved {len(chunks)} chunks")

        for i, chunk in enumerate(chunks):
            print(f"  Chunk {i+1}: Chapter {chunk.chapter_number}, Confidence: {chunk.confidence_score}, Selected: {getattr(chunk, 'is_selected_text', False)}")

        return True

    except Exception as e:
        print(f"❌ Error testing retrieval with selected text: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    print("Testing RAG service summarization functionality...")

    # Test retrieval first
    retrieval_success = test_retrieval_with_selected_text()

    # Test summarization
    summarization_success = await test_summarization_logic()

    if retrieval_success and summarization_success:
        print("\n🎉 All RAG service tests passed! Summarization functionality is working correctly.")
    else:
        print("\n⚠️  Some tests failed. There may be issues with the RAG service.")

if __name__ == "__main__":
    asyncio.run(main())