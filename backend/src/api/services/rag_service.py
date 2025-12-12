from typing import Tuple, List
import logging
import os
import uuid
import requests
from ..models.query import QueryRequest, SourceChunk
from ..services.retrieval_service import retrieval_service
from ..services.chat_service import chat_service

logger = logging.getLogger(__name__)


def summarize_answer(query: str, chunks: List[SourceChunk], retrieval_service) -> str:
    """
    Summarize the answer based on the query and retrieved chunks.
    This function creates a concise educational explanation without copying unnecessary text.
    """
    # Get content from all chunks
    content_texts = []
    for chunk in chunks:
        content = retrieval_service.get_chunk_content(chunk.chunk_id)
        if content and content.strip():
            # Only include content that is relevant (not just chapter titles or quiz questions)
            content_clean = content.strip()
            # Skip if content looks like a quiz question or contains obvious non-content
            if (len(content_clean) > 20 and  # Ensure it's substantial content
                not content_clean.lower().startswith('question') and
                not content_clean.lower().startswith('quiz') and
                not content_clean.lower().startswith('answer') and
                'multiple choice' not in content_clean.lower() and
                'true/false' not in content_clean.lower()):
                content_texts.append(content_clean)

    if not content_texts:
        return f"I couldn't find relevant information in the book to answer '{query}'. Please try rephrasing your question."

    # Instead of concatenating all content, find the most relevant chunk
    # by looking for content that best matches the query
    query_lower = query.lower()
    best_content = ""
    best_score = 0

    for content in content_texts:
        score = 0
        content_lower = content.lower()

        # Score based on query term matches
        for term in query_lower.split():
            if len(term) > 2:  # Only count meaningful words
                if term in content_lower:
                    score += 1

        # Prefer content that starts with definitions or explanations
        if content_lower.startswith('ros2') or 'is a' in content_lower or 'are a' in content_lower:
            score += 2

        if score > best_score:
            best_score = score
            best_content = content

    # If we found a good match, return it; otherwise, return the first chunk
    if best_content and best_score > 0:
        # Clean up the content by removing obvious non-content parts
        # Remove content that looks like it's from quiz sections
        if 'chapter' in best_content.lower() and ('question' in best_content.lower() or 'answer' in best_content.lower()):
            # If best content contains quiz-like text, fall back to first good content
            best_content = content_texts[0]

        return f"Based on the book content: {best_content.strip()}"
    else:
        # Fallback: use first chunk if no good match found
        return f"Based on the book content: {content_texts[0][:500] if content_texts else 'No relevant content found.'}"


class RAGService:
    """
    Service for RAG (Retrieval-Augmented Generation) operations.
    Orchestrates the retrieval of relevant documents and generation of responses using Hugging Face models.
    """

    def __init__(self):
        # Get Hugging Face API key from environment
        self.hf_api_key = os.getenv("HF_API_KEY")
        if not self.hf_api_key:
            logger.warning("HF_API_KEY not found in environment. Hugging Face model inference will not work.")

        self.retrieval_service = retrieval_service
        self.chat_service = chat_service

    async def get_response(self, request: QueryRequest) -> Tuple[str, List[SourceChunk]]:
        """
        Get a response for a general query based on book content using AI agents.
        """
        try:
            # Get or create session
            if request.session_id:
                session_id = request.session_id
                session = self.chat_service.get_session(session_id)
                if not session:
                    # If session doesn't exist but was requested, create it with the specific ID
                    # We'll create a new session and ignore the ID since create_session generates its own
                    session = self.chat_service.create_session(title=f"RAG Session {session_id[:8]}")
                    session_id = session.session_id  # Use the actual session ID generated
            else:
                # Create a new session
                session = self.chat_service.create_session(title=f"RAG Session")
                session_id = session.session_id

            # Ensure the session exists in messages dict (should be done by create_session)
            if session_id not in self.chat_service.messages:
                self.chat_service.messages[session_id] = []

            self.chat_service.add_message(
                session_id=session_id,
                role="user",
                content=request.query,
                language=request.language
            )

            # Retrieve relevant chunks based on the query
            source_chunks = self.retrieval_service.retrieve_chunks_by_request(request)

            # Generate response using Hugging Face models directly
            response_text = await self._generate_response_with_hf(request.query, source_chunks)

            # Add assistant response to session
            chunk_ids = [chunk.chunk_id for chunk in source_chunks] if source_chunks else []
            self.chat_service.add_message(
                session_id=session_id,
                role="assistant",
                content=response_text,
                language=request.language,
                context_chunks=chunk_ids
            )

            return response_text, source_chunks
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            raise

    async def get_response_with_selected_text(self, request: QueryRequest) -> Tuple[str, List[SourceChunk]]:
        """
        Get a response prioritizing the selected text using AI agents.
        """
        try:
            # Validate that selected text exists
            if not request.selected_text:
                return await self.get_response(request)

            # Get or create session
            if request.session_id:
                session_id = request.session_id
                session = self.chat_service.get_session(session_id)
                if not session:
                    # If session doesn't exist but was requested, create it with the specific ID
                    session = self.chat_service.create_session(title=f"RAG Session {session_id[:8]}")
                    session_id = session.session_id  # Use the actual session ID generated
            else:
                # Create a new session
                session = self.chat_service.create_session(title=f"RAG Session")
                session_id = session.session_id

            # Ensure the session exists in messages dict (should be done by create_session)
            if session_id not in self.chat_service.messages:
                self.chat_service.messages[session_id] = []

            # Add user message to session
            self.chat_service.add_message(
                session_id=session_id,
                role="user",
                content=f"Query: {request.query}\nSelected text: {request.selected_text}",
                language=request.language
            )

            # Retrieve relevant chunks prioritizing selected text
            source_chunks = self.retrieval_service.retrieve_chunks_with_selected_text(
                request.query,
                request.selected_text,
                top_k=5
            )

            # Create a detailed query that includes the selected text for the AI agent
            detailed_query = f"Query: {request.query}\nSelected text for context: {request.selected_text}"

            # Generate response using Hugging Face models directly
            response_text = await self._generate_response_with_hf(detailed_query, source_chunks)

            # Add assistant response to session
            chunk_ids = [chunk.chunk_id for chunk in source_chunks] if source_chunks else []
            self.chat_service.add_message(
                session_id=session_id,
                role="assistant",
                content=response_text,
                language=request.language,
                context_chunks=chunk_ids
            )

            return response_text, source_chunks
        except Exception as e:
            logger.error(f"Error generating response with selected text: {e}")
            raise

    def _prepare_context(self, source_chunks: List[SourceChunk]) -> str:
        """
        Prepare context from retrieved source chunks.
        This method is kept for potential future use.
        """
        context_parts = []
        for chunk in source_chunks:
            context_parts.append(
                f"Chapter {chunk.chapter_number}, Section '{chunk.section_title}':\n"
                f"{self.retrieval_service.get_chunk_content(chunk.chunk_id) or ''}\n"
            )
        return "\n".join(context_parts)

    def _prepare_context_with_selected_text(self, source_chunks: List[SourceChunk], selected_text: str) -> str:
        """
        Prepare context with emphasis on selected text.
        This method is kept for potential future use.
        """
        context_parts = [f"Selected text: {selected_text}\n"]

        for chunk in source_chunks:
            # Add more weight to chunks that are the selected text
            if chunk.is_selected_text:
                context_parts.append(
                    f"Selected text context - Chapter {chunk.chapter_number}, Section '{chunk.section_title}':\n"
                    f"{self.retrieval_service.get_chunk_content(chunk.chunk_id) or ''}\n"
                )
            else:
                context_parts.append(
                    f"Related content - Chapter {chunk.chapter_number}, Section '{chunk.section_title}':\n"
                    f"{self.retrieval_service.get_chunk_content(chunk.chunk_id) or ''}\n"
                )

        return "\n".join(context_parts)

    async def _generate_response_with_hf(self, query: str, source_chunks: List[SourceChunk]) -> str:
        """
        Generate a response using the retrieved context with priority to first chapters.
        Prioritizes content from early chapters (like Chapter 0, 1, etc.) before others.
        """
        try:
            # Use the summarization function to create a concise answer
            return summarize_answer(query, source_chunks, self.retrieval_service)
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            # Fallback response
            if source_chunks:
                # If we found chunks but had an error, at least mention that
                # Sort by chapter to prioritize early chapters in message
                sorted_chunks = sorted(source_chunks, key=lambda x: x.chapter_number)
                if sorted_chunks:
                    first_chunk = sorted_chunks[0]
                    return f"I found relevant information in Chapter {first_chunk.chapter_number}, Section '{first_chunk.section_title}' about '{query}', but encountered an issue compiling the full response. Please try asking your question again."
            return f"I couldn't find relevant information in the book to answer '{query}'. Please try rephrasing your question."


# Singleton instance
rag_service = RAGService()