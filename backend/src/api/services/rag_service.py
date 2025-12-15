from typing import Tuple, List
import logging
import os
import uuid
import requests
import re
from qdrant_client import QdrantClient
from qdrant_client.http import models
from ..models.query import QueryRequest, SourceChunk
from ..services.retrieval_service import retrieval_service
from ..services.chat_service import chat_service
from ..utils.constants import QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION_NAME

logger = logging.getLogger(__name__)


def find_direct_answer(query: str) -> str:
    """
    Direct approach to find answers for various query types:
    - "What is X?" -> finds formal definitions
    - "What are X?" -> finds plural definitions
    - "Who is X?" -> finds person descriptions
    - "Why X?" -> finds explanation content
    - "How X?" -> finds procedural content
    """
    # Connect to Qdrant
    client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY
    )

    query_lower = query.lower().strip()

    # Parse query type and subject
    subject = None
    query_type = None

    if query_lower.startswith('what is '):
        subject = query_lower[8:].replace('?', '').strip()
        query_type = 'definition'
    elif query_lower.startswith('what are '):
        subject = query_lower[9:].replace('?', '').strip()
        query_type = 'plural_definition'
    elif query_lower.startswith('who is '):
        subject = query_lower[7:].replace('?', '').strip()
        query_type = 'person'
    elif query_lower.startswith('why '):
        subject = query_lower[4:].replace('?', '').strip()
        query_type = 'explanation'
    elif query_lower.startswith('how '):
        subject = query_lower[4:].replace('?', '').strip()
        query_type = 'procedure'
    else:
        return None  # Not a supported query type for direct search

    # Scroll through all records
    all_results = client.scroll(
        collection_name=QDRANT_COLLECTION_NAME,
        limit=1000,
        with_payload=True,
        with_vectors=False
    )

    records, _ = all_results if isinstance(all_results, tuple) else (all_results, None)

    if query_type == 'definition':
        # For "What is X?" - same as before, prioritize formal definitions
        return _find_definition_for_subject(subject, records)

    elif query_type == 'plural_definition':
        # For "What are X?" - look for plural definitions
        return _find_plural_definition_for_subject(subject, records)

    elif query_type == 'person':
        # For "Who is X?" - look for person descriptions
        return _find_person_description_for_subject(subject, records)

    elif query_type == 'explanation':
        # For "Why X?" - look for explanation content
        return _find_explanation_for_subject(subject, records)

    elif query_type == 'procedure':
        # For "How X?" - look for procedural content
        return _find_procedure_for_subject(subject, records)

    return None

def _find_definition_for_subject(subject, records):
    """Find formal definitions like 'X (description) is ...'"""
    # Priority 1: Content from chapter 1 that contains the formal definition
    for record in records:
        content = record.payload.get("content", "")
        source_path = record.payload.get('source_file_path', 'Unknown')
        content_lower = content.lower()

        # Check if it's from chapter 1 AND contains the definition pattern
        if 'chapter1.md' in source_path.lower():
            pattern = rf"\b{re.escape(subject)}\s*\([^)]+\)\s+is\s+"
            if re.search(pattern, content, re.IGNORECASE):
                return content

    # Priority 2: Exact definition pattern at the beginning of content (non-chapter1)
    for record in records:
        content = record.payload.get("content", "")
        source_path = record.payload.get('source_file_path', 'Unknown')
        content_lower = content.lower()

        # Skip chapter 1 here since we handled it in priority 1
        if 'chapter1.md' in source_path.lower():
            continue

        # Look for the exact definition pattern: "SUBJECT (Description) is ..." at the start
        pattern = rf"\b{re.escape(subject)}\s*\([^)]+\)\s+is\s+"
        if re.search(pattern, content, re.IGNORECASE):
            # Check if the definition appears early in the content
            match_pos = re.search(pattern, content, re.IGNORECASE).start()
            # If it appears in the first half of the content, it's likely the main definition
            if match_pos < len(content) / 2:
                return content

    # Priority 3: Content that starts with or early mentions the subject in a definition context
    for record in records:
        content = record.payload.get("content", "")
        source_path = record.payload.get('source_file_path', 'Unknown')
        content_lower = content.lower()

        # Skip chapter 1 here since we handled it in priority 1
        if 'chapter1.md' in source_path.lower():
            continue

        # Check if the content starts with the subject followed by definition
        words = content.split()
        if len(words) >= 2 and words[0].lower().startswith(subject.lower()) and ('is' in content_lower):
            return content

        # Check if subject appears early and is in definition format
        early_content = ' '.join(words[:15]) if len(words) > 15 else content
        if subject.lower() in early_content.lower() and ('is a' in content_lower or 'is an' in content_lower or 'is the' in content_lower):
            # Extra check: make sure it's defining the subject, not just mentioning it
            if content_lower.count(subject.lower()) >= 2 or f'{subject} (' in content_lower:
                return content

    # Priority 4: Any content with the formal definition pattern - but prioritize chapter 1 content
    chapter1_result = None
    other_result = None

    for record in records:
        content = record.payload.get("content", "")
        source_path = record.payload.get('source_file_path', 'Unknown')
        content_lower = content.lower()

        # Look for definition pattern anywhere in content
        pattern = rf"\b{re.escape(subject)}\s*\([^)]+\)\s+is\s+"
        if re.search(pattern, content, re.IGNORECASE):
            result = content

            # Prioritize chapter 1 content
            if 'chapter1.md' in source_path.lower():
                chapter1_result = result
            else:
                if other_result is None:
                    other_result = result

    # Return chapter 1 result if found, otherwise other result
    return chapter1_result or other_result

def _find_plural_definition_for_subject(subject, records):
    """Find plural definitions like 'X (descriptions) are ...'"""
    for record in records:
        content = record.payload.get("content", "")
        content_lower = content.lower()

        # Look for plural definition patterns
        pattern = rf"\b{re.escape(subject)}\s*\([^)]+\)\s+are\s+"
        if re.search(pattern, content, re.IGNORECASE):
            return content

        # Look for plural context like "X are defined as" or "X are the"
        if subject.lower() in content_lower and ('are ' in content_lower and ('defined as' in content_lower or 'the ' in content_lower)):
            return content

    return None

def _find_person_description_for_subject(subject, records):
    """Find descriptions about people/roles"""
    for record in records:
        content = record.payload.get("content", "")
        content_lower = content.lower()

        # Look for person descriptions
        if subject.lower() in content_lower and any(word in content_lower for word in ['developer', 'engineer', 'researcher', 'scientist', 'team', 'person', 'role', 'position']):
            return content

        # Look for "X is a Y" patterns where Y is a role
        pattern = rf"\b{re.escape(subject)}\s+is\s+(a|an)\s+(engineer|developer|researcher|scientist|expert|professional|team|member|person)"
        if re.search(pattern, content, re.IGNORECASE):
            return content

    return None

def _find_explanation_for_subject(subject, records):
    """Find explanation content for 'why' questions"""
    for record in records:
        content = record.payload.get("content", "")
        content_lower = content.lower()

        # Look for explanation keywords
        explanation_keywords = ['because', 'due to', 'reason', 'caused by', 'results from', 'explanation', 'reasons include', 'explained by']
        if subject.lower() in content_lower and any(keyword in content_lower for keyword in explanation_keywords):
            return content

        # Look for cause and effect patterns
        if subject.lower() in content_lower and ('because' in content_lower or 'reason' in content_lower):
            return content

    return None

def _find_procedure_for_subject(subject, records):
    """Find procedural content for 'how' questions"""
    for record in records:
        content = record.payload.get("content", "")
        content_lower = content.lower()

        # Look for procedural keywords
        procedure_keywords = ['first', 'second', 'next', 'then', 'finally', 'steps', 'process', 'method', 'approach', 'way', 'procedure', 'how to', 'instructions']
        if subject.lower() in content_lower and any(keyword in content_lower for keyword in procedure_keywords):
            return content

        # Look for step-by-step patterns
        if subject.lower() in content_lower and ('first,' in content_lower or '1.' in content_lower or 'step' in content_lower):
            return content

    return None


def summarize_answer(query: str, chunks: List[SourceChunk], retrieval_service) -> str:
    """
    Summarize the answer based on the query and retrieved chunks.
    Generic function that works across all book content using comprehensive semantic relevance.
    Enhanced to use direct definition search for "what is" queries.
    """
    # For specific query types, try to find direct answers first
    direct_answer = find_direct_answer(query)
    if direct_answer:
        return f"Based on the book content: {direct_answer}"
    # Get content from all chunks and filter out non-relevant content
    valid_contents = []
    for chunk in chunks:
        content = retrieval_service.get_chunk_content(chunk.chunk_id)
        if content and content.strip():
            content_clean = content.strip()
            # Filter out quiz questions, answers, and other non-content
            if (len(content_clean) > 20 and
                not any(marker in content_clean.lower() for marker in
                       ['question', 'quiz', 'answer:', 'multiple choice', 'true/false',
                        'question:', 'mcq', 'options', 'correct answer', 'explanation:'])):
                valid_contents.append((chunk, content_clean))

    if not valid_contents:
        return f"I couldn't find relevant information in the book to answer '{query}'. Please try rephrasing your question."

    # Comprehensive relevance scoring
    query_lower = query.lower().strip()
    query_words = set(word for word in query_lower.split() if len(word) > 2)

    scored_results = []
    for chunk, content in valid_contents:
        content_lower = content.lower()
        content_words = set(content_lower.split())

        # Calculate comprehensive relevance score
        score = 0

        # 1. Use the confidence score from the retrieval as a base
        # The retrieval service now returns scores based on both semantic and keyword matching
        base_confidence = getattr(chunk, 'confidence_score', 0.5)  # Get the confidence score from the chunk
        score = base_confidence * 50  # Scale it to contribute significantly to the total score

        # 2. For definition queries like "what is X", give extra weight to direct definitions
        if 'what is' in query_lower or query_lower.startswith('what is '):
            # Extract the subject being defined
            subject = ""
            if query_lower.startswith('what is '):
                subject = query_lower[8:].strip()

            if subject:
                # If content starts with the subject and contains definition patterns, give high score
                if content_lower.startswith(subject.lower()) and (' is ' in content_lower or ' refers to ' in content_lower):
                    score += 30  # Very high boost for direct definitions
                elif subject in content_lower:
                    score += 10  # Boost for content containing the subject

        # 3. Term overlap with emphasis on exact matches
        common_words = query_words.intersection(content_words)
        if common_words:
            # Weight rare/common words differently - common words get lower weight
            for word in common_words:
                # Boost for longer/more specific query terms
                score += max(1, len(word) - 2)

        # 4. Phrase matching - look for exact query phrases in content
        for phrase in [query_lower] + query_lower.split():
            if len(phrase) > 3:  # Only consider meaningful phrases
                phrase_matches = content_lower.count(phrase)
                if phrase_matches > 0:
                    score += phrase_matches * len(phrase)  # Longer phrases get higher weight

        # 5. Semantic pattern matching for different query types
        if 'what is' in query_lower or query_lower.startswith('what is '):
            # Prioritize definition patterns for "what is" questions
            definition_patterns = [' is a ', ' is an ', ' refers to ', ' means ', ' stands for ',
                                 ' defined as ', ' known as ', ' describes ', ' represents ']
            for pattern in definition_patterns:
                if pattern in content_lower:
                    score += 15  # Strong boost for definition patterns

        elif any(qw in query_lower for qw in ['how to', 'how do', 'steps', 'process', 'procedure']):
            # Prioritize procedural content for "how" questions
            procedural_patterns = ['first', 'then', 'next', 'finally', 'steps', 'process', 'method',
                                 'approach', 'technique', 'way to']
            for pattern in procedural_patterns:
                if pattern in content_lower:
                    score += 8

        elif any(qw in query_lower for qw in ['why', 'reason', 'because']):
            # Prioritize explanatory content for "why" questions
            explanatory_patterns = ['because', 'reason', 'due to', 'caused by', 'results from']
            for pattern in explanatory_patterns:
                if pattern in content_lower:
                    score += 10

        # 6. Content quality metrics
        sentences = content.count('.')
        words = len(content.split())

        # Prefer content with good sentence structure
        if sentences >= 1 and 50 <= words <= 500:  # Good length, not too short or long
            score += 5
        elif 20 <= words < 50:  # Short but complete content
            score += 3

        # 7. Subject relevance - check if main subject appears multiple times
        if len(query_words) > 0:
            main_subject = list(query_words)[0] if query_words else ""
            if main_subject and len(main_subject) > 3:  # Only consider meaningful subjects
                subject_frequency = content_lower.count(main_subject)
                if subject_frequency > 0:
                    score += subject_frequency * 3  # Boost based on subject relevance

        # 8. Chapter-topic relevance
        chunk_title_lower = chunk.section_title.lower()
        if any(topic in query_lower for topic in ['ros2', 'simulation', 'isaac', 'vla']) and \
           any(topic in chunk_title_lower for topic in ['ros2', 'simulation', 'isaac', 'vla']):
            score += 7  # Boost for topic-chapter alignment

        scored_results.append((chunk, content, score))

    # Sort by relevance score (highest first)
    scored_results.sort(key=lambda x: x[2], reverse=True)

    # For "what is" queries, try to find the most direct definition
    query_lower = query.lower().strip()
    if query_lower.startswith('what is '):
        subject = query_lower[8:].strip()
        if subject:
            # Look for the chunk that most directly defines the subject
            for chunk, content, score in scored_results:
                content_lower = content.lower()
                # Check if this content directly defines the subject we're looking for
                # Prioritize content like "ROS2 (Robot Operating System 2) is a flexible framework" over
                # "What is Isaac ROS. Isaac ROS is a collection of ROS2 packages"

                # More sophisticated check for direct definition
                # Look for the pattern where the subject is defined, not just mentioned
                import re

                # Check if the content contains the subject in a definition format like "SUBJECT (description) is ..."
                # This would match "ROS2 (Robot Operating System 2) is a flexible framework"
                definition_pattern = rf"\b{re.escape(subject.lower())}\s*\([^)]+\)\s+is\s+"
                if re.search(definition_pattern, content_lower):
                    return f"Based on the book content: {content.strip()}"

                # Also check if the content starts with subject followed by definition
                words = content_lower.split()
                if len(words) >= 2 and words[0] == subject.lower() and (' is ' in content_lower or ' refers to ' in content_lower):
                    return f"Based on the book content: {content.strip()}"

                # Check if subject appears early in content and is part of a definition
                early_content = ' '.join(words[:10]) if len(words) > 10 else content_lower
                if subject.lower() in early_content and (' is ' in content_lower or ' refers to ' in content_lower):
                    # Additional check: make sure the definition is about the subject, not just mentioning it
                    # For "What is Isaac ROS...", the subject "ROS2" appears but it's defining "Isaac ROS", not "ROS2"
                    # For "Chapter 1: Introduction to ROS2 What is ROS2. ROS2 (Robot Operating System 2)...",
                    # the definition is clearly about ROS2
                    if subject.lower() in content_lower.split('.')[0][:100]:  # Check in first sentence/portion
                        return f"Based on the book content: {content.strip()}"

    # Return the most relevant content based on scoring
    if scored_results:
        _, best_content, best_score = scored_results[0]
        if best_score > 0:
            return f"Based on the book content: {best_content.strip()}"
        else:
            # If best score is 0 or negative, still return the best available content
            return f"Based on the book content: {best_content.strip()}"
    else:
        # Fallback - should not reach here due to earlier check, but just in case
        _, fallback_content = valid_contents[0] if valid_contents else (None, "No relevant content found.")
        return f"Based on the book content: {fallback_content[:500].strip()}"


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
            summary = summarize_answer(query, source_chunks, self.retrieval_service)

            # If we have source chunks, we can provide more detailed information
            if source_chunks and "No relevant content found" not in summary and "couldn't find relevant information" not in summary:
                return summary
            else:
                # If no relevant content was found, try to be more specific about why
                if not source_chunks:
                    return f"I couldn't find any relevant information in the book to answer '{query}'. The search returned no results. Please try rephrasing your question."
                else:
                    return f"I couldn't find relevant information in the book to answer '{query}'. The retrieved content didn't match your query well. Please try rephrasing your question."
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