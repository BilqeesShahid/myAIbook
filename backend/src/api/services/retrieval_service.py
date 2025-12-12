import logging
from typing import List, Optional
from ..models.embedding import DocumentChunk
from ..models.query import SourceChunk
from ..models.query import QueryRequest
from ..services.tfidf_embedding_service import tfidf_embedding_service
from ..utils.helpers import calculate_similarity_score

logger = logging.getLogger(__name__)


def reformulate_query(query: str) -> str:
    """
    Reformulate the user query to improve search results.
    This function can expand the query with synonyms or related terms.
    """
    # For now, we'll return the query as is, but this can be enhanced with
    # NLP techniques to expand the query with synonyms or related terms
    # In a more advanced implementation, we might use techniques like:
    # - Query expansion with synonyms
    # - Named entity recognition
    # - Query understanding and reformulation

    # For now, just return the original query
    # In the future, we could enhance this with more sophisticated techniques
    return query


class RetrievalService:
    """
    Service for retrieving relevant document chunks based on queries.
    Handles both general queries and selected text prioritization.
    """

    def __init__(self):
        self.embedding_service = tfidf_embedding_service

    def retrieve_chunks(self, query: str, top_k: int = 5, language: Optional[str] = None) -> List[SourceChunk]:
        """
        Retrieve the most relevant chunks for a given query.
        """
        try:
            # Reformulate the query before searching
            reformulated_query = reformulate_query(query)

            # Search for similar chunks in the vector database
            similar_chunks = self.embedding_service.search_similar(reformulated_query, top_k, language=language)

            # Convert to SourceChunk objects with confidence scores
            source_chunks = []
            for chunk in similar_chunks:
                # Calculate confidence score based on similarity
                # For now, we'll use a simple approach; in a real system,
                # this would be based on the distance returned by the vector search
                confidence_score = 0.8  # Placeholder - would be calculated from distance in real implementation

                source_chunk = SourceChunk(
                    chunk_id=chunk.id,
                    chapter_number=chunk.chapter_number,
                    section_title=chunk.section_title,
                    source_file_path=chunk.source_file_path,
                    confidence_score=confidence_score
                )
                source_chunks.append(source_chunk)

            return source_chunks
        except Exception as e:
            logger.error(f"Error retrieving chunks: {e}")
            raise

    def retrieve_chunks_with_selected_text(self, query: str, selected_text: str, top_k: int = 5) -> List[SourceChunk]:
        """
        Retrieve chunks prioritizing the selected text.
        """
        try:
            # Reformulate the query before searching
            reformulated_query = reformulate_query(query)

            # First, search for chunks relevant to the selected text
            selected_text_chunks = self.embedding_service.search_similar(selected_text, top_k // 2)

            # Then, search for chunks relevant to the query
            query_chunks = self.embedding_service.search_similar(reformulated_query, top_k // 2)

            # Combine and deduplicate chunks
            all_chunks = selected_text_chunks + query_chunks
            seen_ids = set()
            unique_chunks = []

            for chunk in all_chunks:
                if chunk.id not in seen_ids:
                    unique_chunks.append(chunk)
                    seen_ids.add(chunk.id)

            # Convert to SourceChunk objects
            source_chunks = []
            for i, chunk in enumerate(unique_chunks[:top_k]):
                # Higher confidence for chunks from selected text
                is_selected_text = i < len(selected_text_chunks)
                confidence_score = 0.9 if is_selected_text else 0.7

                source_chunk = SourceChunk(
                    chunk_id=chunk.id,
                    chapter_number=chunk.chapter_number,
                    section_title=chunk.section_title,
                    source_file_path=chunk.source_file_path,
                    confidence_score=confidence_score,
                    is_selected_text=is_selected_text
                )
                source_chunks.append(source_chunk)

            return source_chunks
        except Exception as e:
            logger.error(f"Error retrieving chunks with selected text: {e}")
            raise

    def retrieve_chunks_by_request(self, request: QueryRequest) -> List[SourceChunk]:
        """
        Retrieve chunks based on a query request, handling both regular and selected text queries.
        """
        if request.selected_text:
            # Use selected text prioritization
            return self.retrieve_chunks_with_selected_text(
                request.query,
                request.selected_text,
                top_k=5
            )
        else:
            # Use regular retrieval
            return self.retrieve_chunks(
                request.query,
                top_k=5
            )

    def get_chunk_content(self, chunk_id: str) -> Optional[str]:
        """
        Retrieve the content of a specific chunk by its ID.
        """
        try:
            chunk = self.embedding_service.get_embedding(chunk_id)
            return chunk.content if chunk else None
        except Exception as e:
            logger.error(f"Error retrieving chunk content: {e}")
            return None


# Singleton instance
retrieval_service = RetrievalService()