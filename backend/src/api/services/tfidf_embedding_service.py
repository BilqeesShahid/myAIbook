import logging
import os
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from sklearn.feature_extraction.text import TfidfVectorizer
import numpy as np
import pickle

from ..models.embedding import DocumentChunk
from ..utils.constants import (
    QDRANT_URL,
    QDRANT_API_KEY,
    QDRANT_COLLECTION_NAME,
    EMBEDDING_DIMENSION
)

logger = logging.getLogger(__name__)


class TFIDFEmbeddingService:
    """
    Service for generating and storing embeddings using TF-IDF.
    This avoids external API calls and quota limits.
    """

    def __init__(self):
        # Initialize TF-IDF vectorizer with the same number of features as the embedding dimension
        # We'll pad the vectors to reach the target dimension
        self.vectorizer_cache_path = "tfidf_vectorizer.pkl"

        # Try to load a previously fitted vectorizer if it exists
        if os.path.exists(self.vectorizer_cache_path):
            try:
                with open(self.vectorizer_cache_path, 'rb') as f:
                    self.vectorizer = pickle.load(f)
                self.is_fitted = True
                logger.info("Loaded pre-fitted TF-IDF vectorizer from cache")
            except Exception as e:
                logger.warning(f"Could not load cached vectorizer: {e}, creating new one")
                self.vectorizer = TfidfVectorizer(
                    max_features=EMBEDDING_DIMENSION,  # Use the full embedding dimension
                    stop_words='english',
                    lowercase=True,
                    ngram_range=(1, 2)  # Include unigrams and bigrams
                )
                self.is_fitted = False
        else:
            self.vectorizer = TfidfVectorizer(
                max_features=EMBEDDING_DIMENSION,  # Use the full embedding dimension
                stop_words='english',
                lowercase=True,
                ngram_range=(1, 2)  # Include unigrams and bigrams
            )
            self.is_fitted = False

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            prefer_grpc=False  # Using HTTP for better compatibility
        )

        # Ensure collection exists
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the Qdrant collection exists with the correct configuration.
        """
        try:
            # Try to get collection info to see if it exists
            collection_info = self.qdrant_client.get_collection(QDRANT_COLLECTION_NAME)
            logger.info(f"Collection {QDRANT_COLLECTION_NAME} already exists")
            # Check if the vector size matches our expected dimension
            if collection_info.config.params.vectors.size != EMBEDDING_DIMENSION:
                logger.warning(f"Collection vector size {collection_info.config.params.vectors.size} doesn't match expected {EMBEDDING_DIMENSION}")
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=QDRANT_COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=EMBEDDING_DIMENSION,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {QDRANT_COLLECTION_NAME}")

    def _fit_vectorizer(self, texts: List[str]):
        """
        Fit the TF-IDF vectorizer on the provided texts.
        """
        if not self.is_fitted:
            self.vectorizer.fit(texts)
            self.is_fitted = True
            # Cache the fitted vectorizer
            with open(self.vectorizer_cache_path, 'wb') as f:
                pickle.dump(self.vectorizer, f)

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a given text using TF-IDF.
        """
        try:
            # If vectorizer is not fitted yet, we can't generate embeddings for single texts
            # This method should only be called after batch fitting
            if not self.is_fitted:
                # For single text, we'll transform using the existing vectorizer if possible
                # but this is not ideal - better to use batch processing
                # As a fallback, we'll return a zero vector of the expected size
                return [0.0] * EMBEDDING_DIMENSION

            # Transform the text
            embedding = self.vectorizer.transform([text]).toarray()[0]
            embedding_list = embedding.tolist()

            # Pad the embedding to match the required dimension
            padded_embedding = self._pad_vector(embedding_list, EMBEDDING_DIMENSION)
            return padded_embedding
        except Exception as e:
            logger.error(f"Error generating TF-IDF embedding: {e}")
            # Return zero vector as fallback
            return [0.0] * EMBEDDING_DIMENSION

    def _pad_vector(self, vector: List[float], target_dim: int) -> List[float]:
        """
        Pad a vector to the target dimension with zeros.
        """
        if len(vector) >= target_dim:
            return vector[:target_dim]  # Truncate if too long
        else:
            # Pad with zeros
            padded = vector + [0.0] * (target_dim - len(vector))
            return padded

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts using TF-IDF.
        This is the preferred method as it properly fits the vectorizer.
        """
        try:
            # Fit the vectorizer on the texts
            self._fit_vectorizer(texts)

            # Transform all texts
            embeddings_matrix = self.vectorizer.transform(texts).toarray()

            # Convert to list of lists and pad each vector to target dimension
            embeddings = []
            for embedding_row in embeddings_matrix:
                embedding_list = embedding_row.tolist()
                padded_embedding = self._pad_vector(embedding_list, EMBEDDING_DIMENSION)
                embeddings.append(padded_embedding)

            return embeddings
        except Exception as e:
            logger.error(f"Error generating batch TF-IDF embeddings: {e}")
            raise

    def store_embedding(self, document_chunk: DocumentChunk) -> str:
        """
        Store a document chunk with its embedding in Qdrant.
        Note: For single chunks, this uses a fallback approach.
        """
        try:
            # For single chunks, we'll use the fallback method
            embedding_vector = self.generate_embedding(document_chunk.content)

            # Prepare the point for Qdrant
            point = PointStruct(
                id=document_chunk.id,
                vector=embedding_vector,
                payload={
                    "content": document_chunk.content,
                    "chapter_number": document_chunk.chapter_number,
                    "section_title": document_chunk.section_title,
                    "source_file_path": document_chunk.source_file_path,
                    "chunk_index": document_chunk.chunk_index,
                    "metadata": document_chunk.metadata or {}
                }
            )

            # Store in Qdrant
            self.qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=[point]
            )

            return document_chunk.id
        except Exception as e:
            logger.error(f"Error storing embedding: {e}")
            raise

    def store_embeddings_batch(self, document_chunks: List[DocumentChunk]) -> List[str]:
        """
        Store multiple document chunks with their embeddings in Qdrant.
        This method is more efficient as it uses batch processing.
        """
        try:
            # Extract content from all chunks
            texts = [chunk.content for chunk in document_chunks]

            # Generate embeddings in batch for efficiency
            embedding_vectors = self.generate_embeddings_batch(texts)

            point_structs = []
            for i, chunk in enumerate(document_chunks):
                point = PointStruct(
                    id=chunk.id,
                    vector=embedding_vectors[i],
                    payload={
                        "content": chunk.content,
                        "chapter_number": chunk.chapter_number,
                        "section_title": chunk.section_title,
                        "source_file_path": chunk.source_file_path,
                        "chunk_index": chunk.chunk_index,
                        "metadata": chunk.metadata or {}
                    }
                )
                point_structs.append(point)

            # Store all points in Qdrant
            self.qdrant_client.upsert(
                collection_name=QDRANT_COLLECTION_NAME,
                points=point_structs
            )

            return [chunk.id for chunk in document_chunks]
        except Exception as e:
            logger.error(f"Error storing embeddings batch: {e}")
            raise

    def search_similar(self, query: str, top_k: int = 5, language: Optional[str] = None) -> List[DocumentChunk]:
        """
        Search for similar document chunks to the query.
        This properly handles query transformation using the fitted vectorizer.
        Enhanced to better handle definition-type queries like "what is X".
        Now includes keyword-based search to complement semantic search.
        """
        try:
            # For TF-IDF, we need to use the fitted vectorizer to transform the query
            if not self.is_fitted:
                # If not fitted, we need to fit it first with some sample text
                logger.info("TF-IDF vectorizer not fitted yet. Fitting with sample text...")
                # We'll fit with a simple sample - in practice, this should have been done during embedding
                sample_texts = [query]  # Use the query itself as sample to ensure basic fitting
                self.vectorizer.fit(sample_texts)
                self.is_fitted = True

            # Enhance query for better matching, especially for "what is" questions
            enhanced_query = self._enhance_query(query)

            # Perform both semantic search and keyword search
            semantic_results = self._semantic_search(enhanced_query, top_k * 2, language)
            keyword_results = self._keyword_search(query, top_k * 2, language)

            # Combine and rerank results based on both semantic and keyword relevance
            combined_results = self._combine_and_rerank(semantic_results, keyword_results, query, top_k)

            # Convert results to DocumentChunk objects
            results = []
            for hit in combined_results:
                payload = hit.payload
                # Ensure embedding_vector is properly handled (it might be None in search results)
                embedding_vector = hit.vector if hit.vector is not None else [0.0] * EMBEDDING_DIMENSION
                chunk = DocumentChunk(
                    id=hit.id,
                    content=payload.get("content", ""),
                    chapter_number=payload.get("chapter_number", 0),
                    section_title=payload.get("section_title", ""),
                    source_file_path=payload.get("source_file_path", ""),
                    embedding_vector=embedding_vector,
                    chunk_index=payload.get("chunk_index", 0),
                    metadata=payload.get("metadata", {})
                )
                results.append(chunk)

            return results
        except Exception as e:
            logger.error(f"Error searching for similar documents: {e}")
            raise

    def _semantic_search(self, query: str, top_k: int, language: Optional[str] = None) -> List:
        """
        Perform semantic search using TF-IDF vectors.
        """
        # Transform query to vector using the fitted vectorizer
        # This ensures the query uses the same vocabulary as the stored documents
        try:
            query_vector_sparse = self.vectorizer.transform([query])
            query_vector = query_vector_sparse.toarray()[0].tolist()

            # Pad the query vector to match the stored dimension
            padded_query_vector = self._pad_vector(query_vector, EMBEDDING_DIMENSION)
        except ValueError as e:
            # If query contains terms not in fitted vocabulary, it may cause issues
            logger.warning(f"Query transformation issue: {e}. Using original query as fallback.")
            # Try with original query
            try:
                query_vector_sparse = self.vectorizer.transform([query])
                query_vector = query_vector_sparse.toarray()[0].tolist()
                padded_query_vector = self._pad_vector(query_vector, EMBEDDING_DIMENSION)
            except:
                logger.warning(f"Query transformation failed. Using zero vector as fallback.")
                padded_query_vector = [0.0] * EMBEDDING_DIMENSION

        # Prepare search filter for language if specified
        search_filter = None
        if language:
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="metadata.language",
                        match=models.MatchValue(value=language)
                    )
                ]
            )

        # Search in Qdrant using the query vector
        search_results = self.qdrant_client.search(
            collection_name=QDRANT_COLLECTION_NAME,
            query_vector=padded_query_vector,
            limit=top_k,
            query_filter=search_filter  # Apply language filter if specified
        )

        return search_results

    def _keyword_search(self, query: str, top_k: int, language: Optional[str] = None) -> List:
        """
        Perform keyword-based search to find documents containing query terms.
        This complements the semantic search by finding exact matches.
        Enhanced to prioritize definition-like content for "what is" queries.
        """
        # Extract important terms from the query
        query_lower = query.lower()
        # Split query into terms, removing common stop words for better matching
        query_terms = [term.strip() for term in query_lower.split() if len(term.strip()) > 2 and term.strip() not in {'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by'}]

        if not query_terms:
            return []

        # First, retrieve all documents to perform keyword matching
        # This is more efficient for keyword matching than using Qdrant filters for text
        try:
            # Prepare search filter for language if specified
            search_filter = None
            if language:
                search_filter = models.Filter(
                    must=[
                        models.FieldCondition(
                            key="metadata.language",
                            match=models.MatchValue(value=language)
                        )
                    ]
                )

            # Retrieve all documents (or a large number) to perform keyword matching
            all_results = self.qdrant_client.scroll(
                collection_name=QDRANT_COLLECTION_NAME,
                scroll_filter=search_filter,
                limit=1000,  # Adjust based on your collection size
                with_payload=True,
                with_vectors=False
            )

            # The scroll method returns (records, next_page_offset), so we need to unpack correctly
            records, _ = all_results if isinstance(all_results, tuple) else (all_results, None)

            # Perform keyword matching on retrieved documents
            keyword_matched = []
            for record in records:
                content = record.payload.get("content", "").lower()
                original_content = record.payload.get("content", "")  # Keep original for position checks
                score = 0

                # Calculate keyword relevance score
                for term in query_terms:
                    term_count = content.count(term)
                    if term_count > 0:
                        score += term_count * len(term)  # Weight longer terms more

                # Special handling for "what is" queries - look for definition patterns
                if 'what is ' in query_lower:
                    subject = query_lower[8:].strip()
                    if subject:
                        # Boost score if content contains definition patterns with the subject
                        if f'{subject} is ' in content or f'{subject.lower()} is ' in content:
                            score += 20  # Strong boost for "X is ..." patterns
                        elif f'{subject} means' in content or f'{subject.lower()} means' in content:
                            score += 15  # Boost for "X means ..." patterns
                        elif f'{subject} refers to' in content or f'{subject.lower()} refers to' in content:
                            score += 15  # Boost for "X refers to ..." patterns

                        # Check if content starts with the subject (like "ROS2 (Robot Operating System 2) is a flexible framework...")
                        content_words = original_content.split()
                        if len(content_words) > 0 and content_words[0].lower().replace(':', '').replace(',', '').replace('.', '') == subject.lower():
                            score += 10  # Boost if subject is the first word

                # If content contains exact phrase matches, boost the score
                if query_lower in content:
                    score += 10  # Significant boost for exact phrase matches

                # Boost for content that looks like a definition (contains "is a", "is an", etc.)
                definition_patterns = [' is a ', ' is an ', ' refers to ', ' means ', ' stands for ', ' is defined as ']
                for pattern in definition_patterns:
                    if pattern in content:
                        score += 5  # Boost for definition-like content

                if score > 0:  # Only include documents with some keyword match
                    # Create a mock search result object with keyword score
                    # We'll create a simple class that mimics the Qdrant search result structure
                    class MockResult:
                        def __init__(self, id, payload, score):
                            self.id = id
                            self.payload = payload
                            self.score = score
                            self.vector = None

                    mock_result = MockResult(record.id, record.payload, min(1.0, score / 10.0))
                    keyword_matched.append(mock_result)

            # Sort by keyword score in descending order and return top_k
            keyword_matched.sort(key=lambda x: x.score, reverse=True)
            return keyword_matched[:top_k]

        except Exception as e:
            logger.warning(f"Keyword search failed: {e}. Falling back to semantic search only.")
            return []

    def _combine_and_rerank(self, semantic_results: List, keyword_results: List, query: str, top_k: int) -> List:
        """
        Combine semantic and keyword search results and rerank them based on a combined score.
        """
        # Create a dictionary to store unique results with their scores
        result_scores = {}

        # Add semantic results with their scores
        for hit in semantic_results:
            if hit.id not in result_scores:
                # Normalize semantic score to 0-1 range and weight it
                semantic_score = max(0, hit.score)  # Ensure non-negative
                result_scores[hit.id] = {
                    'hit': hit,
                    'semantic_score': semantic_score,
                    'keyword_score': 0.0,
                    'combined_score': semantic_score * 0.7  # Weight semantic score 70%
                }
            else:
                # Update semantic score if it's higher
                if hit.score > result_scores[hit.id]['semantic_score']:
                    result_scores[hit.id]['semantic_score'] = hit.score
                    result_scores[hit.id]['combined_score'] = hit.score * 0.7 + result_scores[hit.id]['keyword_score'] * 0.3

        # Add keyword results with their scores
        for hit in keyword_results:
            if hit.id not in result_scores:
                # Calculate keyword relevance score based on content matching
                content = hit.payload.get("content", "").lower()
                query_terms = [term.strip() for term in query.lower().split() if len(term.strip()) > 2]

                keyword_score = 0
                for term in query_terms:
                    term_count = content.count(term)
                    if term_count > 0:
                        keyword_score += term_count * len(term)  # Weight longer terms more

                # If content contains exact phrase matches, boost the score
                query_lower = query.lower()
                if query_lower in content:
                    keyword_score += 10  # Significant boost for exact phrase matches

                result_scores[hit.id] = {
                    'hit': hit,
                    'semantic_score': 0.0,
                    'keyword_score': min(1.0, keyword_score / 10.0),  # Normalize to 0-1 range
                    'combined_score': keyword_score / 10.0 * 0.3  # Weight keyword score 30%
                }
            else:
                # Update keyword score and combined score
                content = hit.payload.get("content", "").lower()
                query_terms = [term.strip() for term in query.lower().split() if len(term.strip()) > 2]

                keyword_score = 0
                for term in query_terms:
                    term_count = content.count(term)
                    if term_count > 0:
                        keyword_score += term_count * len(term)

                # If content contains exact phrase matches, boost the score
                query_lower = query.lower()
                if query_lower in content:
                    keyword_score += 10

                result_scores[hit.id]['keyword_score'] = min(1.0, keyword_score / 10.0)
                result_scores[hit.id]['combined_score'] = result_scores[hit.id]['semantic_score'] * 0.7 + min(1.0, keyword_score / 10.0) * 0.3

        # Sort results by combined score in descending order
        sorted_results = sorted(result_scores.values(), key=lambda x: x['combined_score'], reverse=True)

        # Return top_k results
        return [item['hit'] for item in sorted_results[:top_k]]

    def _enhance_query(self, query: str) -> str:
        """
        Enhance the query for better matching, especially for definition-type questions.
        Enhanced to expand acronyms and related terms.
        """
        query_lower = query.lower().strip()
        enhanced_query = query

        # If it's a "what is" question, enhance it to include variations
        if query_lower.startswith('what is '):
            # Extract the subject: "what is ROS2" -> "ROS2"
            subject = query_lower[8:].strip()  # Remove "what is " and get the subject
            if subject:
                # Expand acronyms - if we have something like "ROS2", also search for "Robot Operating System"
                expanded_terms = self._expand_acronyms(subject)
                # Return both the original query and the subject and expanded terms for better matching
                enhanced_query = f"{query} {subject} {' '.join(expanded_terms)}"

        # For other types of queries, also expand acronyms
        else:
            expanded_terms = self._expand_acronyms(query)
            if expanded_terms:
                enhanced_query = f"{query} {' '.join(expanded_terms)}"

        return enhanced_query

    def _expand_acronyms(self, text: str) -> List[str]:
        """
        Expand common acronyms in the text to their full forms for better matching.
        """
        expansions = {
            'ros2': ['robot operating system'],
            'ros': ['robot operating system'],
            'ai': ['artificial intelligence'],
            'ml': ['machine learning'],
            'dl': ['deep learning'],
            'gpu': ['graphics processing unit'],
            'cpu': ['central processing unit'],
            'api': ['application programming interface'],
            'sdk': ['software development kit'],
            'iot': ['internet of things'],
            'nlp': ['natural language processing'],
            'cv': ['computer vision'],
            'rl': ['reinforcement learning'],
            'isaac': ['nvidia isaac'],
            'sim': ['simulation']
        }

        text_lower = text.lower()
        found_expansions = []

        for acronym, full_forms in expansions.items():
            if acronym in text_lower:
                found_expansions.extend(full_forms)

        return found_expansions

    def delete_embedding(self, chunk_id: str):
        """
        Delete an embedding from Qdrant by ID.
        """
        try:
            self.qdrant_client.delete(
                collection_name=QDRANT_COLLECTION_NAME,
                points_selector=models.PointIdsList(
                    points=[chunk_id]
                )
            )
        except Exception as e:
            logger.error(f"Error deleting embedding: {e}")
            raise

    def delete_all_embeddings(self):
        """
        Delete all embeddings from the Qdrant collection.
        """
        try:
            # Delete all points in the collection
            self.qdrant_client.delete(
                collection_name=QDRANT_COLLECTION_NAME,
                points_selector=models.FilterSelector(
                    filter=models.Filter(
                        must=[]
                    )
                )
            )
            logger.info("All embeddings deleted from collection")
        except Exception as e:
            logger.error(f"Error deleting all embeddings: {e}")
            raise

    def get_embedding(self, chunk_id: str) -> Optional[DocumentChunk]:
        """
        Retrieve a document chunk by its ID.
        """
        try:
            records = self.qdrant_client.retrieve(
                collection_name=QDRANT_COLLECTION_NAME,
                ids=[chunk_id]
            )

            if records:
                record = records[0]
                payload = record.payload
                return DocumentChunk(
                    id=record.id,
                    content=payload.get("content", ""),
                    chapter_number=payload.get("chapter_number", 0),
                    section_title=payload.get("section_title", ""),
                    source_file_path=payload.get("source_file_path", ""),
                    embedding_vector=record.vector,
                    chunk_index=payload.get("chunk_index", 0),
                    metadata=payload.get("metadata", {})
                )
            return None
        except Exception as e:
            logger.error(f"Error retrieving embedding: {e}")
            raise


# Singleton instance
tfidf_embedding_service = TFIDFEmbeddingService()