from typing import Optional
from .retrieval_service import retrieval_service
import logging

logger = logging.getLogger(__name__)


class TranslationService:
    def __init__(self):
        self.retrieval_service = retrieval_service

    async def translate_text(self, text: str, target_language: str = "ur") -> str:
        """
        Translate text to the target language.
        For Urdu translation, we'll try to find the corresponding Urdu content
        from the embedded documents.
        """
        try:
            if target_language.lower() == "ur":
                # For Urdu translation, we'll search for the corresponding Urdu content
                # This is a simplified approach - in a real system, you'd have more sophisticated
                # alignment between English and Urdu content

                # Try to find similar content in Urdu documents
                # First, search for relevant chunks in the knowledge base
                source_chunks = self.retrieval_service.retrieve_chunks(text, top_k=3, language="ur")

                if source_chunks:
                    # Return the first matching Urdu chunk if available
                    chunk_content = self.retrieval_service.get_chunk_content(source_chunks[0].chunk_id)
                    if chunk_content and chunk_content.strip():
                        return chunk_content

                # If no matching Urdu content found, return a message indicating that
                return f"[Urdu Translation Not Available]\n\nOriginal content:\n\n{text}"
            else:
                # For other languages, return a message indicating unsupported
                return f"[Translation to {target_language} Not Available]\n\nOriginal content:\n\n{text}"

        except Exception as e:
            # If translation fails, return original text with a note
            logger.error(f"Error in translation: {e}")
            return f"[Translation Error]\n\nOriginal content:\n\n{text}"


# Create a singleton instance
translation_service = TranslationService()