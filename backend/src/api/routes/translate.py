from fastapi import APIRouter, HTTPException
from datetime import datetime
import uuid
from pydantic import BaseModel

# Absolute imports
from src.api.services.translation_service import translation_service

router = APIRouter()

class TranslateRequest(BaseModel):
    text: str
    target_language: str = "ur"


class TranslateResponse(BaseModel):
    translated_text: str
    source_language: str
    target_language: str
    timestamp: datetime


@router.post("/translate", response_model=TranslateResponse)
async def translate_text(request: TranslateRequest):
    """
    Endpoint to translate text to the target language.
    """
    try:
        # Validate request
        if not request.text or len(request.text.strip()) == 0:
            raise HTTPException(status_code=400, detail="Text cannot be empty")

        if len(request.text) > 10000:  # Max content length
            raise HTTPException(status_code=400, detail="Text is too long")

        if not request.target_language:
            raise HTTPException(status_code=400, detail="Target language cannot be empty")

        # Get translation from translation service
        translated_text = await translation_service.translate_text(
            request.text,
            request.target_language
        )

        # Create response object
        response = TranslateResponse(
            translated_text=translated_text,
            source_language="en",  # Assuming source is always English for now
            target_language=request.target_language,
            timestamp=datetime.now()
        )

        return response
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")