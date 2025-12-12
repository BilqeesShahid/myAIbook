from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging
import os
from dotenv import load_dotenv

from mangum import Mangum  # IMPORTANT for Vercel

# Absolute import
from src.api.routes import ask, selected_text, translate

# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Starting up RAG Chatbot API...")
    yield
    logger.info("Shutting down RAG Chatbot API...")

# Create application
app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot system",
    version="1.0.0",
    lifespan=lifespan
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(ask.router, prefix="/api", tags=["ask"])
app.include_router(selected_text.router, prefix="/api", tags=["selected-text"])
app.include_router(translate.router, prefix="/api", tags=["translate"])

# Root endpoint
@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

# Health
@app.get("/health")
async def health_check():
    return {"status": "healthy", "environment": os.getenv("FASTAPI_ENV", "development")}

# ⭐ REQUIRED FOR VERCEL
handler = Mangum(app)
