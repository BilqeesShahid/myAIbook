from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging
import os
from dotenv import load_dotenv

<<<<<<< HEAD
<<<<<<< HEAD
from src.api.routes import ask, selected_text, translate
=======
from mangum import Mangum  # IMPORTANT for Vercel
=======
 
>>>>>>> 40bdb1f (Save my local changes)

# Absolute import
from src.api.routes import ask
>>>>>>> f4f799c (Add latest backend changes)

load_dotenv()

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info("Starting up RAG Chatbot API...")
    yield
    logger.info("Shutting down RAG Chatbot API...")

app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot system",
    version="1.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(ask.router, prefix="/api", tags=["ask"])

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
<<<<<<< HEAD
    return {"status": "healthy"}
=======
    return {"status": "healthy", "environment": os.getenv("FASTAPI_ENV", "development")}

 
>>>>>>> 40bdb1f (Save my local changes)
