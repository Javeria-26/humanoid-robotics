from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .rag_agent.api import router as rag_agent_router
import os
from dotenv import load_dotenv
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded

# Load environment variables
load_dotenv()

# Create FastAPI application
app = FastAPI(
    title="RAG Agent API",
    description="API for the RAG-enabled AI agent that answers questions using retrieved book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiter middleware
limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.exception_handler(RateLimitExceeded)(_rate_limit_exceeded_handler)

# Include the RAG agent router
app.include_router(rag_agent_router, prefix="/api/v1", tags=["rag-agent"])

# Health check endpoint is included in the rag_agent router


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )