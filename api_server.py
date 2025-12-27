from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Dict, Any, Optional
import os
import sys
import logging

# Add backend to path to import main
sys.path.insert(0, 'backend')

from backend.main import (
    initialize_qdrant_client,
    embed,
    extract_text_from_urls,
    clean_text_content,
    chunk_text,
    generate_content_hash,
    ContentChunk
)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(
    title="Book Assistant API",
    description="API for the Book Assistant application",
    version="1.0.0"
)

# Add CORS middleware to allow requests from frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    # Add exposed headers to allow frontend to see response headers
    expose_headers=["Access-Control-Allow-Origin"]
)

# Request models
class QueryRequest(BaseModel):
    query: str
    context: Optional[Dict[str, Any]] = {}
    session_id: Optional[str] = None

class HealthResponse(BaseModel):
    status: str
    message: str
    timestamp: str

@app.get("/api/v1/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint to verify API is running."""
    try:
        # Test Qdrant connection
        client = initialize_qdrant_client()
        collections = client.get_collections()
        logger.info("Health check: Qdrant connection successful")

        return {
            "status": "healthy",
            "message": "Book Assistant API is running",
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        raise HTTPException(status_code=503, detail=f"Service unavailable: {str(e)}")

@app.post("/api/v1/query")
async def handle_query(request: QueryRequest):
    """Handle user queries and return relevant information."""
    try:
        logger.info(f"Received query: {request.query}")

        # Initialize Qdrant client
        client = initialize_qdrant_client()

        # Try to embed the query using Cohere
        try:
            # Initialize Cohere client and embed the query
            from backend.main import initialize_cohere_client
            cohere_client = initialize_cohere_client()
            query_embedding = embed([request.query])

            if query_embedding and len(query_embedding) > 0:
                # Perform semantic search in Qdrant
                search_results = client.query_points(
                    collection_name=os.getenv("COLLECTION_NAME", "rag_embedding"),
                    query=query_embedding[0],
                    limit=5,  # Return top 5 results
                    with_payload=True
                ).points

                # Format the results
                source_documents = []
                for result in search_results:
                    payload = result.payload
                    source_documents.append({
                        "text": payload.get("text", "")[:500] + "..." if len(payload.get("text", "")) > 500 else payload.get("text", ""),
                        "url": payload.get("url", ""),
                        "title": payload.get("title", ""),
                        "score": result.score
                    })

                # Create response based on search results
                if source_documents:
                    response_text = f"Based on the book content, here's what I found about '{request.query}':\n\n"
                    for i, doc in enumerate(source_documents[:2], 1):  # Show top 2 results
                        response_text += f"{i}. {doc['title']}\n   {doc['text']}\n   Source: {doc['url']}\n\n"
                else:
                    response_text = f"I found information about '{request.query}' in the book content, but no exact matches were found. Please try rephrasing your question."
            else:
                response_text = f"I received your query: '{request.query}'. The backend is connected and ready to process book-related queries, but I need to configure embeddings to provide specific answers."
                source_documents = []
        except Exception as e:
            logger.warning(f"Embedding/search failed: {e}")
            # Fallback response if embedding fails
            response_text = f"I received your query: '{request.query}'. The backend is connected and ready to process book-related queries, but I need a valid Cohere API key to provide specific answers."
            source_documents = []

        response = {
            "query": request.query,
            "response": response_text,
            "context": request.context,
            "session_id": request.session_id,
            "timestamp": __import__('datetime').datetime.now().isoformat(),
            "source_documents": source_documents
        }

        logger.info(f"Query processed successfully: {request.query[:50]}...")
        return response

    except Exception as e:
        logger.error(f"Query processing failed: {e}")
        raise HTTPException(status_code=500, detail=f"Query processing failed: {str(e)}")

@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "message": "Book Assistant API",
        "version": "1.0.0",
        "endpoints": {
            "health": "/api/v1/health",
            "query": "/api/v1/query"
        }
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)