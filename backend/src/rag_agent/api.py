from fastapi import APIRouter, HTTPException, Depends, BackgroundTasks, Request
from typing import Dict, Any
import time
from .models import QueryRequest, QueryResponse, ErrorResponse
from .retrieval import retrieval_service
from .agent import rag_agent
from .utils import format_response_for_api, validate_query_text, sanitize_input
from datetime import datetime
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from aiocache import cached, Cache
import hashlib

# Create API router
router = APIRouter()

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
router._limiter = limiter

# Initialize cache
cache = Cache(Cache.MEMORY, ttl=300)  # Cache for 5 minutes

# Note: Rate limit exception handler will be managed by the main app


@router.get("/health", summary="Health check endpoint")
async def health_check() -> Dict[str, Any]:
    """
    Check if the RAG agent service is running and healthy
    """
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat()
    }


# Cache key generator function
def get_cache_key(query: str, max_results: int, include_citations: bool) -> str:
    """
    Generate a cache key based on query parameters
    """
    key_string = f"{query}:{max_results}:{include_citations}"
    return hashlib.md5(key_string.encode()).hexdigest()

async def process_query(query_request: QueryRequest) -> QueryResponse:
    """
    Process the query without rate limiting - used for caching
    """
    # Validate the query text
    if not validate_query_text(query_request.query):
        raise HTTPException(
            status_code=400,
            detail="Query must be between 1 and 1000 characters and cannot be empty"
        )

    # Sanitize the input
    sanitized_query = sanitize_input(query_request.query)

    # Additional validation for parameters
    if query_request.max_results < 1 or query_request.max_results > 20:
        raise HTTPException(
            status_code=400,
            detail="max_results must be between 1 and 20"
        )

    # Start timing the processing
    start_time = time.time()

    # Retrieve relevant documents from Qdrant
    retrieved_docs = retrieval_service.retrieve_documents(
        sanitized_query,
        max_results=query_request.max_results
    )

    # If no documents found, handle appropriately
    if not retrieved_docs:
        # Return a response indicating no relevant documents were found
        processing_time = (time.time() - start_time) * 1000

        # Create a response indicating no results
        api_response = format_response_for_api(
            query=sanitized_query,
            answer="No relevant documents were found to answer your question.",
            citations=[],
            retrieved_docs_count=0,
            processing_time=processing_time
        )

        return api_response

    # Generate response using the RAG agent with validation
    generated_response = rag_agent.generate_and_validate_response(
        sanitized_query,
        retrieved_docs,
        include_citations=query_request.include_citations
    )

    # Calculate processing time in milliseconds
    processing_time = (time.time() - start_time) * 1000

    # Format the response according to API specification
    api_response = format_response_for_api(
        query=sanitized_query,
        answer=generated_response.response_text,
        citations=generated_response.citations,
        retrieved_docs_count=len(retrieved_docs),
        processing_time=processing_time
    )

    return api_response

@router.post("/query", summary="Query the RAG agent with a question about book content")
@limiter.limit("10/minute")  # 10 requests per minute per IP
async def query_endpoint(request: Request, query_request: QueryRequest) -> QueryResponse:
    """
    Submit a natural language query to retrieve answers from book content with citations
    """
    try:
        # Generate cache key
        cache_key = get_cache_key(
            query_request.query,
            query_request.max_results,
            query_request.include_citations
        )

        # Try to get from cache first
        cached_result = await cache.get(cache_key)
        if cached_result:
            # Add cache hit info to response
            import logging
            logging.info(f"Cache hit for query: {query_request.query[:50]}...")
            return cached_result

        # Process the query
        result = await process_query(query_request)

        # Store in cache
        await cache.set(cache_key, result)

        # Log the request
        import logging
        logging.info(f"Processed query: {query_request.query[:50]}... "
                    f"Retrieved {result.retrieved_documents_count} documents, "
                    f"Processing time: {result.processing_time}ms")

        return result

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log the error
        import logging
        logging.error(f"Error processing query: {e}")

        # Return appropriate error response
        error_response = ErrorResponse(
            error="QUERY_FAILED",
            message=f"Failed to process query: {str(e)}"
        )
        raise HTTPException(status_code=500, detail=error_response.message)


@router.get("/", summary="API root endpoint")
async def api_root():
    """
    Root endpoint for the RAG Agent API
    """
    return {
        "message": "RAG Agent API",
        "version": "1.0.0",
        "endpoints": {
            "health": "/health",
            "query": "/query (POST)",
            "docs": "/docs",
            "redoc": "/redoc"
        }
    }


# Additional endpoints can be added here as needed