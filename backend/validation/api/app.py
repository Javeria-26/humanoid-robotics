"""
FastAPI application for the retrieval validation service.

This module sets up the FastAPI application with all necessary routes
and dependencies for the validation API.
"""
from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from typing import Dict, Any
import logging
import os
from ..config import Config
from ..logger import setup_logging


# Set up logging
logger = setup_logging(__name__)

# Create FastAPI app
app = FastAPI(
    title="Retrieval Validation API",
    description="API for validating retrieval of embedded book content from Qdrant",
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

# Authentication middleware
async def authenticate_request(request: Request):
    """
    Simple API key authentication middleware.
    In production, use a more robust authentication system.
    """
    api_key = request.headers.get("X-API-Key")
    expected_api_key = os.getenv("VALIDATION_API_KEY")

    if expected_api_key and api_key != expected_api_key:
        logger.warning(f"Unauthorized access attempt to {request.url.path}")
        raise HTTPException(status_code=401, detail="Invalid API key")

    # Continue with request if authentication passes or is disabled
    return

# Add middleware to all routes (except health and docs)
@app.middleware("http")
async def add_auth_middleware(request: Request, call_next):
    """
    Middleware to add authentication to protected endpoints.
    """
    # Skip authentication for health check and documentation
    if request.url.path in ["/health", "/", "/docs", "/redoc", "/openapi.json"]:
        response = await call_next(request)
        return response

    # Apply authentication to other endpoints
    await authenticate_request(request)
    response = await call_next(request)
    return response

# Import routes after app creation to avoid circular imports
from .routes import validation

# Include validation routes
app.include_router(validation.router, prefix="/validation", tags=["validation"])

# Add a health check endpoint
@app.get("/health", tags=["health"])
async def health_check():
    """
    Health check endpoint to verify API is running.
    """
    if not Config.validate():
        missing_vars = Config.get_missing_vars()
        logger.warning(f"Missing required environment variables: {missing_vars}")
        return {
            "status": "unhealthy",
            "message": f"Missing required environment variables: {missing_vars}",
            "timestamp": __import__('datetime').datetime.now().isoformat()
        }

    return {
        "status": "healthy",
        "message": "Validation API is running",
        "timestamp": __import__('datetime').datetime.now().isoformat()
    }

# Add a root endpoint
@app.get("/", tags=["root"])
async def root():
    """
    Root endpoint with API information.
    """
    return {
        "message": "Retrieval Validation API",
        "version": "1.0.0",
        "description": "API for validating retrieval of embedded book content from Qdrant",
        "docs_url": "/docs",
        "redoc_url": "/redoc",
        "health_url": "/health"
    }

# Add exception handlers
@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """
    Global exception handler for the API.
    """
    logger.error(f"Global exception: {str(exc)}", exc_info=True)
    raise HTTPException(status_code=500, detail=f"Internal server error: {str(exc)}")

# Add startup event
@app.on_event("startup")
async def startup_event():
    """
    Startup event to initialize the validation service.
    """
    logger.info("Starting up Retrieval Validation API")

    # Validate configuration on startup
    if not Config.validate():
        missing_vars = Config.get_missing_vars()
        logger.error(f"Missing required environment variables: {missing_vars}")
        raise RuntimeError(f"Missing required environment variables: {missing_vars}")

    logger.info("Validation API startup completed successfully")

# Add shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """
    Shutdown event for cleanup.
    """
    logger.info("Shutting down Retrieval Validation API")