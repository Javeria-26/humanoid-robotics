# API Contracts: Frontend-Backend Integration

## Overview
This document defines the API contracts for the RAG chatbot system, specifying the endpoints and data formats used for frontend-backend communication.

## Base URL
`http://localhost:8000/api/v1` (development) or `https://[production-url]/api/v1` (production)

## Authentication
No authentication required for basic queries. Authentication may be added in future releases.

## Endpoints

### 1. Health Check
**Endpoint**: `GET /health`
**Description**: Check if the backend service is running and healthy
**Authentication**: None required
**Request**:
- Method: GET
- URL: `/api/v1/health`
- Headers: None required
- Body: None

**Response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T10:00:00.000000"
}
```
**Success Codes**:
- 200: Service is healthy

### 2. Query Processing
**Endpoint**: `POST /query`
**Description**: Submit a query to the RAG system and receive an AI-generated response
**Authentication**: None required
**Request**:
- Method: POST
- URL: `/api/v1/query`
- Headers:
  - `Content-Type: application/json`
- Body:
```json
{
  "query": "string, required - The user's question",
  "context": {
    "selected_text": "string, optional - Text selected by user on the page",
    "page_info": {
      "url": "string, optional - Current page URL",
      "title": "string, optional - Current page title"
    }
  },
  "include_citations": "boolean, optional - Whether to include citations (default: true)",
  "max_results": "number, optional - Max documents to retrieve (default: 5, min: 1, max: 20)"
}
```

**Response** (Success - 200):
```json
{
  "query": "string - The original query text",
  "answer": "string - The AI-generated response",
  "citations": [
    {
      "document_id": "string - Unique identifier for the document",
      "source": "string - URL or location of the source document",
      "text_snippet": "string - Excerpt from the source document",
      "title": "string, optional - Title of the source document",
      "page_number": "number, optional - Page number in the source document"
    }
  ],
  "retrieved_documents_count": "number - Number of documents used to generate response",
  "processing_time": "number - Time taken to process the query in milliseconds",
  "session_id": "string, optional - Session identifier for conversation context"
}
```

**Response** (Validation Error - 400):
```json
{
  "error": "QUERY_VALIDATION_ERROR",
  "message": "string - Error message explaining the validation issue"
}
```

**Response** (Processing Error - 500):
```json
{
  "error": "QUERY_FAILED",
  "message": "string - Error message explaining the processing issue"
}
```

**Success Codes**:
- 200: Query processed successfully
- 400: Invalid request format or validation error
- 500: Internal server error

### 3. API Root
**Endpoint**: `GET /`
**Description**: Get information about the API and available endpoints
**Authentication**: None required
**Request**:
- Method: GET
- URL: `/api/v1/`
- Headers: None required
- Body: None

**Response**:
```json
{
  "message": "RAG Agent API",
  "version": "1.0.0",
  "endpoints": {
    "health": "/health",
    "query": "/query (POST)",
    "docs": "/docs",
    "redoc": "/redoc"
  }
}
```

## Common Headers
- `Content-Type: application/json` for requests with JSON bodies
- `Accept: application/json` for JSON responses

## Error Handling
All error responses follow the same structure:
```json
{
  "error": "ERROR_CODE",
  "message": "Human-readable error message"
}
```

## Rate Limiting
The API implements rate limiting: 10 requests per minute per IP address.

## Request/Response Examples

### Example Query Request
```json
{
  "query": "What are the main components of the ROS 2 nervous system?",
  "context": {
    "selected_text": "The ROS 2 nervous system consists of nodes, topics, and services.",
    "page_info": {
      "url": "https://example.com/docs/ros2-nervous-system",
      "title": "ROS 2 Nervous System Overview"
    }
  },
  "include_citations": true,
  "max_results": 5
}
```

### Example Query Response
```json
{
  "query": "What are the main components of the ROS 2 nervous system?",
  "answer": "The main components of the ROS 2 nervous system are nodes, topics, and services. Nodes are the processes that perform computation. Topics are the named buses over which nodes exchange messages. Services provide a request/reply communication pattern.",
  "citations": [
    {
      "document_id": "doc-12345",
      "source": "https://example.com/docs/ros2-nervous-system",
      "text_snippet": "The ROS 2 nervous system consists of nodes, topics, and services...",
      "title": "ROS 2 Nervous System Overview"
    }
  ],
  "retrieved_documents_count": 1,
  "processing_time": 1250,
  "session_id": "session-abc123"
}
```