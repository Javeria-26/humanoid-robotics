# Agent Context: RAG Agent & API Backend

## Feature Overview
- **Feature**: RAG-enabled AI agent using OpenAI Agents SDK and FastAPI
- **Branch**: 6-rag-agent-api
- **Objective**: Build an AI agent that answers questions using retrieved book content with citations

## Technical Stack
- **Language**: Python 3.11
- **Web Framework**: FastAPI
- **AI Agent**: OpenAI Agents SDK
- **Vector Database**: Qdrant
- **API Documentation**: Automatic with FastAPI (Swagger UI/ReDoc)
- **Backend Location**: backend/ directory

## OpenAI Agent Configuration
- **Agent Type**: Assistant API-based agent
- **Model**: gpt-4-turbo (default) or specified via environment variable
- **Tools**: Custom retrieval functions that interface with Qdrant
- **Instructions**: System prompt focused on using retrieved context for accurate responses

## API Design
- **Endpoint**: POST /query
- **Request Format**: JSON with query text and optional parameters
- **Response Format**: JSON with answer and citation information
- **Health Check**: GET /health endpoint for service monitoring

## RAG Architecture
- **Retrieval**: Query Qdrant vector database for relevant book content
- **Generation**: Use OpenAI agent with retrieved context to generate answers
- **Citation**: Include source references in responses
- **Validation**: Ensure responses are grounded in retrieved content

## Implementation Patterns
- Use Pydantic models for request/response validation
- Implement async endpoints for better performance
- Include proper error handling and logging
- Add response time tracking for performance monitoring
- Implement retry logic for external API calls

## Success Criteria Alignment
- Agent answers queries using retrieved context only (no hallucinations)
- API responds correctly to multiple queries
- Responses include proper citations to source documents
- 90% relevance accuracy in responses
- 95% of responses include proper citations when relevant content exists

## Key Components
- `rag_agent/agent.py`: OpenAI agent integration
- `rag_agent/retrieval.py`: Qdrant integration for document retrieval
- `rag_agent/api.py`: FastAPI endpoints
- `rag_agent/models.py`: Pydantic data models
- `tests/unit/`: Unit tests for individual components
- `tests/integration/`: Integration tests for the complete RAG pipeline