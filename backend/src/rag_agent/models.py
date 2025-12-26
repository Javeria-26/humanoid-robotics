from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class QueryRequest(BaseModel):
    """
    API request object for the query endpoint
    """
    query: str = Field(..., description="The user's question (required)")
    include_citations: bool = Field(True, description="Whether to include citation information (default: true)")
    max_results: int = Field(5, ge=1, le=20, description="Maximum number of documents to retrieve (default: 5)")


class Citation(BaseModel):
    """
    Reference information that indicates which specific documents were used to generate the response
    """
    document_id: str = Field(..., description="Reference to the source document")
    source: str = Field(..., description="Human-readable source reference (book title, chapter, etc.)")
    text_snippet: str = Field(..., description="Relevant text excerpt from the source")


class QueryResponse(BaseModel):
    """
    API response object from the query endpoint
    """
    query: str = Field(..., description="The original user query")
    answer: str = Field(..., description="The AI-generated answer")
    citations: List[Citation] = Field(..., description="List of citations used to generate the answer")
    retrieved_documents_count: int = Field(..., description="Number of documents retrieved")
    processing_time: float = Field(..., description="Time taken to process the query in milliseconds")


class Query(BaseModel):
    """
    A natural language question from a user seeking information from book content
    """
    query_text: str = Field(..., description="The user's question or query text", min_length=1, max_length=1000)
    query_id: Optional[str] = Field(None, description="Unique identifier for the query")
    user_context: Optional[Dict[str, Any]] = Field(None, description="Additional context about the user's session or preferences")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the query was submitted")


class RetrievedDocument(BaseModel):
    """
    Book passages or sections identified as relevant to the user's query
    """
    document_id: str = Field(..., description="Unique identifier for the document")
    content: str = Field(..., description="The actual text content of the document")
    source: str = Field(..., description="Reference to the original book or document location")
    score: float = Field(..., description="Relevance score from the retrieval system")
    metadata: Dict[str, Any] = Field(..., description="Additional metadata about the document (book title, chapter, page, etc.)")


class GeneratedResponse(BaseModel):
    """
    The AI-generated answer that is grounded in the retrieved documents
    """
    response_text: str = Field(..., description="The generated answer text")
    query_id: str = Field(..., description="Reference to the original query")
    citations: List[Citation] = Field(..., description="List of source documents used in generation")
    confidence_score: Optional[float] = Field(None, description="Confidence level in the response")
    timestamp: datetime = Field(default_factory=datetime.now, description="When the response was generated")
    status: str = Field(..., description="Status of the generation (success, partial, error)")


class ErrorResponse(BaseModel):
    """
    Error response object
    """
    error: str = Field(..., description="Error code")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict[str, Any]] = Field(None, description="Additional error details (optional)")