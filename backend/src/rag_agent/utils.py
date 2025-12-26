import logging
from datetime import datetime
from typing import List, Dict, Any
from .models import Citation, RetrievedDocument
import time


def setup_logging():
    """
    Configure logging for the RAG agent application
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('rag_agent.log')
        ]
    )


def create_citation_format(documents: List[RetrievedDocument]) -> List[Citation]:
    """
    Create citation objects from retrieved documents
    """
    citations = []
    for doc in documents:
        citation = Citation(
            document_id=doc.document_id,
            source=doc.source,
            text_snippet=doc.content[:200] + "..." if len(doc.content) > 200 else doc.content
        )
        citations.append(citation)
    return citations


def measure_time(func):
    """
    Decorator to measure execution time of functions
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
        return result, execution_time
    return wrapper


def validate_query_text(query: str) -> bool:
    """
    Validate query text according to requirements
    """
    if not query or len(query.strip()) == 0:
        return False

    # Check if query is within the required length (1-1000 characters)
    if len(query) < 1 or len(query) > 1000:
        return False

    return True


def format_response_for_api(query: str, answer: str, citations: List[Citation],
                          retrieved_docs_count: int, processing_time: float):
    """
    Format the response according to the API specification
    """
    from .models import QueryResponse

    return QueryResponse(
        query=query,
        answer=answer,
        citations=citations,
        retrieved_documents_count=retrieved_docs_count,
        processing_time=processing_time
    )


def detect_hallucinations(answer: str, context: List[RetrievedDocument]) -> bool:
    """
    Basic hallucination detection - check if the answer contains information not present in the context
    This is a simplified implementation
    """
    answer_lower = answer.lower()

    # Check for any claims in the answer that aren't supported by the context
    for doc in context:
        if len(doc.content) > 20:  # Only check longer text snippets
            doc_content_lower = doc.content.lower()
            # Simple check: if there's no overlap in key terms, it might be a hallucination
            answer_words = set(answer_lower.split())
            context_words = set(doc_content_lower.split())

            # If there's minimal overlap, flag as potential hallucination
            if len(answer_words & context_words) < max(3, len(answer_words) * 0.1):
                return True

    return False


def sanitize_input(text: str) -> str:
    """
    Sanitize input text to prevent injection attacks
    """
    # Remove potentially dangerous characters
    sanitized = text.replace('\0', '')  # Remove null bytes
    return sanitized


# Initialize logging when module is loaded
setup_logging()