import os
from typing import List, Optional
from .models import RetrievedDocument, GeneratedResponse, Citation
from .retrieval import retrieval_service
from dotenv import load_dotenv
from datetime import datetime
import logging
import re

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)


class RAGAgent:
    """
    RAG (Retrieval-Augmented Generation) Agent that generates responses
    based on retrieved documents from Qdrant
    """

    def __init__(self):
        # Using a simple rule-based response generator instead of an external LLM API
        # This allows the system to work without requiring API keys
        self.model = "local-rules-based"
        self.temperature = float(os.getenv("TEMPERATURE", "0.3"))
        self.max_tokens = int(os.getenv("MAX_TOKENS", "1000"))

    def generate_response(self, query: str, retrieved_docs: List[RetrievedDocument],
                         include_citations: bool = True) -> GeneratedResponse:
        """
        Generate a response based on the query and retrieved documents using a rule-based approach
        """
        try:
            # Create citations from retrieved documents
            citations = []
            if include_citations and retrieved_docs:
                for doc in retrieved_docs:
                    citation = Citation(
                        document_id=doc.document_id,
                        source=doc.source,
                        text_snippet=doc.content[:200] + "..." if len(doc.content) > 200 else doc.content
                    )
                    citations.append(citation)

            # Generate response based on the query and context
            if not retrieved_docs:
                answer = "No relevant documents were found to answer your question. The information might not be available in the current knowledge base."
            else:
                # Simple approach: extract relevant information from the most relevant document
                most_relevant_doc = retrieved_docs[0]  # First document is most relevant due to retrieval scoring
                doc_content = most_relevant_doc.content

                # Try to find answer in the document content
                query_lower = query.lower()
                content_lower = doc_content.lower()

                # Look for direct matches of query terms in the document
                if any(term in content_lower for term in query_lower.split() if len(term) > 3):
                    # Extract relevant sentences from the document that might answer the query
                    sentences = doc_content.split('.')
                    relevant_sentences = []

                    for sentence in sentences:
                        if any(term in sentence.lower() for term in query_lower.split() if len(term) > 2):
                            relevant_sentences.append(sentence.strip())

                    if relevant_sentences:
                        answer = ' '.join(relevant_sentences[:3])  # Take up to 3 relevant sentences
                    else:
                        answer = f"Based on the retrieved documents, here's what I found: {doc_content[:500]}..."
                else:
                    answer = f"Based on the retrieved documents: {doc_content[:500]}..."

            # Create and return the generated response
            generated_response = GeneratedResponse(
                response_text=answer,
                query_id="",  # Use empty string instead of None
                citations=citations,
                confidence_score=None,  # Not implemented in this version
                status="success"
            )

            return generated_response

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            # Return a default error response
            return GeneratedResponse(
                response_text="Sorry, I encountered an error while processing your request.",
                query_id="",
                citations=[],
                confidence_score=0.0,
                status="error"
            )

    def generate_response_with_retry(self, query: str, retrieved_docs: List[RetrievedDocument],
                                   include_citations: bool = True, max_retries: int = 3) -> GeneratedResponse:
        """
        Generate a response with retry logic for failed attempts
        """
        last_exception = None

        for attempt in range(max_retries):
            try:
                # Generate the response
                generated_response = self.generate_response(query, retrieved_docs, include_citations)

                # Validate that the response is grounded in the context
                is_valid = self.validate_response_against_context(generated_response.response_text, retrieved_docs)

                # Calculate confidence score
                confidence_score = self.calculate_confidence_score(generated_response.response_text, retrieved_docs)
                generated_response.confidence_score = confidence_score

                if not is_valid:
                    # If not properly grounded, update the status
                    generated_response.status = "partial"
                else:
                    # If valid, return immediately
                    return generated_response

                # If we reach here, the response is valid but we'll continue to see if we can improve
                return generated_response

            except Exception as e:
                last_exception = e
                if attempt == max_retries - 1:  # Last attempt
                    # If all retries failed, raise the last exception
                    raise last_exception
                # Otherwise, continue to next attempt
                import time
                time.sleep(0.5 * (attempt + 1))  # Exponential backoff

        # This should not be reached, but included for completeness
        raise last_exception

    def _format_context(self, retrieved_docs: List[RetrievedDocument]) -> str:
        """
        Format the retrieved documents into a context string
        """
        context_parts = []
        for i, doc in enumerate(retrieved_docs):
            context_parts.append(f"Document {i+1}:")
            context_parts.append(f"Source: {doc.source}")
            context_parts.append(f"Content: {doc.content}")
            context_parts.append("---")

        return "\n".join(context_parts)

    def _create_prompt(self, query: str, context: str) -> str:
        """
        Create the prompt for the response generation
        """
        prompt = f"""
        Context:
        {context}

        Question: {query}

        Please provide a detailed answer based only on the provided context. If the answer is not available in the context, clearly state that the information is not available in the provided documents.

        If you provide citations, make sure to reference the specific documents from the context.
        """
        return prompt.strip()

    def detect_hallucinations(self, response: str, retrieved_docs: List[RetrievedDocument]) -> tuple[bool, str]:
        """
        Detect hallucinations in the response by comparing with retrieved context
        Returns (is_hallucinated, explanation)
        """
        if not retrieved_docs:
            return True, "No supporting documents found for response"

        response_lower = response.lower()
        response_sentences = [s.strip() for s in response.split('.') if s.strip()]

        hallucinated_sentences = []
        supported_sentences = []

        for sentence in response_sentences:
            sentence_lower = sentence.lower()
            sentence_words = set(sentence_lower.split())

            # Check if this sentence is supported by any of the documents
            is_supported = False
            for doc in retrieved_docs:
                if len(doc.content) > 20:  # Only check longer text snippets
                    doc_lower = doc.content.lower()
                    doc_words = set(doc_lower.split())

                    # Check for overlap between sentence and document content
                    overlap = len(sentence_words & doc_words)
                    if overlap > 2:  # At least 2 common words
                        is_supported = True
                        supported_sentences.append(sentence)
                        break

            if not is_supported and sentence.strip():
                hallucinated_sentences.append(sentence)

        # Calculate hallucination rate
        total_sentences = len(response_sentences)
        hallucinated_count = len(hallucinated_sentences)

        if total_sentences > 0:
            hallucination_rate = hallucinated_count / total_sentences
            if hallucination_rate > 0.5:  # More than 50% hallucinated
                return True, f"High hallucination rate: {hallucination_rate:.2%} of sentences not supported by context"

        return hallucinated_count > 0, f"{hallucinated_count} out of {total_sentences} sentences may be hallucinated"

    def validate_response_against_context(self, response: str, retrieved_docs: List[RetrievedDocument]) -> bool:
        """
        Validate that the response is grounded in the retrieved context
        This is a basic implementation - more sophisticated validation could be implemented
        """
        is_hallucinated, _ = self.detect_hallucinations(response, retrieved_docs)
        return not is_hallucinated

    def calculate_confidence_score(self, response: str, retrieved_docs: List[RetrievedDocument]) -> float:
        """
        Calculate a confidence score based on how well the response aligns with retrieved documents
        Returns a score between 0.0 and 1.0
        """
        if not retrieved_docs:
            return 0.0  # No confidence if no documents

        # Use the hallucination detection to help calculate confidence
        is_hallucinated, explanation = self.detect_hallucinations(response, retrieved_docs)

        if is_hallucinated:
            # If hallucinated, calculate based on overlap
            response_lower = response.lower()
            response_words = set(response_lower.split())

            max_overlap = 0
            for doc in retrieved_docs:
                if len(doc.content) > 20:
                    doc_lower = doc.content.lower()
                    doc_words = set(doc_lower.split())
                    overlap = len(response_words & doc_words)
                    overlap_ratio = overlap / max(len(response_words), 1)
                    max_overlap = max(max_overlap, overlap_ratio)

            # Scale confidence based on overlap (0.0 to 0.5 for hallucinated responses)
            return min(0.5, max_overlap)
        else:
            # For non-hallucinated responses, confidence is higher
            # Calculate based on average relevance score of documents used
            total_relevance = 0
            valid_docs = 0

            for doc in retrieved_docs:
                if doc.score is not None:
                    total_relevance += doc.score
                    valid_docs += 1

            avg_relevance = total_relevance / valid_docs if valid_docs > 0 else 0.5
            # Boost confidence for non-hallucinated responses
            return min(1.0, avg_relevance + 0.3)

    def generate_and_validate_response(self, query: str, retrieved_docs: List[RetrievedDocument],
                                    include_citations: bool = True) -> GeneratedResponse:
        """
        Generate a response and validate that it's grounded in the retrieved context
        """
        # Generate the response
        generated_response = self.generate_response(query, retrieved_docs, include_citations)

        # Validate that the response is grounded in the context
        is_valid = self.validate_response_against_context(generated_response.response_text, retrieved_docs)

        # Calculate confidence score
        confidence_score = self.calculate_confidence_score(generated_response.response_text, retrieved_docs)
        generated_response.confidence_score = confidence_score

        if not is_valid:
            # If not properly grounded, update the status
            generated_response.status = "partial"
            # Optionally, we could regenerate or return a different response
            # For now, we'll just update the status to indicate the issue

        return generated_response

    def generate_response_with_retry(self, query: str, retrieved_docs: List[RetrievedDocument],
                                   include_citations: bool = True, max_retries: int = 3) -> GeneratedResponse:
        """
        Generate a response with retry logic for failed attempts
        """
        # For the local rule-based implementation, we don't need complex retry logic
        # since it doesn't involve external API calls that might fail
        return self.generate_response(query, retrieved_docs, include_citations)


# Global instance of the RAG agent
rag_agent = RAGAgent()