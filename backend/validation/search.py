"""
Similarity search functionality for the retrieval validation system.

This module provides functionality to perform similarity searches
using Cohere embeddings and Qdrant vector storage.
"""
import logging
from typing import List, Optional, Dict
from .cohere_client import cohere_client
from .qdrant_client import qdrant_client
from .base import RetrievedResult, TestQuery
from .logger import validation_logger as logger


class SimilaritySearchService:
    """
    Service class for performing similarity searches using Cohere embeddings and Qdrant.
    """

    def __init__(self):
        self.cohere_client = cohere_client
        self.qdrant_client = qdrant_client

    def initialize(self) -> bool:
        """
        Initialize the service by ensuring Cohere and Qdrant clients are ready.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        cohere_ok = self.cohere_client.is_initialized() or self.cohere_client.initialize()
        qdrant_ok = self.qdrant_client.is_initialized() or self.qdrant_client.initialize()
        return cohere_ok and qdrant_ok

    def search_by_text(self, query_text: str, top_k: int = 5, collection_name: str = None) -> List[RetrievedResult]:
        """
        Perform similarity search using text query.

        Args:
            query_text: Text to search for similar content
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            List of RetrievedResult objects
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return []

        try:
            # Generate embedding for the query text
            query_embedding = self.cohere_client.embed([query_text])

            if not query_embedding or len(query_embedding) == 0:
                logger.error("Failed to generate embedding for query text")
                return []

            query_vector = query_embedding[0]

            # Perform the search in Qdrant
            results = self.qdrant_client.query_points(
                query_vector=query_vector,
                top_k=top_k,
                collection_name=collection_name
            )

            # Set the query ID for each result
            for result in results:
                result.query_id = "unknown"  # Will be set by caller if known

            logger.info(f"Search for '{query_text[:50]}...' returned {len(results)} results")
            return results

        except Exception as e:
            logger.error(f"Error performing similarity search: {str(e)}")
            return []

    def search_by_query(self, test_query: TestQuery, top_k: int = 5, collection_name: str = None) -> List[RetrievedResult]:
        """
        Perform similarity search using a TestQuery object.

        Args:
            test_query: TestQuery object containing the query text
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            List of RetrievedResult objects
        """
        results = self.search_by_text(test_query.text, top_k, collection_name)

        # Update the query_id for each result
        for result in results:
            result.query_id = test_query.id

        return results

    def batch_search(self, queries: List[str], top_k: int = 5, collection_name: str = None) -> List[List[RetrievedResult]]:
        """
        Perform multiple similarity searches in batch.

        Args:
            queries: List of query texts to search for
            top_k: Number of top results to return for each query
            collection_name: Name of the collection to search in

        Returns:
            List of lists, where each inner list contains RetrievedResult objects for the corresponding query
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return [[] for _ in queries]

        all_results = []
        for query_text in queries:
            results = self.search_by_text(query_text, top_k, collection_name)
            all_results.append(results)

        logger.info(f"Batch search completed for {len(queries)} queries")
        return all_results

    def search_with_embedding(self, query_embedding: List[float], top_k: int = 5, collection_name: str = None) -> List[RetrievedResult]:
        """
        Perform similarity search using a pre-generated embedding vector.

        Args:
            query_embedding: Pre-generated embedding vector to search with
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            List of RetrievedResult objects
        """
        if not self.qdrant_client.is_initialized():
            logger.error("Qdrant client not initialized")
            return []

        try:
            results = self.qdrant_client.query_points(
                query_vector=query_embedding,
                top_k=top_k,
                collection_name=collection_name
            )

            logger.info(f"Search with embedding returned {len(results)} results")
            return results

        except Exception as e:
            logger.error(f"Error performing similarity search with embedding: {str(e)}")
            return []

    def validate_search_functionality(self, sample_query: str = "test search functionality") -> bool:
        """
        Validate that the search functionality is working correctly.

        Args:
            sample_query: Sample query text to test with

        Returns:
            bool: True if search functionality is working, False otherwise
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return False

        try:
            # Try a simple search
            results = self.search_by_text(sample_query, top_k=1)

            # Basic validation: should be able to generate embeddings and get results
            test_embedding = self.cohere_client.embed([sample_query])
            if not test_embedding:
                logger.error("Failed to generate embeddings for validation")
                return False

            logger.info(f"Search functionality validation: {len(results)} results returned for sample query")
            return True

        except Exception as e:
            logger.error(f"Error validating search functionality: {str(e)}")
            return False

    def search_by_category(self, category: str, top_k: int = 5, collection_name: str = None) -> List[Dict[str, List[RetrievedResult]]]:
        """
        Perform searches for all queries in a specific category.

        Args:
            category: Category of queries to search for
            top_k: Number of top results to return for each query
            collection_name: Name of the collection to search in

        Returns:
            List of dictionaries mapping query ID to its results
        """
        from .test_queries import default_query_manager

        if not self.initialize():
            logger.error("Service not initialized properly")
            return []

        category_queries = default_query_manager.get_queries_by_category(category)
        results = []

        for test_query in category_queries:
            query_results = self.search_by_query(test_query, top_k, collection_name)
            results.append({
                'query_id': test_query.id,
                'query_text': test_query.text,
                'results': query_results
            })

        logger.info(f"Category search completed for {len(category_queries)} queries in '{category}' category")
        return results


# Global instance for convenience
similarity_search_service = SimilaritySearchService()