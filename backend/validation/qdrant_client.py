"""
Qdrant client utilities for the retrieval validation system.

This module provides utilities for connecting to Qdrant and performing
vector operations following patterns from main.py.
"""
import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
from .config import Config
from .base import RetrievedResult, ContentChunk


logger = logging.getLogger(__name__)


class QdrantClientWrapper:
    """
    Wrapper class for Qdrant client with validation-specific functionality.
    """

    def __init__(self):
        self.client: Optional[QdrantClient] = None
        self._initialized = False

    def initialize(self) -> bool:
        """
        Initialize the Qdrant client connection.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            if not Config.validate():
                missing_vars = Config.get_missing_vars()
                logger.error(f"Missing required environment variables: {missing_vars}")
                return False

            # Initialize Qdrant client
            qdrant_url = Config.QDRANT_URL
            qdrant_api_key = Config.QDRANT_API_KEY

            logger.info(f"Initializing Qdrant client with URL: {qdrant_url}")

            if qdrant_api_key:
                self.client = QdrantClient(
                    url=qdrant_url,
                    api_key=qdrant_api_key,
                    timeout=Config.DEFAULT_TIMEOUT
                )
            else:
                self.client = QdrantClient(
                    url=qdrant_url,
                    timeout=Config.DEFAULT_TIMEOUT
                )

            # Test the connection
            self.client.get_collections()
            logger.info("Successfully connected to Qdrant")
            self._initialized = True
            return True

        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            self._initialized = False
            return False

    def is_initialized(self) -> bool:
        """
        Check if the client has been initialized.

        Returns:
            bool: True if initialized, False otherwise
        """
        return self._initialized and self.client is not None

    def get_vector_count(self, collection_name: str = None) -> int:
        """
        Get the count of vectors in the specified collection.

        Args:
            collection_name: Name of the collection to count vectors in

        Returns:
            int: Number of vectors in the collection
        """
        if not self.is_initialized():
            logger.error("Qdrant client not initialized")
            return 0

        try:
            collection_name = collection_name or Config.COLLECTION_NAME
            count = self.client.count(
                collection_name=collection_name
            )
            return count.count
        except Exception as e:
            logger.error(f"Error getting vector count: {str(e)}")
            return 0

    def get_all_points(self, collection_name: str = None, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get sample points from the collection for validation.

        Args:
            collection_name: Name of the collection to get points from
            limit: Maximum number of points to return

        Returns:
            List of points with payload data
        """
        if not self.is_initialized():
            logger.error("Qdrant client not initialized")
            return []

        try:
            collection_name = collection_name or Config.COLLECTION_NAME
            response = self.client.scroll(
                collection_name=collection_name,
                limit=limit
            )
            points, _ = response
            return [point.dict() for point in points]
        except Exception as e:
            logger.error(f"Error getting points from Qdrant: {str(e)}")
            return []

    def query_points(self, query_vector: List[float], top_k: int = 5, collection_name: str = None) -> List[RetrievedResult]:
        """
        Query points in Qdrant using a vector.

        Args:
            query_vector: The vector to search for similar vectors
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            List of RetrievedResult objects
        """
        if not self.is_initialized():
            logger.error("Qdrant client not initialized")
            return []

        try:
            collection_name = collection_name or Config.COLLECTION_NAME
            search_results = self.client.query_points(
                collection_name=collection_name,
                query=query_vector,
                limit=top_k,
                with_payload=True
            ).points

            results = []
            for result in search_results:
                retrieved_result = RetrievedResult(
                    id=result.id,
                    text=result.payload.get('text', ''),
                    score=result.score,
                    metadata=result.payload,
                    query_id='unknown'  # Will be set by caller
                )
                results.append(retrieved_result)

            return results

        except Exception as e:
            logger.error(f"Error querying points from Qdrant: {str(e)}")
            return []

    def get_collection_info(self, collection_name: str = None) -> Optional[Dict[str, Any]]:
        """
        Get information about a collection.

        Args:
            collection_name: Name of the collection to get info for

        Returns:
            Dictionary with collection information
        """
        if not self.is_initialized():
            logger.error("Qdrant client not initialized")
            return None

        try:
            collection_name = collection_name or Config.COLLECTION_NAME
            collection_info = self.client.get_collection(collection_name=collection_name)
            return collection_info.dict()
        except Exception as e:
            logger.error(f"Error getting collection info: {str(e)}")
            return None

    def validate_connection(self) -> bool:
        """
        Validate the connection to Qdrant by testing basic operations.

        Returns:
            bool: True if connection is valid, False otherwise
        """
        if not self.is_initialized():
            logger.error("Qdrant client not initialized")
            return False

        try:
            # Test basic connection by getting collections
            collections = self.client.get_collections()
            logger.info(f"Successfully validated Qdrant connection, found {len(collections.collections)} collections")
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant connection: {str(e)}")
            return False


# Global instance for convenience
qdrant_client = QdrantClientWrapper()