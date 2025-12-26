"""
Qdrant vector loading service for the retrieval validation system.

This module provides functionality to load vectors and metadata from Qdrant
for validation purposes.
"""
import logging
from typing import List, Dict, Any, Optional
from ..qdrant_client import qdrant_client
from ..base import RetrievedResult
from ..base import ValidationResult
from ..logger import validation_logger as logger


class VectorLoaderService:
    """
    Service class for loading vectors and metadata from Qdrant.
    """

    def __init__(self):
        self.qdrant_client = qdrant_client

    def initialize(self) -> bool:
        """
        Initialize the service by ensuring Qdrant client is ready.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        if not self.qdrant_client.is_initialized():
            success = self.qdrant_client.initialize()
            if not success:
                logger.error("Failed to initialize Qdrant client for VectorLoaderService")
                return False
        return True

    def load_vectors_sample(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Load a sample of vectors from Qdrant for validation.

        Args:
            limit: Maximum number of vectors to load

        Returns:
            List of vector data with payload information
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return []

        try:
            points = self.qdrant_client.get_all_points(limit=limit)
            logger.info(f"Loaded {len(points)} sample vectors from Qdrant")
            return points
        except Exception as e:
            logger.error(f"Error loading vector samples from Qdrant: {str(e)}")
            return []

    def get_vector_count(self, collection_name: str = None) -> int:
        """
        Get the total count of vectors in the specified collection.

        Args:
            collection_name: Name of the collection to count vectors in

        Returns:
            Number of vectors in the collection
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return 0

        try:
            count = self.qdrant_client.get_vector_count(collection_name)
            logger.info(f"Total vectors in collection: {count}")
            return count
        except Exception as e:
            logger.error(f"Error getting vector count from Qdrant: {str(e)}")
            return 0

    def validate_collection_exists(self, collection_name: str = None) -> ValidationResult:
        """
        Validate that the specified collection exists in Qdrant.

        Args:
            collection_name: Name of the collection to check

        Returns:
            ValidationResult indicating if collection exists
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            collection_info = self.qdrant_client.get_collection_info(collection_name)
            if collection_info:
                return ValidationResult(
                    success=True,
                    message=f"Collection '{collection_name or 'default'}' exists and is accessible",
                    details=collection_info
                )
            else:
                return ValidationResult(
                    success=False,
                    message=f"Collection '{collection_name or 'default'}' does not exist"
                )
        except Exception as e:
            logger.error(f"Error checking collection existence: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error checking collection existence: {str(e)}"
            )

    def load_specific_vector(self, vector_id: str, collection_name: str = None) -> Optional[Dict[str, Any]]:
        """
        Load a specific vector by its ID from Qdrant.

        Args:
            vector_id: ID of the vector to load
            collection_name: Name of the collection to search in

        Returns:
            Vector data with payload information, or None if not found
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return None

        try:
            # Use scroll with a filter to find the specific vector
            from qdrant_client.http import models

            response = self.qdrant_client.client.scroll(
                collection_name=collection_name or self.qdrant_client.client._collection_name,
                scroll_filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="id",  # Assuming we have an id field in payload
                            match=models.MatchValue(value=vector_id)
                        )
                    ]
                ),
                limit=1
            )

            points, _ = response
            if points:
                return points[0].dict()
            else:
                # If we can't find by ID in payload, we might need to try differently
                # For now, let's try to retrieve by the point ID itself
                try:
                    records = self.qdrant_client.client.retrieve(
                        collection_name=collection_name or self.qdrant_client.client._collection_name,
                        ids=[vector_id]
                    )
                    if records:
                        return records[0].dict()
                except:
                    pass  # If this also fails, return None
                return None
        except Exception as e:
            logger.error(f"Error loading specific vector {vector_id}: {str(e)}")
            return None

    def validate_vector_access(self, collection_name: str = None) -> ValidationResult:
        """
        Validate that vectors can be accessed from the collection.

        Args:
            collection_name: Name of the collection to validate

        Returns:
            ValidationResult indicating access status
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            # Try to load a sample to verify access
            sample = self.load_vectors_sample(limit=1)
            if sample:
                return ValidationResult(
                    success=True,
                    message="Successfully accessed vectors from collection",
                    details={"sample_size": len(sample)}
                )
            else:
                # Check if collection exists but is empty
                count = self.get_vector_count(collection_name)
                if count == 0:
                    return ValidationResult(
                        success=True,  # Technically successful access, just no data
                        message="Successfully accessed collection, but it appears to be empty",
                        details={"vector_count": count}
                    )
                else:
                    return ValidationResult(
                        success=False,
                        message="Failed to access vectors from collection despite non-zero count"
                    )
        except Exception as e:
            logger.error(f"Error validating vector access: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error validating vector access: {str(e)}"
            )


# Global instance for convenience
vector_loader_service = VectorLoaderService()