"""
Configuration module for retrieval validation system.

This module handles loading and validation of environment variables
for Qdrant and Cohere services.
"""
import os
from typing import Optional


class Config:
    """Configuration class for retrieval validation system."""

    # Qdrant Configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    COLLECTION_NAME: str = os.getenv("COLLECTION_NAME", "rag_embedding")

    # Cohere Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    EMBEDDING_MODEL: str = os.getenv("EMBEDDING_MODEL", "multilingual-22-12")

    # Validation Configuration
    DEFAULT_TOP_K: int = int(os.getenv("DEFAULT_TOP_K", "5"))
    DEFAULT_TIMEOUT: int = int(os.getenv("DEFAULT_TIMEOUT", "30"))

    @classmethod
    def validate(cls) -> bool:
        """
        Validate that required configuration values are present.

        Returns:
            bool: True if all required configuration is present, False otherwise
        """
        required_vars = [
            cls.QDRANT_URL,
            cls.QDRANT_API_KEY,
            cls.COHERE_API_KEY,
        ]

        return all(var for var in required_vars)

    @classmethod
    def get_missing_vars(cls) -> list:
        """
        Get a list of missing required environment variables.

        Returns:
            list: List of missing environment variable names
        """
        missing = []
        if not cls.QDRANT_URL:
            missing.append("QDRANT_URL")
        if not cls.QDRANT_API_KEY:
            missing.append("QDRANT_API_KEY")
        if not cls.COHERE_API_KEY:
            missing.append("COHERE_API_KEY")

        return missing