"""
Cohere client utilities for the retrieval validation system.

This module provides utilities for connecting to Cohere and generating
embeddings following patterns from main.py.
"""
import os
import logging
import time
from typing import List, Optional
from cohere import Client
from .config import Config


logger = logging.getLogger(__name__)


def exponential_backoff(retries: int, base_delay: float = 1.0) -> float:
    """Calculate delay for exponential backoff"""
    import random
    return base_delay * (2 ** retries) + random.uniform(0, 1)


class CohereClientWrapper:
    """
    Wrapper class for Cohere client with validation-specific functionality.
    """

    def __init__(self):
        self.client: Optional[Client] = None
        self._initialized = False

    def initialize(self) -> bool:
        """
        Initialize the Cohere client connection.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        try:
            if not Config.validate():
                missing_vars = Config.get_missing_vars()
                logger.error(f"Missing required environment variables: {missing_vars}")
                return False

            api_key = Config.COHERE_API_KEY
            if not api_key:
                logger.error("COHERE_API_KEY environment variable is required")
                return False

            logger.info("Initializing Cohere client")
            self.client = Client(api_key=api_key)
            self._initialized = True
            return True

        except Exception as e:
            logger.error(f"Failed to initialize Cohere client: {str(e)}")
            self._initialized = False
            return False

    def is_initialized(self) -> bool:
        """
        Check if the client has been initialized.

        Returns:
            bool: True if initialized, False otherwise
        """
        return self._initialized and self.client is not None

    def embed(self, texts: List[str], model: str = None) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere API.

        Args:
            texts: List of text strings to embed
            model: Cohere model to use for embeddings

        Returns:
            List of embedding vectors (each a list of floats)
        """
        if not self.is_initialized():
            logger.error("Cohere client not initialized")
            return []

        model = model or Config.EMBEDDING_MODEL

        if not texts:
            logger.warning("Empty text list provided to embed function")
            return []

        try:
            # Cohere API has limits on batch size, so we'll process in chunks if needed
            # The maximum batch size for Cohere embeddings is typically 96
            batch_size = 96
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                logger.info(f"Generating embeddings for batch {i//batch_size + 1} of {len(texts)} texts")

                # Apply retry logic for the Cohere API call
                max_retries = 3
                retry_count = 0

                while retry_count < max_retries:
                    try:
                        response = self.client.embed(
                            texts=batch,
                            model=model,
                            input_type="search_query"  # Using search_query for query embeddings
                        )

                        batch_embeddings = [embedding for embedding in response.embeddings]
                        all_embeddings.extend(batch_embeddings)
                        break  # Success, exit retry loop

                    except Exception as e:
                        retry_count += 1
                        if retry_count >= max_retries:
                            logger.error(f"Failed to generate embeddings for batch {i//batch_size + 1} after {max_retries} attempts: {str(e)}")
                            raise e
                        else:
                            delay = exponential_backoff(retry_count - 1)
                            logger.warning(f"Attempt {retry_count} failed for batch {i//batch_size + 1}, retrying in {delay:.2f}s: {str(e)}")
                            time.sleep(delay)

            logger.info(f"Successfully generated embeddings for {len(all_embeddings)} texts")
            return all_embeddings

        except Exception as e:
            logger.error(f"Error in embed function: {str(e)}")
            return []


# Global instance for convenience
cohere_client = CohereClientWrapper()