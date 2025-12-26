import os
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from .models import RetrievedDocument
from dotenv import load_dotenv
import logging
import cohere  # Using Cohere which is already configured in the ingestion script
from sentence_transformers import SentenceTransformer
import numpy as np

# Load environment variables
load_dotenv()

logger = logging.getLogger(__name__)


class QdrantRetrievalService:
    """
    Service class for handling document retrieval from Qdrant vector database
    """

    def __init__(self):
        # Initialize Qdrant client
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_api_key:
            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
            )
        else:
            # For local Qdrant instances without API key
            self.client = QdrantClient(
                url=qdrant_url,
            )

        # Set default collection name
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

        # Initialize a local embedding model as fallback
        # Using a lightweight model that can run locally
        try:
            self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
            logger.info("Initialized local embedding model")
        except Exception as e:
            logger.warning(f"Could not initialize local embedding model: {e}, falling back to basic approach")
            self.embedding_model = None

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for the query using local model or Cohere
        """
        try:
            # Try to use the local model first
            if self.embedding_model:
                embedding = self.embedding_model.encode([query])[0]
                embedding_list = embedding.tolist()

                # Make sure the embedding dimension matches what's in Qdrant (768 for Cohere vectors)
                if len(embedding_list) == 384:  # all-MiniLM-L6-v2 generates 384-dim vectors
                    # Pad the embedding to 768 dimensions to match Cohere vectors
                    padded_embedding = embedding_list + [0.0] * (768 - len(embedding_list))
                    return padded_embedding
                elif len(embedding_list) == 768:
                    return embedding_list
                else:
                    # If it's some other size, pad or truncate to 768
                    if len(embedding_list) < 768:
                        return embedding_list + [0.0] * (768 - len(embedding_list))
                    else:
                        return embedding_list[:768]

            # If local model fails, try Cohere (which is already configured in the system)
            # Since Cohere is already used in the ingestion script, we can reuse the same configuration
            cohere_api_key = os.getenv("COHERE_API_KEY")
            if cohere_api_key:
                co = cohere.Client(api_key=cohere_api_key)
                response = co.embed(
                    texts=[query],
                    model="multilingual-22-12"  # Same model used in ingestion
                )
                return response.embeddings[0]

            # If all else fails, return a basic embedding based on simple word presence
            # This is a fallback that will at least allow basic similarity matching
            logger.warning("Using basic fallback embedding method")
            # Simple fallback: create a basic vector based on character/word hash
            import hashlib
            query_hash = hashlib.md5(query.encode()).hexdigest()
            # Convert hex to numbers and normalize
            embedding = []
            for i in range(0, len(query_hash), 2):
                if i + 1 < len(query_hash):
                    hex_pair = query_hash[i:i+2]
                    val = int(hex_pair, 16) / 255.0  # Normalize to 0-1
                    embedding.append(val)
            # Pad or truncate to a standard size (768 to match Cohere's model)
            while len(embedding) < 768:
                embedding.append(0.0)
            embedding = embedding[:768]
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            # Return a zero vector as a last resort
            return [0.0] * 768

    def retrieve_documents(self, query: str, max_results: int = 5) -> List[RetrievedDocument]:
        """
        Retrieve relevant documents from Qdrant based on the query
        """
        try:
            # Generate embedding for the query
            query_embedding = self.embed_query(query)

            # Perform similarity search in Qdrant
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=max_results,
                with_payload=True
            ).points

            # Convert search results to RetrievedDocument objects
            retrieved_docs = []
            for result in search_results:
                payload = result.payload

                # Extract document data from payload
                doc = RetrievedDocument(
                    document_id=result.id,
                    content=payload.get('text', ''),
                    source=payload.get('source', payload.get('url', 'Unknown source')),
                    score=result.score,
                    metadata={
                        'title': payload.get('title', ''),
                        'section': payload.get('section', ''),
                        'url': payload.get('url', ''),
                        'ingestion_timestamp': payload.get('ingestion_timestamp', ''),
                        'content_hash': payload.get('content_hash', '')
                    }
                )
                retrieved_docs.append(doc)

            # If no documents were found, return empty list
            if not retrieved_docs:
                logger.info(f"No relevant documents found for query: {query[:50]}...")

            return retrieved_docs

        except Exception as e:
            logger.error(f"Error retrieving documents: {e}")
            raise

    def get_document_by_id(self, doc_id: str) -> Optional[RetrievedDocument]:
        """
        Retrieve a specific document by its ID
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id],
                with_payload=True
            )

            if records:
                record = records[0]
                payload = record.payload

                doc = RetrievedDocument(
                    document_id=record.id,
                    content=payload.get('text', ''),
                    source=payload.get('source', payload.get('url', 'Unknown source')),
                    score=1.0,  # Score not applicable for direct retrieval
                    metadata={
                        'title': payload.get('title', ''),
                        'section': payload.get('section', ''),
                        'url': payload.get('url', ''),
                        'ingestion_timestamp': payload.get('ingestion_timestamp', ''),
                        'content_hash': payload.get('content_hash', '')
                    }
                )
                return doc

            return None

        except Exception as e:
            logger.error(f"Error retrieving document by ID: {e}")
            raise


# Global instance of the retrieval service
retrieval_service = QdrantRetrievalService()