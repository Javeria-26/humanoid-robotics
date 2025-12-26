# Research Report: Retrieval Pipeline Validation

**Feature**: Retrieval Pipeline Validation
**Date**: 2025-12-25

## Decision: Qdrant Cloud Connection Parameters
**Rationale**: Based on the existing .env file in the backend directory, the system is already configured to connect to Qdrant Cloud using:
- URL: `https://625fb96d-4968-40a0-a167-39c6b286b83d.us-east4-0.gcp.cloud.qdrant.io:6333`
- API Key: Provided in environment variable
**Alternatives considered**: Local Qdrant instance (default) was available as fallback, but production configuration uses Qdrant Cloud as specified in requirements.

## Decision: Cohere API Key and Embedding Model
**Rationale**: The existing backend/main.py uses Cohere's `multilingual-22-12` model for embeddings with the API key stored in `COHERE_API_KEY` environment variable. This same model should be used for validation to ensure consistency between ingestion and retrieval.
**Alternatives considered**: Other Cohere models like `embed-multilingual-v2.0`, but the existing implementation uses `multilingual-22-12` which is already configured.

## Decision: Existing Backend Structure
**Rationale**: The backend directory structure is already established with:
- `main.py` containing the ingestion pipeline
- Dependencies in `requirements.txt`
- Configuration in `.env`
- Virtual environment in `.venv`
- Python project configuration in `pyproject.toml`
The retrieval validation module should be added to this existing structure, potentially as a new module or as additional functions within main.py.
**Alternatives considered**: Creating a separate validation service, but keeping it within the same backend structure maintains consistency and reuses existing configurations.

## Decision: Format of Stored Metadata in Qdrant
**Rationale**: Based on the existing main.py implementation, metadata is stored in Qdrant with these fields:
- `url`: Source URL of the content
- `title`: Title of the page/section
- `section`: Section identifier
- `text`: The actual content text
- `ingestion_timestamp`: When the content was ingested
- `content_hash`: Hash for duplicate detection
This same metadata structure will be available for validation purposes.
**Alternatives considered**: Different metadata schemas, but the existing schema is already implemented and working.

## Decision: Similarity Search Best Practices
**Rationale**: The existing main.py already implements a `sample_query_validation` function (lines 951-1004) that demonstrates how to:
- Generate embeddings for query text using the same Cohere model
- Perform vector search in Qdrant using `query_points` method
- Retrieve results with payload (metadata)
- Validate results based on relevance
This existing pattern should be expanded for comprehensive validation.
**Alternatives considered**: Different search algorithms or relevance metrics, but the current implementation follows standard vector similarity search patterns.