# Agent Context: Retrieval Pipeline Validation

## Feature Overview
- **Feature**: Retrieval Pipeline Validation for Unified Book RAG Chatbot
- **Branch**: 5-retrieval-validation
- **Objective**: Validate retrieval of embedded book content from Qdrant

## Technical Stack
- **Language**: Python 3.9+
- **Vector Database**: Qdrant Cloud
- **Embedding Service**: Cohere API (multilingual-22-12 model)
- **Existing Backend**: Located in backend/ directory

## Qdrant Configuration
- **URL**: https://625fb96d-4968-40a0-a167-39c6b286b83d.us-east4-0.gcp.cloud.qdrant.io:6333
- **Collection Name**: rag_embedding (default)
- **Vector Size**: 768 dimensions (Cohere multilingual model)
- **Distance Function**: Cosine distance

## Metadata Schema
The system stores the following metadata in Qdrant:
- `url`: Source URL of the content
- `title`: Title of the page/section
- `section`: Section identifier
- `text`: The actual content text
- `ingestion_timestamp`: When the content was ingested
- `content_hash`: Hash for duplicate detection

## Existing Implementation Patterns
- Use `query_points` method for similarity search in Qdrant
- Use Cohere's `multilingual-22-12` model for embedding generation
- Implement retry logic with exponential backoff for API calls
- Include rate limiting to respect external services
- Use content hashing for duplicate detection

## Validation Requirements
- Test queries must return relevant content with 80% precision for top 5 results
- Metadata in retrieved results must be 99% complete and accurate
- Validation process must complete within 30 minutes
- At least 95% of test queries must return results within timeout threshold
- 75% of queries should have most relevant content in top 3 positions

## Key Functions to Reference
- `sample_query_validation` in main.py: Example of query execution
- `initialize_qdrant_client` in main.py: Qdrant connection setup
- `embed` function in main.py: Embedding generation with error handling
- `generate_content_hash` in main.py: Content deduplication approach