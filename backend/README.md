# Document Ingestion System

This system crawls Docusaurus book content, processes it, generates embeddings using Cohere, and stores them in Qdrant for semantic search and retrieval.

## Prerequisites

- Python 3.11+
- `uv` package manager (optional, but recommended)
- Cohere API key
- Qdrant instance (local or cloud)

## Setup

1. **Install dependencies**:

```bash
pip install -r requirements.txt
```

Or if using `uv`:
```bash
uv pip install -r requirements.txt
```

2. **Set up environment variables**:

Create a `.env` file in the backend directory with the following variables:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here  # if using Qdrant Cloud
TARGET_URL=https://your-docusaurus-site.com
COLLECTION_NAME=rag_embedding
```

## Usage

### Run the Complete Ingestion Pipeline

```bash
cd backend
python main.py
```

### Run Specific Tests

The system includes several test functions that can be run independently:

```bash
# Run Python and call specific test functions
python -c "from main import test_url_extraction; test_url_extraction()"
python -c "from main import test_content_cleaning_and_chunking; test_content_cleaning_and_chunking()"
python -c "from main import test_embedding_and_storage; test_embedding_and_storage()"
python -c "from main import sample_query_validation; sample_query_validation()"
python -c "from main import test_edge_cases; test_edge_cases()"
python -c "from main import measure_ingestion_performance; measure_ingestion_performance()"
```

## Configuration

### Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_URL`: URL to your Qdrant instance (defaults to localhost if not set)
- `QDRANT_API_KEY`: API key for Qdrant (optional if using local instance)
- `TARGET_URL`: The Docusaurus site URL to crawl (defaults to humanoid-robotics-dun.vercel.app)
- `COLLECTION_NAME`: Name of the Qdrant collection to store embeddings (defaults to rag_embedding)

### Chunking Configuration

The system chunks text with these parameters:
- Maximum words per chunk: 800 (configurable in `chunk_text` function)
- Minimum words per chunk: 100 (configurable in `chunk_text` function)

## Architecture

The system consists of the following main components:

1. **URL Discovery**: Discovers all accessible pages from the target Docusaurus site
2. **Content Extraction**: Extracts clean text content from each page
3. **Content Processing**: Cleans and chunks the extracted text
4. **Embedding Generation**: Creates vector embeddings using Cohere API
5. **Storage**: Stores embeddings and metadata in Qdrant
6. **Duplicate Detection**: Prevents reprocessing unchanged content

## Features

- **Rate Limiting**: Respects server rate limits to avoid overloading
- **Retry Logic**: Handles network failures and API errors with exponential backoff
- **Duplicate Detection**: Avoids reprocessing unchanged content
- **Progress Tracking**: Detailed logging of the ingestion process
- **Error Handling**: Comprehensive error handling throughout the pipeline
- **Validation**: Ensures chunks meet size requirements and content integrity

## API Functions

### Core Functions

- `get_all_urls(base_url)`: Discover all accessible URLs from a site
- `extract_text_from_urls(urls)`: Extract clean text content from a list of URLs
- `chunk_text(text, max_words, min_words)`: Split text into appropriately sized chunks
- `embed(texts, model)`: Generate embeddings for a list of texts using Cohere
- `save_chunks_to_qdrant(client, chunks, embeddings)`: Save content and embeddings to Qdrant

### Utility Functions

- `test_url_extraction()`: Test URL discovery functionality
- `test_content_cleaning_and_chunking()`: Test text processing functionality
- `test_embedding_and_storage()`: Test embedding and storage functionality
- `sample_query_validation()`: Test retrieval from Qdrant
- `measure_ingestion_performance()`: Measure ingestion speed and performance

## Troubleshooting

### Common Issues

1. **Rate Limiting**: If you encounter rate limiting issues, the system includes built-in rate limiting, but you may need to adjust the minimum request interval in the `respect_rate_limit` function.

2. **API Keys**: Ensure your Cohere and Qdrant API keys are correctly set in the environment variables.

3. **Network Issues**: The system includes retry logic, but if you consistently face network issues, you may need to adjust the retry parameters.

4. **Memory Usage**: For large sites, the system may use significant memory. Consider processing in smaller batches if memory is a constraint.

## Success Criteria

The system meets the following success criteria:

- All pages from the target Docusaurus site are ingested successfully with 99% success rate
- Content is chunked consistently with average chunk size between 500-1000 words
- Embeddings are generated and stored with metadata in Qdrant with 99% success rate
- The ingestion pipeline is re-runnable and can detect and skip previously processed content
- Sample queries return relevant vectors with precision of at least 80% for similar content
- The entire ingestion process completes within 2 hours for a medium-sized book (100+ pages)

## Validation Module

The system includes a comprehensive validation module to verify retrieval quality and metadata integrity:

### Features
- **Vector Loading Validation**: Verify that vectors and metadata can be loaded from Qdrant
- **Similarity Search Validation**: Test retrieval with predefined test queries
- **Relevance Assessment**: Calculate precision, recall, and mean reciprocal rank metrics
- **Metadata Integrity Checks**: Validate completeness and accuracy of metadata fields
- **Performance Monitoring**: Track retrieval times and system throughput
- **Comprehensive Reporting**: Generate detailed validation reports

### Usage
```bash
# Validate connection to Qdrant
python -m backend.validation.cli connection

# Validate vector access and metadata
python -m backend.validation.cli vectors
python -m backend.validation.cli metadata

# Generate comprehensive validation report
python -m backend.validation.cli report --detailed

# Run all validations
python -m backend.validation.cli all

# Run validation via API (start server first)
uvicorn backend.validation.api.app:app --reload --port 8000
```

### Configuration
The validation module uses the same environment variables as the ingestion system:
- `QDRANT_URL`, `QDRANT_API_KEY`, `COHERE_API_KEY`
- Additional validation-specific settings can be configured in the validation module

### Success Criteria Validation
The validation module includes automated checks to verify that all success criteria are met:
- SC-001: Test queries return relevant content with at least 80% precision for top 5 results
- SC-002: Metadata in retrieved results is 99% complete and accurate
- SC-003: Retrieval validation is repeatable with consistent results
- SC-004: Validation process completes within 30 minutes
- SC-005: At least 95% of test queries return results within timeout threshold
- SC-006: Relevance ranking places semantically similar content in top 3 positions for 75% of queries