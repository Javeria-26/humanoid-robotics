# Retrieval Validation Module

This module provides functionality to validate the retrieval of embedded book content from Qdrant, ensuring that queries return relevant results and metadata is accurate.

## Overview

The retrieval validation system performs comprehensive validation of the RAG (Retrieval Augmented Generation) pipeline by:

1. Connecting to Qdrant to load stored vectors and metadata
2. Executing similarity searches with predefined test queries
3. Validating the relevance of retrieved results
4. Verifying the integrity and accuracy of metadata
5. Generating comprehensive validation reports

## Architecture

The validation system is organized into several key components:

- **Core Services**: `vector_loader`, `metadata_inspector`, `query_executor`
- **Validation Logic**: `relevance_scoring`, `metadata_validator`, `validation_analyzer`
- **API Layer**: FastAPI endpoints in `api/` directory
- **Persistence**: Storage and retrieval of validation runs
- **Utilities**: Configuration, logging, and helper functions

## Usage

### Running Validation via CLI

```bash
# Validate connection to Qdrant
python -m backend.validation.cli connection

# Validate vector access
python -m backend.validation.cli vectors

# Validate metadata quality
python -m backend.validation.cli metadata

# Generate validation report
python -m backend.validation.cli report

# Run all validations
python -m backend.validation.cli all
```

### Using the API

Start the API server:
```bash
uvicorn backend.validation.api.app:app --reload --port 8000
```

API endpoints:
- `POST /validation/run` - Trigger a comprehensive validation run
- `GET /validation/{validation_id}` - Get status/results of a validation run
- `POST /validation/test-query` - Test a single query
- `GET /validation/metrics` - Get overall validation metrics
- `GET /health` - Check API health

### Programmatic Usage

```python
from backend.validation.query_executor import query_execution_engine
from backend.validation.test_queries import TestQuery

# Create a test query
test_query = TestQuery(
    id="test-1",
    text="What is humanoid robotics?"
)

# Execute the query
result = query_execution_engine.execute_single_query(test_query, top_k=5)

# Print results
for retrieved in result.retrieved_results:
    print(f"Score: {retrieved.score}, Text preview: {retrieved.text[:100]}...")
```

## Configuration

The validation module uses the same configuration as the ingestion system:

- `QDRANT_URL`: Qdrant Cloud instance URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `COHERE_API_KEY`: Cohere API key
- `COLLECTION_NAME`: Qdrant collection name (default: "rag_embedding")
- `DEFAULT_TOP_K`: Default number of results to retrieve (default: 5)
- `DEFAULT_TIMEOUT`: Default timeout for API calls (default: 30 seconds)

## Success Criteria

The validation system evaluates the following success criteria:

1. **SC-001**: Test queries return relevant content with at least 80% precision for the top 5 results
2. **SC-002**: Metadata in retrieved results is 99% complete and accurate across all stored fields
3. **SC-003**: Retrieval validation is repeatable with consistent results across multiple validation runs
4. **SC-004**: The validation process completes within 30 minutes for a medium-sized vector collection
5. **SC-005**: At least 95% of test queries return results within the configured timeout threshold
6. **SC-006**: Relevance ranking places the most semantically similar content in the top 3 positions for 75% of queries

## Modules

### Core Services
- `vector_loader.py`: Loads vectors from Qdrant for validation
- `metadata_inspector.py`: Inspects and validates metadata integrity
- `query_executor.py`: Executes test queries and manages results

### Validation Logic
- `relevance_scoring.py`: Calculates relevance metrics (precision, recall, MRR, etc.)
- `metadata_validator.py`: Validates metadata accuracy and completeness
- `validation_analyzer.py`: Analyzes validation results and generates insights

### API Layer
- `api/app.py`: Main FastAPI application
- `api/routes/validation.py`: Validation API endpoints

### Utilities
- `config.py`: Configuration management
- `logger.py`: Logging infrastructure
- `utils.py`: Utility functions
- `persistence.py`: Validation result storage
- `metrics.py`: Metrics aggregation
- `comprehensive_report.py`: Detailed reporting

## Environment Setup

Create a `.env` file in the backend directory:

```env
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
COLLECTION_NAME=rag_embedding
```

## Running the Validation System

1. Ensure all dependencies are installed:
   ```bash
   pip install -r requirements.txt
   ```

2. Set up environment variables as shown above

3. Run validation using CLI, API, or programmatically as described in the usage section

## Performance Considerations

- The validation system is designed to handle medium-sized vector collections efficiently
- Results are cached where appropriate to avoid redundant API calls
- Error handling and retry logic is implemented for robust operation
- The system tracks performance metrics to identify bottlenecks