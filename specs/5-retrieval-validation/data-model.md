# Data Model: Retrieval Pipeline Validation

**Feature**: Retrieval Pipeline Validation
**Date**: 2025-12-25

## Core Entities

### TestQuery
Represents a predefined query used to validate retrieval relevance and quality
- `id`: string - Unique identifier for the test query
- `text`: string - The query text to be used for similarity search
- `category`: string - Category or topic of the query (optional)
- `expected_result_ids`: List[string] - IDs of expected relevant results (optional)
- `created_at`: datetime - Timestamp when the query was created

### RetrievedResult
Represents content returned from Qdrant similarity search for a given test query
- `id`: string - ID of the retrieved document from Qdrant
- `text`: string - The actual content text
- `score`: float - Relevance score from similarity search (0.0 to 1.0)
- `metadata`: Dict - Metadata from Qdrant (url, title, section, etc.)
- `query_id`: string - ID of the test query that generated this result

### ValidationResult
Summary of retrieval validation results including relevance metrics and metadata integrity checks
- `id`: string - Unique identifier for this validation result
- `query_id`: string - ID of the test query used
- `total_results`: int - Total number of results returned
- `relevant_results`: int - Number of results considered relevant
- `precision`: float - Precision metric (relevant_results / total_results)
- `metadata_accuracy`: float - Percentage of metadata fields that are accurate
- `validation_timestamp`: datetime - When validation was performed
- `retrieval_time_ms`: float - Time taken to retrieve results in milliseconds

### MetadataIntegrityCheck
Specific check for the integrity and completeness of metadata in retrieved results
- `result_id`: string - ID of the retrieved result being checked
- `field_name`: string - Name of the metadata field being validated
- `expected_value`: string - Expected value for the field
- `actual_value`: string - Actual value found in the result
- `is_valid`: bool - Whether the field value is valid
- `error_message`: string - Error message if validation failed

## Validation Metrics

### RetrievalQualityMetrics
- `precision_at_k`: float - Precision at K positions (top 5, top 10, etc.)
- `recall_at_k`: float - Recall at K positions
- `mean_reciprocal_rank`: float - Mean reciprocal rank of relevant results
- `hit_rate`: float - Percentage of queries that return at least one relevant result

### MetadataIntegrityMetrics
- `metadata_completeness`: float - Percentage of expected metadata fields present
- `metadata_accuracy`: float - Percentage of metadata fields with correct values
- `field_wise_accuracy`: Dict[string, float] - Accuracy per metadata field

### PerformanceMetrics
- `avg_retrieval_time`: float - Average time to retrieve results
- `p95_retrieval_time`: float - 95th percentile retrieval time
- `validation_throughput`: float - Queries validated per minute