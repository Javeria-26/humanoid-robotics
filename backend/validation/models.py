"""
Data models for the retrieval validation system.

This module defines the data models for validation entities based on the
data model specification, including TestQuery, RetrievedResult, ValidationResult,
and MetadataIntegrityCheck.
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Dict, Any, Optional
from .base import TestQuery, RetrievedResult


@dataclass
class MetadataIntegrityCheck:
    """
    Specific check for the integrity and completeness of metadata in retrieved results.
    """
    result_id: str  # ID of the retrieved result being checked
    field_name: str  # Name of the metadata field being validated
    expected_value: str  # Expected value for the field
    actual_value: str  # Actual value found in the result
    is_valid: bool  # Whether the field value is valid
    error_message: str = ""  # Error message if validation failed
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class ValidationResultModel:
    """
    Summary of retrieval validation results including relevance metrics and metadata integrity checks.
    """
    id: str  # Unique identifier for this validation result
    query_id: str  # ID of the test query used
    total_results: int  # Total number of results returned
    relevant_results: int  # Number of results considered relevant
    precision: float  # Precision metric (relevant_results / total_results)
    metadata_accuracy: float  # Percentage of metadata fields that are accurate
    validation_timestamp: datetime  # When validation was performed
    retrieval_time_ms: float  # Time taken to retrieve results in milliseconds
    details: Optional[Dict[str, Any]] = None  # Additional validation details


@dataclass
class ValidationRun:
    """
    Represents a complete validation run with multiple test queries.
    """
    id: str  # Unique identifier for this validation run
    test_queries: List[TestQuery]  # List of test queries used in this run
    results: List[ValidationResultModel]  # Results for each query
    overall_metrics: Dict[str, float]  # Overall metrics for the run
    start_time: datetime  # When the validation run started
    end_time: datetime  # When the validation run ended
    completed: bool = False  # Whether the run is complete


@dataclass
class ValidationMetrics:
    """
    Represents validation metrics for a single validation run.
    """
    precision_at_k: float  # Precision at K positions (top 5, top 10, etc.)
    recall_at_k: float  # Recall at K positions
    mean_reciprocal_rank: float  # Mean reciprocal rank of relevant results
    hit_rate: float  # Percentage of queries that return at least one relevant result
    metadata_completeness: float  # Percentage of expected metadata fields present
    metadata_accuracy: float  # Percentage of metadata fields with correct values
    field_wise_accuracy: Dict[str, float]  # Accuracy per metadata field
    avg_retrieval_time: float  # Average time to retrieve results
    p95_retrieval_time: float  # 95th percentile retrieval time
    validation_throughput: float  # Queries validated per minute
    timestamp: datetime = field(default_factory=datetime.now)


@dataclass
class TestQueryResult:
    """
    Represents the results of a single test query execution.
    """
    query: TestQuery
    retrieved_results: List[RetrievedResult]
    execution_time_ms: float
    metrics: ValidationMetrics
    is_successful: bool = True
    error_message: str = ""