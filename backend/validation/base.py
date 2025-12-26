"""
Base classes and interfaces for retrieval validation system.

This module defines the foundational classes and interfaces that will be
used throughout the validation system following existing patterns from main.py.
"""
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Union
from dataclasses import dataclass
from datetime import datetime


@dataclass
class ValidationResult:
    """
    Represents the result of a validation operation.
    """
    success: bool
    message: str
    details: Optional[Dict[str, Any]] = None
    timestamp: datetime = datetime.now()


class BaseValidator(ABC):
    """
    Abstract base class for all validators in the system.
    """

    @abstractmethod
    def validate(self, *args, **kwargs) -> ValidationResult:
        """
        Validate the provided data and return a result.

        Args:
            *args: Variable arguments for validation
            **kwargs: Keyword arguments for validation

        Returns:
            ValidationResult: The result of the validation
        """
        pass


class BaseService(ABC):
    """
    Abstract base class for all services in the system.
    """

    @abstractmethod
    def initialize(self) -> bool:
        """
        Initialize the service and return success status.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        pass


@dataclass
class ContentChunk:
    """
    Represents a chunk of content with metadata, following the pattern from main.py.
    """
    id: str
    text: str
    url: str
    title: str
    section: str
    ingestion_timestamp: str
    content_hash: str


@dataclass
class RetrievedResult:
    """
    Represents content returned from Qdrant similarity search for a given test query.
    """
    id: str
    text: str
    score: float  # Relevance score from similarity search (0.0 to 1.0)
    metadata: Dict[str, Any]  # Metadata from Qdrant (url, title, section, etc.)
    query_id: str  # ID of the test query that generated this result


@dataclass
class TestQuery:
    """
    Represents a predefined query used to validate retrieval relevance and quality.
    """
    id: str
    text: str
    category: Optional[str] = None  # Category or topic of the query (optional)
    expected_result_ids: Optional[List[str]] = None  # IDs of expected relevant results (optional)
    created_at: datetime = datetime.now()


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
    avg_retrieval_time: float  # Average time to retrieve results
    p95_retrieval_time: float  # 95th percentile retrieval time
    validation_throughput: float  # Queries validated per minute
    timestamp: datetime = datetime.now()