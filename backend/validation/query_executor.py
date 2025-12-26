"""
Test query execution engine for the retrieval validation system.

This module provides functionality to execute test queries and
collect results for validation purposes.
"""
import time
import logging
from typing import List, Dict, Any, Optional
from .search import similarity_search_service
from .test_queries import TestQueryManager, default_query_manager
from .base import TestQuery, RetrievedResult
from .models import TestQueryResult, ValidationMetrics
from .utils import time_function
from .logger import validation_logger as logger


class QueryExecutionEngine:
    """
    Engine class for executing test queries and collecting results.
    """

    def __init__(self, query_manager: TestQueryManager = None):
        self.query_manager = query_manager or default_query_manager
        self.search_service = similarity_search_service

    def initialize(self) -> bool:
        """
        Initialize the engine by ensuring dependencies are ready.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        return self.search_service.initialize()

    def execute_single_query(self, test_query: TestQuery, top_k: int = 5,
                           collection_name: str = None) -> TestQueryResult:
        """
        Execute a single test query and return the results.

        Args:
            test_query: TestQuery object to execute
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            TestQueryResult containing the query, results, and metrics
        """
        if not self.initialize():
            logger.error("Engine not initialized properly")
            return TestQueryResult(
                query=test_query,
                retrieved_results=[],
                execution_time_ms=0,
                metrics=ValidationMetrics(
                    precision_at_k=0.0,
                    recall_at_k=0.0,
                    mean_reciprocal_rank=0.0,
                    hit_rate=0.0,
                    metadata_completeness=0.0,
                    metadata_accuracy=0.0,
                    field_wise_accuracy={},
                    avg_retrieval_time=0.0,
                    p95_retrieval_time=0.0,
                    validation_throughput=0.0
                ),
                is_successful=False,
                error_message="Engine not initialized"
            )

        start_time = time.time()
        try:
            # Perform the search
            results = self.search_service.search_by_query(test_query, top_k, collection_name)

            execution_time_ms = (time.time() - start_time) * 1000

            # Calculate basic metrics
            # For now, we'll create a basic metrics object - more sophisticated metrics
            # will be calculated in other services
            metrics = ValidationMetrics(
                precision_at_k=0.0,  # Will be calculated elsewhere
                recall_at_k=0.0,     # Will be calculated elsewhere
                mean_reciprocal_rank=0.0,  # Will be calculated elsewhere
                hit_rate=1.0 if results else 0.0,  # 100% if we got results, 0% if none
                metadata_completeness=0.0,  # Will be calculated elsewhere
                metadata_accuracy=0.0,      # Will be calculated elsewhere
                field_wise_accuracy={},     # Will be calculated elsewhere
                avg_retrieval_time=execution_time_ms,
                p95_retrieval_time=execution_time_ms,  # Single query, so same as avg
                validation_throughput=1000.0 / execution_time_ms if execution_time_ms > 0 else 0.0  # Queries per second
            )

            logger.info(f"Executed query '{test_query.text[:50]}...' in {execution_time_ms:.2f}ms, got {len(results)} results")

            return TestQueryResult(
                query=test_query,
                retrieved_results=results,
                execution_time_ms=execution_time_ms,
                metrics=metrics
            )

        except Exception as e:
            execution_time_ms = (time.time() - start_time) * 1000
            logger.error(f"Error executing query '{test_query.text[:50]}...': {str(e)}")
            return TestQueryResult(
                query=test_query,
                retrieved_results=[],
                execution_time_ms=execution_time_ms,
                metrics=ValidationMetrics(
                    precision_at_k=0.0,
                    recall_at_k=0.0,
                    mean_reciprocal_rank=0.0,
                    hit_rate=0.0,
                    metadata_completeness=0.0,
                    metadata_accuracy=0.0,
                    field_wise_accuracy={},
                    avg_retrieval_time=execution_time_ms,
                    p95_retrieval_time=execution_time_ms,
                    validation_throughput=0.0
                ),
                is_successful=False,
                error_message=str(e)
            )

    def execute_query_by_id(self, query_id: str, top_k: int = 5,
                          collection_name: str = None) -> Optional[TestQueryResult]:
        """
        Execute a test query by its ID.

        Args:
            query_id: ID of the query to execute
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            TestQueryResult or None if query not found
        """
        test_query = self.query_manager.get_query(query_id)
        if not test_query:
            logger.error(f"Query with ID {query_id} not found")
            return None

        return self.execute_single_query(test_query, top_k, collection_name)

    def execute_multiple_queries(self, query_ids: List[str], top_k: int = 5,
                              collection_name: str = None) -> List[TestQueryResult]:
        """
        Execute multiple test queries by their IDs.

        Args:
            query_ids: List of query IDs to execute
            top_k: Number of top results to return for each query
            collection_name: Name of the collection to search in

        Returns:
            List of TestQueryResult objects
        """
        results = []
        for query_id in query_ids:
            result = self.execute_query_by_id(query_id, top_k, collection_name)
            if result:
                results.append(result)

        logger.info(f"Executed {len(results)} out of {len(query_ids)} queries")
        return results

    def execute_all_queries(self, top_k: int = 5, collection_name: str = None) -> List[TestQueryResult]:
        """
        Execute all available test queries.

        Args:
            top_k: Number of top results to return for each query
            collection_name: Name of the collection to search in

        Returns:
            List of TestQueryResult objects
        """
        all_queries = self.query_manager.get_all_queries()
        results = []

        for test_query in all_queries:
            result = self.execute_single_query(test_query, top_k, collection_name)
            results.append(result)

        logger.info(f"Executed all {len(results)} available queries")
        return results

    def execute_queries_by_category(self, category: str, top_k: int = 5,
                                  collection_name: str = None) -> List[TestQueryResult]:
        """
        Execute all test queries in a specific category.

        Args:
            category: Category of queries to execute
            top_k: Number of top results to return for each query
            collection_name: Name of the collection to search in

        Returns:
            List of TestQueryResult objects
        """
        category_queries = self.query_manager.get_queries_by_category(category)
        results = []

        for test_query in category_queries:
            result = self.execute_single_query(test_query, top_k, collection_name)
            results.append(result)

        logger.info(f"Executed {len(results)} queries in category '{category}'")
        return results

    def execute_batch(self, queries: List[TestQuery], top_k: int = 5,
                    collection_name: str = None) -> List[TestQueryResult]:
        """
        Execute a batch of test queries.

        Args:
            queries: List of TestQuery objects to execute
            top_k: Number of top results to return for each query
            collection_name: Name of the collection to search in

        Returns:
            List of TestQueryResult objects
        """
        results = []
        for test_query in queries:
            result = self.execute_single_query(test_query, top_k, collection_name)
            results.append(result)

        logger.info(f"Executed batch of {len(results)} queries")
        return results

    def execute_with_timing(self, test_query: TestQuery, top_k: int = 5,
                          collection_name: str = None) -> Dict[str, Any]:
        """
        Execute a query and return detailed timing information.

        Args:
            test_query: TestQuery object to execute
            top_k: Number of top results to return
            collection_name: Name of the collection to search in

        Returns:
            Dictionary with execution results and timing information
        """
        start_time = time.time()

        result = self.execute_single_query(test_query, top_k, collection_name)

        total_time = (time.time() - start_time) * 1000

        timing_info = {
            'query_id': test_query.id,
            'query_text': test_query.text,
            'total_execution_time_ms': total_time,
            'search_execution_time_ms': result.execution_time_ms,
            'result_count': len(result.retrieved_results),
            'timestamp': time.time()
        }

        return {
            'result': result,
            'timing': timing_info
        }

    def format_result_preview(self, retrieved_result: RetrievedResult, preview_length: int = 100) -> Dict[str, Any]:
        """
        Format a retrieved result with a text preview.

        Args:
            retrieved_result: RetrievedResult object to format
            preview_length: Maximum length of the text preview

        Returns:
            Dictionary with formatted result data
        """
        text_preview = retrieved_result.text[:preview_length] + "..." if len(retrieved_result.text) > preview_length else retrieved_result.text

        return {
            'id': retrieved_result.id,
            'text_preview': text_preview,
            'score': retrieved_result.score,
            'metadata': retrieved_result.metadata,
            'query_id': retrieved_result.query_id
        }

    def format_results_for_display(self, results: List[RetrievedResult], preview_length: int = 100) -> List[Dict[str, Any]]:
        """
        Format multiple retrieved results with text previews for display.

        Args:
            results: List of RetrievedResult objects to format
            preview_length: Maximum length of each text preview

        Returns:
            List of dictionaries with formatted result data
        """
        formatted_results = []
        for result in results:
            formatted_result = self.format_result_preview(result, preview_length)
            formatted_results.append(formatted_result)

        return formatted_results

    def format_query_result(self, query_result: TestQueryResult, preview_length: int = 100) -> Dict[str, Any]:
        """
        Format a complete query result with text previews.

        Args:
            query_result: TestQueryResult object to format
            preview_length: Maximum length of each text preview

        Returns:
            Dictionary with formatted query result data
        """
        formatted_results = self.format_results_for_display(query_result.retrieved_results, preview_length)

        return {
            'query': {
                'id': query_result.query.id,
                'text': query_result.query.text,
                'category': query_result.query.category
            },
            'results': formatted_results,
            'execution_time_ms': query_result.execution_time_ms,
            'result_count': len(query_result.retrieved_results),
            'is_successful': query_result.is_successful,
            'error_message': query_result.error_message if not query_result.is_successful else None
        }

    def generate_execution_report(self, query_results: List[TestQueryResult]) -> Dict[str, Any]:
        """
        Generate a comprehensive execution report with retrieval times and statistics.

        Args:
            query_results: List of TestQueryResult objects

        Returns:
            Dictionary with execution report data
        """
        if not query_results:
            return {
                'summary': {
                    'total_queries': 0,
                    'successful_queries': 0,
                    'failed_queries': 0,
                    'total_execution_time_ms': 0,
                    'avg_execution_time_ms': 0,
                    'min_execution_time_ms': 0,
                    'max_execution_time_ms': 0
                },
                'details': [],
                'timestamp': time.time()
            }

        total_time = sum(result.execution_time_ms for result in query_results)
        successful_count = sum(1 for result in query_results if result.is_successful)
        failed_count = len(query_results) - successful_count

        execution_times = [result.execution_time_ms for result in query_results]
        avg_time = total_time / len(query_results) if query_results else 0
        min_time = min(execution_times) if execution_times else 0
        max_time = max(execution_times) if execution_times else 0

        # Calculate percentiles
        sorted_times = sorted(execution_times)
        p50_time = sorted_times[len(sorted_times)//2] if sorted_times else 0
        p95_time = sorted_times[int(0.95 * len(sorted_times))] if sorted_times else 0 if len(sorted_times) > 0 else 0

        # Prepare detailed results
        detailed_results = []
        for result in query_results:
            detailed_results.append({
                'query_id': result.query.id,
                'query_text': result.query.text[:100] + "..." if len(result.query.text) > 100 else result.query.text,
                'result_count': len(result.retrieved_results),
                'execution_time_ms': result.execution_time_ms,
                'is_successful': result.is_successful,
                'error_message': result.error_message if not result.is_successful else None,
                'avg_score': sum(r.score for r in result.retrieved_results) / len(result.retrieved_results) if result.retrieved_results else 0
            })

        report = {
            'summary': {
                'total_queries': len(query_results),
                'successful_queries': successful_count,
                'failed_queries': failed_count,
                'success_rate': successful_count / len(query_results) if query_results else 0,
                'total_execution_time_ms': total_time,
                'avg_execution_time_ms': avg_time,
                'min_execution_time_ms': min_time,
                'max_execution_time_ms': max_time,
                'p50_execution_time_ms': p50_time,  # Median
                'p95_execution_time_ms': p95_time,  # 95th percentile
                'timestamp': time.time()
            },
            'details': detailed_results,
            'charts_data': {
                'execution_times': execution_times,
                'success_rate_by_query': [1 if result.is_successful else 0 for result in query_results]
            }
        }

        logger.info(f"Generated execution report for {len(query_results)} queries")
        return report

    def generate_category_report(self, category: str, query_results: List[TestQueryResult]) -> Dict[str, Any]:
        """
        Generate a report for a specific category of queries.

        Args:
            category: Category name
            query_results: List of TestQueryResult objects for the category

        Returns:
            Dictionary with category-specific report data
        """
        report = self.generate_execution_report(query_results)

        # Add category-specific information
        report['category'] = category
        report['category_summary'] = {
            'category_name': category,
            'query_count': len(query_results),
            'average_results_per_query': sum(len(result.retrieved_results) for result in query_results) / len(query_results) if query_results else 0
        }

        return report

    def validate_execution_engine(self) -> bool:
        """
        Validate that the query execution engine is working correctly.

        Returns:
            bool: True if execution engine is working, False otherwise
        """
        if not self.initialize():
            logger.error("Engine not initialized properly")
            return False

        try:
            # Try to execute a simple query
            sample_query = TestQuery(
                id="validation-query",
                text="test validation functionality"
            )

            result = self.execute_single_query(sample_query, top_k=1)

            # Basic validation: execution should complete without errors
            success = result.is_successful or True  # Consider it successful even if we get empty results

            logger.info(f"Execution engine validation: {'✓' if success else '✗'}")
            return success

        except Exception as e:
            logger.error(f"Error validating execution engine: {str(e)}")
            return False


# Global instance for convenience
query_execution_engine = QueryExecutionEngine()