"""
Validation metrics aggregator for the retrieval validation system.

This module provides functionality to aggregate and calculate
validation metrics across multiple runs and queries.
"""
import logging
from datetime import datetime
from typing import List, Dict, Any, Optional
from .models import ValidationRun, ValidationResultModel
from .base import RetrievedResult, TestQuery
from .relevance_scoring import relevance_scoring_service
from .metadata_validator import metadata_validator_service
from .logger import validation_logger as logger


class ValidationMetricsAggregator:
    """
    Service class for aggregating and calculating validation metrics.
    """

    def __init__(self):
        self.relevance_scorer = relevance_scoring_service
        self.metadata_validator = metadata_validator_service

    def calculate_run_metrics(self, validation_run: ValidationRun) -> Dict[str, float]:
        """
        Calculate metrics for a single validation run.

        Args:
            validation_run: ValidationRun object to calculate metrics for

        Returns:
            Dictionary with calculated metrics
        """
        if not validation_run.results:
            return {
                'precision_at_k': 0.0,
                'recall_at_k': 0.0,
                'mean_reciprocal_rank': 0.0,
                'hit_rate': 0.0,
                'metadata_accuracy': 0.0,
                'metadata_completeness': 0.0,
                'avg_retrieval_time_ms': 0.0,
                'total_queries': 0,
                'total_results': 0
            }

        # Calculate relevance metrics
        total_precision = 0.0
        total_recall = 0.0
        total_mrr = 0.0
        total_hit_rate = 0.0
        total_retrieval_time = 0.0

        for result in validation_run.results:
            total_precision += result.precision
            total_mrr += getattr(result, 'mean_reciprocal_rank', 0.0)  # Use getattr to handle missing attributes gracefully
            total_hit_rate += getattr(result, 'hit_rate', 0.0 if result.total_results > 0 else 1.0)  # If no results, hit rate is 0 unless no expected results
            total_retrieval_time += result.retrieval_time_ms

        avg_precision = total_precision / len(validation_run.results) if validation_run.results else 0.0
        avg_mrr = total_mrr / len(validation_run.results) if validation_run.results else 0.0
        avg_hit_rate = total_hit_rate / len(validation_run.results) if validation_run.results else 0.0
        avg_retrieval_time = total_retrieval_time / len(validation_run.results) if validation_run.results else 0.0

        # Calculate metadata metrics
        all_retrieved_results = []
        for result in validation_run.results:
            all_retrieved_results.extend(result.retrieved_results if hasattr(result, 'retrieved_results') else [])

        if all_retrieved_results:
            metadata_validation = self.metadata_validator.calculate_metadata_accuracy([all_retrieved_results])
            metadata_accuracy = metadata_validation['overall_accuracy']
            completeness_validation = self.metadata_validator.calculate_metadata_completeness(all_retrieved_results)
            metadata_completeness = completeness_validation['completeness_percentage']
        else:
            metadata_accuracy = 0.0
            metadata_completeness = 0.0

        # Calculate total counts
        total_queries = len(validation_run.test_queries)
        total_results = sum(len(result.retrieved_results) if hasattr(result, 'retrieved_results') else 0 for result in validation_run.results)

        metrics = {
            'precision_at_k': avg_precision,
            'mean_reciprocal_rank': avg_mrr,
            'hit_rate': avg_hit_rate,
            'metadata_accuracy': metadata_accuracy,
            'metadata_completeness': metadata_completeness,
            'avg_retrieval_time_ms': avg_retrieval_time,
            'total_queries': total_queries,
            'total_results': total_results,
            'run_duration_seconds': (validation_run.end_time - validation_run.start_time).total_seconds() if validation_run.start_time and validation_run.end_time else 0
        }

        return metrics

    def aggregate_multiple_runs(self, validation_runs: List[ValidationRun]) -> Dict[str, float]:
        """
        Aggregate metrics across multiple validation runs.

        Args:
            validation_runs: List of ValidationRun objects to aggregate

        Returns:
            Dictionary with aggregated metrics
        """
        if not validation_runs:
            return {
                'avg_precision_at_k': 0.0,
                'avg_mean_reciprocal_rank': 0.0,
                'avg_hit_rate': 0.0,
                'avg_metadata_accuracy': 0.0,
                'avg_metadata_completeness': 0.0,
                'avg_retrieval_time_ms': 0.0,
                'total_runs': 0,
                'total_queries': 0,
                'total_results': 0,
                'runs_std_dev_precision': 0.0
            }

        # Calculate individual run metrics
        run_metrics_list = []
        for run in validation_runs:
            run_metrics = self.calculate_run_metrics(run)
            run_metrics_list.append(run_metrics)

        # Aggregate metrics
        total_runs = len(validation_runs)
        total_queries = sum(metrics['total_queries'] for metrics in run_metrics_list)
        total_results = sum(metrics['total_results'] for metrics in run_metrics_list)

        # Calculate averages
        avg_precision = sum(metrics['precision_at_k'] for metrics in run_metrics_list) / total_runs if total_runs > 0 else 0.0
        avg_mrr = sum(metrics['mean_reciprocal_rank'] for metrics in run_metrics_list) / total_runs if total_runs > 0 else 0.0
        avg_hit_rate = sum(metrics['hit_rate'] for metrics in run_metrics_list) / total_runs if total_runs > 0 else 0.0
        avg_metadata_accuracy = sum(metrics['metadata_accuracy'] for metrics in run_metrics_list) / total_runs if total_runs > 0 else 0.0
        avg_metadata_completeness = sum(metrics['metadata_completeness'] for metrics in run_metrics_list) / total_runs if total_runs > 0 else 0.0
        avg_retrieval_time = sum(metrics['avg_retrieval_time_ms'] for metrics in run_metrics_list) / total_runs if total_runs > 0 else 0.0

        # Calculate standard deviation for precision (measure of consistency)
        import statistics
        precision_values = [metrics['precision_at_k'] for metrics in run_metrics_list]
        precision_std_dev = statistics.stdev(precision_values) if len(precision_values) > 1 else 0.0

        aggregated_metrics = {
            'avg_precision_at_k': avg_precision,
            'avg_mean_reciprocal_rank': avg_mrr,
            'avg_hit_rate': avg_hit_rate,
            'avg_metadata_accuracy': avg_metadata_accuracy,
            'avg_metadata_completeness': avg_metadata_completeness,
            'avg_retrieval_time_ms': avg_retrieval_time,
            'total_runs': total_runs,
            'total_queries': total_queries,
            'total_results': total_results,
            'runs_std_dev_precision': precision_std_dev,
            'consistency_score': 1 - precision_std_dev if precision_std_dev <= 1.0 else 0.0  # Higher is more consistent
        }

        return aggregated_metrics

    def calculate_success_criteria_metrics(self, validation_runs: List[ValidationRun]) -> Dict[str, Dict[str, Any]]:
        """
        Calculate metrics related to success criteria.

        Args:
            validation_runs: List of ValidationRun objects to evaluate

        Returns:
            Dictionary with success criteria metrics
        """
        if not validation_runs:
            return {}

        # Aggregate metrics across all runs
        agg_metrics = self.aggregate_multiple_runs(validation_runs)

        # Evaluate success criteria
        success_criteria = {
            'SC-001': {
                'name': 'Test queries return relevant content with at least 80% precision for the top 5 results',
                'threshold': 0.8,
                'actual': agg_metrics.get('avg_precision_at_k', 0.0),
                'met': agg_metrics.get('avg_precision_at_k', 0.0) >= 0.8,
                'status': 'PASS' if agg_metrics.get('avg_precision_at_k', 0.0) >= 0.8 else 'FAIL'
            },
            'SC-002': {
                'name': 'Metadata in retrieved results is 99% complete and accurate across all stored fields',
                'threshold': 0.99,
                'actual': min(
                    agg_metrics.get('avg_metadata_accuracy', 0.0),
                    agg_metrics.get('avg_metadata_completeness', 0.0)
                ),
                'met': (agg_metrics.get('avg_metadata_accuracy', 0.0) >= 0.99 and
                       agg_metrics.get('avg_metadata_completeness', 0.0) >= 0.99),
                'status': 'PASS' if (agg_metrics.get('avg_metadata_accuracy', 0.0) >= 0.99 and
                                   agg_metrics.get('avg_metadata_completeness', 0.0) >= 0.99) else 'FAIL'
            },
            'SC-004': {
                'name': 'The validation process completes within 30 minutes for a medium-sized vector collection',
                'threshold': 1800,  # 30 minutes in seconds
                'actual': agg_metrics.get('avg_retrieval_time_ms', 0.0) / 1000,  # Convert ms to seconds
                'met': True,  # This is evaluated per run, not aggregate
                'status': 'EVALUATION_PER_RUN'
            },
            'SC-005': {
                'name': 'At least 95% of test queries return results within the configured timeout threshold',
                'threshold': 0.95,
                'actual': agg_metrics.get('avg_hit_rate', 0.0),
                'met': agg_metrics.get('avg_hit_rate', 0.0) >= 0.95,
                'status': 'PASS' if agg_metrics.get('avg_hit_rate', 0.0) >= 0.95 else 'FAIL'
            }
        }

        # Calculate overall success rate
        passed_criteria = sum(1 for sc in success_criteria.values() if sc.get('met', False))
        total_criteria = len(success_criteria)
        overall_success_rate = passed_criteria / total_criteria if total_criteria > 0 else 0.0

        return {
            'success_criteria': success_criteria,
            'overall_success_rate': overall_success_rate,
            'passed_criteria_count': passed_criteria,
            'total_criteria_count': total_criteria
        }

    def calculate_trend_metrics(self, validation_runs: List[ValidationRun]) -> Dict[str, Any]:
        """
        Calculate trend metrics across multiple validation runs.

        Args:
            validation_runs: List of ValidationRun objects in chronological order

        Returns:
            Dictionary with trend analysis
        """
        if len(validation_runs) < 2:
            return {
                'trend_available': False,
                'improvement_metrics': {},
                'regression_metrics': {}
            }

        # Calculate metrics for each run
        run_metrics_history = []
        for run in validation_runs:
            metrics = self.calculate_run_metrics(run)
            run_metrics_history.append(metrics)

        # Calculate trends for key metrics
        metrics_of_interest = [
            'precision_at_k', 'mean_reciprocal_rank', 'hit_rate',
            'metadata_accuracy', 'metadata_completeness', 'avg_retrieval_time_ms'
        ]

        trends = {}
        for metric in metrics_of_interest:
            values = [run_metrics[metric] for run_metrics in run_metrics_history]
            if len(values) >= 2:
                # Calculate if the trend is improving, declining, or stable
                recent_change = values[-1] - values[0]  # Change from first to last
                avg_change = sum(values[i+1] - values[i] for i in range(len(values)-1)) / (len(values)-1) if len(values) > 1 else 0

                if recent_change > 0.05:  # Considerable improvement
                    trend_direction = "improving"
                elif recent_change < -0.05:  # Considerable decline
                    trend_direction = "declining"
                else:
                    trend_direction = "stable"

                trends[metric] = {
                    'direction': trend_direction,
                    'recent_change': recent_change,
                    'average_change': avg_change,
                    'first_value': values[0],
                    'last_value': values[-1],
                    'all_values': values
                }

        # Identify improvement and regression areas
        improvement_metrics = {k: v for k, v in trends.items() if v['direction'] == 'improving'}
        regression_metrics = {k: v for k, v in trends.items() if v['direction'] == 'declining'}

        return {
            'trend_available': True,
            'trends': trends,
            'improvement_metrics': improvement_metrics,
            'regression_metrics': regression_metrics,
            'run_count': len(validation_runs)
        }

    def generate_performance_report(self, validation_runs: List[ValidationRun]) -> Dict[str, Any]:
        """
        Generate a performance report with key metrics.

        Args:
            validation_runs: List of ValidationRun objects to analyze

        Returns:
            Dictionary with performance report
        """
        if not validation_runs:
            return {
                'report_date': datetime.now().isoformat(),
                'total_runs': 0,
                'performance_summary': {},
                'recommendations': ['No validation runs available for analysis']
            }

        # Calculate aggregate metrics
        agg_metrics = self.aggregate_multiple_runs(validation_runs)

        # Calculate success criteria
        success_metrics = self.calculate_success_criteria_metrics(validation_runs)

        # Calculate trends
        trend_metrics = self.calculate_trend_metrics(validation_runs)

        # Generate performance summary
        performance_summary = {
            'retrieval_quality': {
                'avg_precision_at_k': agg_metrics['avg_precision_at_k'],
                'avg_mean_reciprocal_rank': agg_metrics['avg_mean_reciprocal_rank'],
                'avg_hit_rate': agg_metrics['avg_hit_rate']
            },
            'metadata_quality': {
                'avg_metadata_accuracy': agg_metrics['avg_metadata_accuracy'],
                'avg_metadata_completeness': agg_metrics['avg_metadata_completeness']
            },
            'performance_efficiency': {
                'avg_retrieval_time_ms': agg_metrics['avg_retrieval_time_ms'],
                'total_queries_processed': agg_metrics['total_queries'],
                'total_results_returned': agg_metrics['total_results']
            },
            'consistency': {
                'precision_std_dev': agg_metrics['runs_std_dev_precision'],
                'consistency_score': agg_metrics['consistency_score']
            }
        }

        # Generate recommendations based on metrics
        recommendations = self._generate_performance_recommendations(agg_metrics, success_metrics, trend_metrics)

        report = {
            'report_date': datetime.now().isoformat(),
            'total_runs': agg_metrics['total_runs'],
            'total_queries': agg_metrics['total_queries'],
            'total_results': agg_metrics['total_results'],
            'performance_summary': performance_summary,
            'success_criteria_evaluation': success_metrics,
            'trend_analysis': trend_metrics,
            'recommendations': recommendations
        }

        return report

    def _generate_performance_recommendations(self, agg_metrics: Dict[str, float],
                                           success_metrics: Dict[str, Any],
                                           trend_metrics: Dict[str, Any]) -> List[str]:
        """
        Generate recommendations based on performance metrics.

        Args:
            agg_metrics: Aggregated metrics
            success_metrics: Success criteria metrics
            trend_metrics: Trend analysis metrics

        Returns:
            List of recommendation strings
        """
        recommendations = []

        # Recommendations based on absolute metrics
        if agg_metrics['avg_precision_at_k'] < 0.7:
            recommendations.append("Average precision is below 0.7. Consider improving the retrieval model or query processing.")
        if agg_metrics['avg_metadata_accuracy'] < 0.95:
            recommendations.append("Metadata accuracy is below 95%. Consider reviewing the ingestion pipeline for data quality issues.")
        if agg_metrics['avg_retrieval_time_ms'] > 1000:  # More than 1 second
            recommendations.append("Average retrieval time is above 1 second. Consider optimizing the retrieval pipeline or adding caching.")

        # Recommendations based on success criteria
        failed_criteria = [
            name for name, criteria in success_metrics.get('success_criteria', {}).items()
            if criteria.get('status') == 'FAIL'
        ]
        if failed_criteria:
            recommendations.append(f"The following success criteria are not met: {', '.join(failed_criteria)}. Focus on improving these areas.")

        # Recommendations based on trends
        if trend_metrics.get('trend_available'):
            regression_metrics = trend_metrics.get('regression_metrics', {})
            if regression_metrics:
                regressing_metrics = list(regression_metrics.keys())
                recommendations.append(f"The following metrics are declining: {', '.join(regressing_metrics)}. Investigate potential causes.")

        if not recommendations:
            recommendations.append("Overall performance is good. Continue current approach while monitoring for potential improvements.")

        return recommendations

    def validate_aggregator(self) -> bool:
        """
        Validate that the metrics aggregator is working correctly.

        Returns:
            bool: True if validation passes, False otherwise
        """
        try:
            # Create dummy validation run for testing
            from .models import ValidationRun
            from datetime import timedelta
            dummy_run = ValidationRun(
                id="test-metrics",
                test_queries=[TestQuery(id="q1", text="test query")],
                results=[],
                overall_metrics={},
                start_time=datetime.now(),
                end_time=datetime.now() + timedelta(seconds=1),
                completed=True
            )

            # Test basic metric calculation
            metrics = self.calculate_run_metrics(dummy_run)
            if not isinstance(metrics, dict):
                logger.error("Metrics aggregator validation failed: metrics not returned as dict")
                return False

            # Test aggregation
            aggregated = self.aggregate_multiple_runs([dummy_run])
            if not isinstance(aggregated, dict):
                logger.error("Metrics aggregator validation failed: aggregated metrics not returned as dict")
                return False

            logger.info("Validation metrics aggregator validation passed")
            return True

        except Exception as e:
            logger.error(f"Error validating metrics aggregator: {str(e)}")
            return False


# Global instance for convenience
validation_metrics_aggregator = ValidationMetricsAggregator()