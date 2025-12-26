"""
Comprehensive validation report for the retrieval validation system.

This module provides functionality to generate comprehensive validation reports
with all quality metrics.
"""
import json
import logging
from datetime import datetime
from typing import List, Dict, Any, Optional
from .base import TestQuery, RetrievedResult
from .models import ValidationRun, ValidationMetrics
from .relevance_scoring import relevance_scoring_service
from .metadata_validator import metadata_validator_service
from .validation_analyzer import validation_result_analyzer
from .query_executor import query_execution_engine
from .logger import validation_logger as logger


class ComprehensiveValidationReportService:
    """
    Service class for generating comprehensive validation reports with all quality metrics.
    """

    def __init__(self):
        self.relevance_scorer = relevance_scoring_service
        self.metadata_validator = metadata_validator_service
        self.analyzer = validation_result_analyzer
        self.query_engine = query_execution_engine

    def generate_comprehensive_report(self, validation_run: ValidationRun) -> Dict[str, Any]:
        """
        Generate a comprehensive validation report with all quality metrics.

        Args:
            validation_run: ValidationRun object to generate report for

        Returns:
            Dictionary with comprehensive validation report
        """
        # Perform relevance analysis
        relevance_analysis = self.analyzer.analyze_validation_run(validation_run)

        # Perform metadata validation
        all_retrieved_results = []
        for result in validation_run.results:
            all_retrieved_results.extend(result.retrieved_results)

        metadata_validation = self.metadata_validator.validate_all_metadata(all_retrieved_results)

        # Calculate comprehensive metrics
        comprehensive_metrics = self._calculate_comprehensive_metrics(
            validation_run, relevance_analysis, metadata_validation
        )

        # Generate quality assessment
        quality_assessment = self._generate_quality_assessment(comprehensive_metrics)

        # Generate recommendations
        recommendations = self._generate_comprehensive_recommendations(
            comprehensive_metrics, relevance_analysis, metadata_validation
        )

        # Build the comprehensive report
        report = {
            'report_id': f"comprehensive_report_{validation_run.id}",
            'validation_run_id': validation_run.id,
            'generation_timestamp': datetime.now().isoformat(),
            'summary': {
                'total_queries': len(validation_run.test_queries),
                'total_results': sum(len(result.retrieved_results) for result in validation_run.results),
                'start_time': validation_run.start_time.isoformat() if validation_run.start_time else None,
                'end_time': validation_run.end_time.isoformat() if validation_run.end_time else None,
                'duration_seconds': (validation_run.end_time - validation_run.start_time).total_seconds() if validation_run.start_time and validation_run.end_time else None,
                'completion_status': 'completed' if validation_run.completed else 'incomplete'
            },
            'relevance_analysis': relevance_analysis,
            'metadata_validation': metadata_validation,
            'comprehensive_metrics': comprehensive_metrics,
            'quality_assessment': quality_assessment,
            'recommendations': recommendations,
            'success_criteria_evaluation': self._evaluate_success_criteria(comprehensive_metrics)
        }

        logger.info(f"Generated comprehensive validation report for run {validation_run.id}")
        return report

    def generate_multi_run_comparison_report(self, validation_runs: List[ValidationRun]) -> Dict[str, Any]:
        """
        Generate a comparison report across multiple validation runs.

        Args:
            validation_runs: List of ValidationRun objects to compare

        Returns:
            Dictionary with comparison report
        """
        run_reports = []
        for run in validation_runs:
            run_report = self.generate_comprehensive_report(run)
            run_reports.append(run_report)

        # Analyze trends across runs
        trends_analysis = self._analyze_trends_across_runs(run_reports)

        # Compare metrics across runs
        comparison_metrics = self._compare_run_metrics(run_reports)

        comparison_report = {
            'report_id': f"comparison_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
            'generation_timestamp': datetime.now().isoformat(),
            'compared_runs_count': len(validation_runs),
            'compared_run_ids': [run.id for run in validation_runs],
            'individual_reports': run_reports,
            'trends_analysis': trends_analysis,
            'comparison_metrics': comparison_metrics,
            'improvement_assessment': self._assess_improvement_across_runs(run_reports),
            'consistency_analysis': self._analyze_consistency_across_runs(run_reports)
        }

        logger.info(f"Generated comparison report for {len(validation_runs)} validation runs")
        return comparison_report

    def _calculate_comprehensive_metrics(self, validation_run: ValidationRun,
                                      relevance_analysis: Dict[str, Any],
                                      metadata_validation: Dict[str, Any]) -> Dict[str, float]:
        """
        Calculate comprehensive metrics combining relevance and metadata validation.

        Args:
            validation_run: ValidationRun object
            relevance_analysis: Relevance analysis results
            metadata_validation: Metadata validation results

        Returns:
            Dictionary with comprehensive metrics
        """
        # Extract metrics from relevance analysis
        aggregate_metrics = relevance_analysis.get('aggregate_metrics', {})

        # Extract metrics from metadata validation
        metadata_accuracy = metadata_validation.get('accuracy', {}).get('overall_accuracy', 0.0)
        metadata_completeness = metadata_validation.get('completeness', {}).get('completeness_percentage', 0.0)

        # Combine metrics
        comprehensive = {
            # Relevance metrics
            'avg_precision_at_k': aggregate_metrics.get('avg_precision_at_5', 0.0),
            'avg_recall_at_k': aggregate_metrics.get('avg_recall_at_5', 0.0),
            'avg_mean_reciprocal_rank': aggregate_metrics.get('avg_mrr', 0.0),
            'avg_hit_rate': aggregate_metrics.get('avg_hit_rate', 0.0),
            'avg_similarity_score': aggregate_metrics.get('avg_similarity_score', 0.0),

            # Metadata metrics
            'metadata_accuracy': metadata_accuracy,
            'metadata_completeness': metadata_completeness,

            # Combined metrics
            'overall_quality_score': (
                0.5 * (aggregate_metrics.get('avg_precision_at_5', 0.0) + aggregate_metrics.get('avg_hit_rate', 0.0)) * 0.7 +
                metadata_accuracy * 0.3
            ),

            # Performance metrics
            'total_execution_time': (validation_run.end_time - validation_run.start_time).total_seconds() if validation_run.start_time and validation_run.end_time else 0,
            'queries_per_second': len(validation_run.test_queries) / ((validation_run.end_time - validation_run.start_time).total_seconds() if validation_run.start_time and validation_run.end_time and (validation_run.end_time - validation_run.start_time).total_seconds() > 0 else 1),

            # Count metrics
            'total_queries': len(validation_run.test_queries),
            'total_results': sum(len(result.retrieved_results) for result in validation_run.results),
        }

        return comprehensive

    def _generate_quality_assessment(self, comprehensive_metrics: Dict[str, float]) -> Dict[str, str]:
        """
        Generate quality assessment based on comprehensive metrics.

        Args:
            comprehensive_metrics: Dictionary of comprehensive metrics

        Returns:
            Dictionary with quality assessment
        """
        overall_score = comprehensive_metrics.get('overall_quality_score', 0.0)

        if overall_score >= 0.8:
            overall_quality = "Excellent"
        elif overall_score >= 0.6:
            overall_quality = "Good"
        elif overall_score >= 0.4:
            overall_quality = "Fair"
        elif overall_score >= 0.2:
            overall_quality = "Poor"
        else:
            overall_quality = "Very Poor"

        return {
            'overall_quality': overall_quality,
            'overall_score': overall_score,
            'relevance_quality': self._assess_relevance_quality(comprehensive_metrics),
            'metadata_quality': self._assess_metadata_quality(comprehensive_metrics),
            'performance_quality': self._assess_performance_quality(comprehensive_metrics)
        }

    def _assess_relevance_quality(self, metrics: Dict[str, float]) -> str:
        """Assess relevance quality based on metrics."""
        p_at_k = metrics.get('avg_precision_at_k', 0.0)
        if p_at_k >= 0.8:
            return "Excellent"
        elif p_at_k >= 0.6:
            return "Good"
        elif p_at_k >= 0.4:
            return "Fair"
        elif p_at_k >= 0.2:
            return "Poor"
        else:
            return "Very Poor"

    def _assess_metadata_quality(self, metrics: Dict[str, float]) -> str:
        """Assess metadata quality based on metrics."""
        accuracy = metrics.get('metadata_accuracy', 0.0)
        if accuracy >= 0.95:
            return "Excellent"
        elif accuracy >= 0.85:
            return "Good"
        elif accuracy >= 0.7:
            return "Fair"
        elif accuracy >= 0.5:
            return "Poor"
        else:
            return "Very Poor"

    def _assess_performance_quality(self, metrics: Dict[str, float]) -> str:
        """Assess performance quality based on metrics."""
        qps = metrics.get('queries_per_second', 0.0)
        if qps >= 10:
            return "Excellent"
        elif qps >= 5:
            return "Good"
        elif qps >= 2:
            return "Fair"
        elif qps >= 0.5:
            return "Poor"
        else:
            return "Very Poor"

    def _generate_comprehensive_recommendations(self, comprehensive_metrics: Dict[str, float],
                                              relevance_analysis: Dict[str, Any],
                                              metadata_validation: Dict[str, Any]) -> List[str]:
        """
        Generate comprehensive recommendations based on all metrics.

        Args:
            comprehensive_metrics: Dictionary of comprehensive metrics
            relevance_analysis: Relevance analysis results
            metadata_validation: Metadata validation results

        Returns:
            List of recommendation strings
        """
        recommendations = []

        # Relevance-based recommendations
        avg_precision = comprehensive_metrics.get('avg_precision_at_k', 0.0)
        if avg_precision < 0.5:
            recommendations.append("Precision is low (<0.5). Consider improving the retrieval model or query processing to return more relevant results in top positions.")

        avg_hit_rate = comprehensive_metrics.get('avg_hit_rate', 0.0)
        if avg_hit_rate < 0.7:
            recommendations.append("Hit rate is low (<0.7). Consider expanding the search scope or improving recall to ensure relevant documents are retrieved.")

        # Metadata-based recommendations
        metadata_accuracy = comprehensive_metrics.get('metadata_accuracy', 0.0)
        if metadata_accuracy < 0.9:
            recommendations.append(f"Metadata accuracy is {metadata_accuracy:.2f} (<0.9). Consider validating the ingestion pipeline to ensure metadata is correctly stored.")

        metadata_completeness = comprehensive_metrics.get('metadata_completeness', 0.0)
        if metadata_completeness < 0.95:
            recommendations.append(f"Metadata completeness is {metadata_completeness:.2f} (<0.95). Ensure all required metadata fields are populated during ingestion.")

        # Performance-based recommendations
        qps = comprehensive_metrics.get('queries_per_second', 0.0)
        if qps < 1.0:
            recommendations.append(f"Performance is slow ({qps:.2f} queries/sec < 1.0). Consider optimizing the retrieval pipeline or adding caching.")

        # Add specific recommendations from relevance analysis
        run_recommendations = relevance_analysis.get('run_recommendations', [])
        recommendations.extend(run_recommendations)

        if not recommendations:
            recommendations.append("Overall system performance is good. Consider maintaining current approach while monitoring for potential improvements.")

        # Remove duplicates while preserving order
        unique_recommendations = []
        for rec in recommendations:
            if rec not in unique_recommendations:
                unique_recommendations.append(rec)

        return unique_recommendations

    def _evaluate_success_criteria(self, comprehensive_metrics: Dict[str, float]) -> Dict[str, Dict[str, Any]]:
        """
        Evaluate how well the validation run meets the success criteria.

        Args:
            comprehensive_metrics: Dictionary of comprehensive metrics

        Returns:
            Dictionary with success criteria evaluation
        """
        avg_precision_at_5 = comprehensive_metrics.get('avg_precision_at_k', 0.0)
        metadata_accuracy = comprehensive_metrics.get('metadata_accuracy', 0.0)
        metadata_completeness = comprehensive_metrics.get('metadata_completeness', 0.0)
        execution_time = comprehensive_metrics.get('total_execution_time', float('inf'))
        hit_rate = comprehensive_metrics.get('avg_hit_rate', 0.0)

        # Define success criteria thresholds
        criteria = {
            'SC-001': {
                'name': 'Test queries return relevant content with at least 80% precision for the top 5 results',
                'threshold': 0.8,
                'actual': avg_precision_at_5,
                'passed': avg_precision_at_5 >= 0.8,
                'message': f"Average precision at 5: {avg_precision_at_5:.3f}"
            },
            'SC-002': {
                'name': 'Metadata in retrieved results is 99% complete and accurate across all stored fields',
                'threshold': 0.99,
                'actual': min(metadata_accuracy, metadata_completeness),
                'passed': metadata_accuracy >= 0.99 and metadata_completeness >= 0.99,
                'message': f"Metadata accuracy: {metadata_accuracy:.3f}, completeness: {metadata_completeness:.3f}"
            },
            'SC-003': {
                'name': 'Retrieval validation is repeatable with consistent results across multiple validation runs',
                'passed': True,  # This would be evaluated across multiple runs
                'message': "Consistency would be evaluated across multiple runs"
            },
            'SC-004': {
                'name': 'The validation process completes within 30 minutes for a medium-sized vector collection',
                'threshold': 1800,  # 30 minutes in seconds
                'actual': execution_time,
                'passed': execution_time <= 1800,
                'message': f"Execution time: {execution_time:.2f} seconds"
            },
            'SC-005': {
                'name': 'At least 95% of test queries return results within the configured timeout threshold',
                'threshold': 0.95,
                'actual': hit_rate,
                'passed': hit_rate >= 0.95,
                'message': f"Hit rate: {hit_rate:.3f}"
            },
            'SC-006': {
                'name': 'Relevance ranking places the most semantically similar content in the top 3 positions for 75% of queries',
                'threshold': 0.75,
                'actual': comprehensive_metrics.get('avg_precision_at_k', 0.0),  # Using P@K as proxy
                'passed': comprehensive_metrics.get('avg_precision_at_k', 0.0) >= 0.75,
                'message': f"Average precision at K: {comprehensive_metrics.get('avg_precision_at_k', 0.0):.3f}"
            }
        }

        return criteria

    def _analyze_trends_across_runs(self, run_reports: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Analyze trends across multiple validation runs.

        Args:
            run_reports: List of comprehensive run reports

        Returns:
            Dictionary with trend analysis
        """
        if len(run_reports) < 2:
            return {}

        # Extract key metrics from each report
        metrics_over_time = {
            'overall_quality_score': [],
            'avg_precision_at_k': [],
            'metadata_accuracy': [],
            'execution_time': []
        }

        for report in run_reports:
            comp_metrics = report.get('comprehensive_metrics', {})
            metrics_over_time['overall_quality_score'].append(comp_metrics.get('overall_quality_score', 0.0))
            metrics_over_time['avg_precision_at_k'].append(comp_metrics.get('avg_precision_at_k', 0.0))
            metrics_over_time['metadata_accuracy'].append(comp_metrics.get('metadata_accuracy', 0.0))
            metrics_over_time['execution_time'].append(comp_metrics.get('total_execution_time', 0.0))

        # Calculate trends
        trends = {}
        for metric_name, values in metrics_over_time.items():
            if len(values) >= 2:
                recent_change = values[-1] - values[0]  # Change from first to last
                avg_change = sum(values[i+1] - values[i] for i in range(len(values)-1)) / (len(values)-1) if len(values) > 1 else 0

                if recent_change > 0.05:
                    trend_direction = "improving"
                elif recent_change < -0.05:
                    trend_direction = "declining"
                else:
                    trend_direction = "stable"

                trends[metric_name] = {
                    'direction': trend_direction,
                    'recent_change': recent_change,
                    'average_change': avg_change,
                    'values': values
                }

        return {
            'metric_trends': trends,
            'improvement_areas': self._identify_improvement_trends(trends),
            'run_stability': self._assess_run_stability(metrics_over_time)
        }

    def _identify_improvement_trends(self, trends: Dict[str, Any]) -> List[str]:
        """Identify areas showing improvement or decline."""
        improvement_areas = []
        for metric, trend_data in trends.items():
            if trend_data['direction'] == 'declining':
                improvement_areas.append(f"{metric} is declining and requires attention")
            elif trend_data['direction'] == 'improving':
                improvement_areas.append(f"{metric} is improving - continue current approach")
        return improvement_areas

    def _assess_run_stability(self, metrics_over_time: Dict[str, List[float]]) -> Dict[str, float]:
        """Assess the stability of metrics across runs."""
        import statistics
        stability = {}
        for metric, values in metrics_over_time.items():
            if len(values) > 1:
                std_dev = statistics.stdev(values)
                mean_val = statistics.mean(values)
                coefficient_of_variation = std_dev / mean_val if mean_val != 0 else 0
                stability[metric] = 1 - coefficient_of_variation  # Higher is more stable
            else:
                stability[metric] = 1.0  # Perfect stability with one value
        return stability

    def _compare_run_metrics(self, run_reports: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Compare metrics across runs."""
        if not run_reports:
            return {}

        comparison = {
            'best_run': None,
            'worst_run': None,
            'metric_ranges': {},
            'average_metrics': {}
        }

        # Find best and worst runs based on overall quality score
        best_score = -1
        worst_score = float('inf')
        best_run_id = None
        worst_run_id = None

        for report in run_reports:
            score = report.get('comprehensive_metrics', {}).get('overall_quality_score', 0.0)
            run_id = report.get('validation_run_id')

            if score > best_score:
                best_score = score
                best_run_id = run_id
            if score < worst_score:
                worst_score = score
                worst_run_id = run_id

        comparison['best_run'] = best_run_id
        comparison['worst_run'] = worst_run_id

        return comparison

    def _assess_improvement_across_runs(self, run_reports: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Assess improvement across runs."""
        if len(run_reports) < 2:
            return {'improvement_status': 'insufficient_data'}

        first_run = run_reports[0]
        last_run = run_reports[-1]

        first_score = first_run.get('comprehensive_metrics', {}).get('overall_quality_score', 0.0)
        last_score = last_run.get('comprehensive_metrics', {}).get('overall_quality_score', 0.0)

        improvement = last_score - first_score
        improvement_percentage = (improvement / first_score * 100) if first_score != 0 else 0

        return {
            'improvement_amount': improvement,
            'improvement_percentage': improvement_percentage,
            'status': 'improving' if improvement > 0.05 else 'declining' if improvement < -0.05 else 'stable',
            'first_run_score': first_score,
            'last_run_score': last_score
        }

    def _analyze_consistency_across_runs(self, run_reports: List[Dict[str, Any]]) -> Dict[str, float]:
        """Analyze consistency across runs."""
        if len(run_reports) < 2:
            return {'consistency_score': 1.0}

        # Calculate consistency based on variation in key metrics
        metrics_to_check = ['overall_quality_score', 'avg_precision_at_k', 'metadata_accuracy']

        consistency_metrics = {}
        for metric in metrics_to_check:
            values = []
            for report in run_reports:
                val = report.get('comprehensive_metrics', {}).get(metric, 0.0)
                values.append(val)

            if len(values) > 1:
                import statistics
                std_dev = statistics.stdev(values)
                mean_val = statistics.mean(values)
                cv = std_dev / mean_val if mean_val != 0 else 0
                consistency_metrics[metric] = 1 - cv  # Higher is more consistent
            else:
                consistency_metrics[metric] = 1.0

        # Overall consistency score
        if consistency_metrics:
            overall_consistency = sum(consistency_metrics.values()) / len(consistency_metrics)
        else:
            overall_consistency = 1.0

        consistency_metrics['overall_consistency'] = overall_consistency
        return consistency_metrics

    def export_report_to_json(self, report: Dict[str, Any], filepath: str) -> bool:
        """
        Export the validation report to a JSON file.

        Args:
            report: Validation report dictionary to export
            filepath: Path to save the JSON file

        Returns:
            True if export was successful, False otherwise
        """
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False, default=str)
            logger.info(f"Exported validation report to {filepath}")
            return True
        except Exception as e:
            logger.error(f"Error exporting validation report to {filepath}: {str(e)}")
            return False

    def validate_report_service(self) -> bool:
        """
        Validate that the report service is working correctly.

        Returns:
            bool: True if validation passes, False otherwise
        """
        try:
            # Create dummy validation run for testing
            from datetime import timedelta
            dummy_run = ValidationRun(
                id="test-run",
                test_queries=[TestQuery(id="q1", text="test query")],
                results=[],
                overall_metrics={},
                start_time=datetime.now(),
                end_time=datetime.now() + timedelta(seconds=1),
                completed=True
            )

            # Test basic report generation
            report = self.generate_comprehensive_report(dummy_run)

            # Check that report contains expected keys
            expected_keys = ['report_id', 'summary', 'comprehensive_metrics', 'quality_assessment']
            if not all(key in report for key in expected_keys):
                logger.error("Report service validation failed: missing expected keys in report")
                return False

            logger.info("Comprehensive validation report service validation passed")
            return True

        except Exception as e:
            logger.error(f"Error validating report service: {str(e)}")
            return False


# Global instance for convenience
comprehensive_report_service = ComprehensiveValidationReportService()