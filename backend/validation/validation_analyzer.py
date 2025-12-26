"""
Validation result analyzer for the retrieval validation system.

This module provides functionality to analyze validation results
and assess relevance quality.
"""
import logging
from typing import List, Dict, Any, Optional
from .base import RetrievedResult, TestQuery, ValidationResultModel
from .relevance_scoring import relevance_scoring_service
from .metadata_validator import metadata_validator_service
from .models import ValidationRun, ValidationMetrics
from .logger import validation_logger as logger


class ValidationResultAnalyzer:
    """
    Analyzer class for evaluating validation results and assessing relevance quality.
    """

    def __init__(self):
        self.relevance_scorer = relevance_scoring_service
        self.metadata_validator = metadata_validator_service

    def analyze_relevance_quality(self, retrieved_results: List[RetrievedResult],
                                 query: TestQuery,
                                 expected_relevant_ids: List[str] = None) -> Dict[str, Any]:
        """
        Analyze the relevance quality of retrieved results for a query.

        Args:
            retrieved_results: List of retrieved results
            query: Original query that generated these results
            expected_relevant_ids: List of IDs that are expected to be relevant (optional)

        Returns:
            Dictionary with relevance quality analysis
        """
        expected_relevant_ids = expected_relevant_ids or []

        # Calculate relevance metrics
        relevance_metrics = self.relevance_scorer.calculate_relevance_metrics(
            retrieved_results, expected_relevant_ids, query
        )

        # Calculate content-based similarity scores
        similarity_scores = self.relevance_scorer.assess_relevance_by_content_similarity(
            query, retrieved_results
        )

        # Analyze score distribution
        score_distribution = self._analyze_score_distribution([r.score for r in retrieved_results])

        # Analyze result ranking quality
        ranking_quality = self._analyze_ranking_quality(retrieved_results, expected_relevant_ids)

        analysis = {
            'query_id': query.id,
            'query_text': query.text,
            'result_count': len(retrieved_results),
            'relevance_metrics': relevance_metrics,
            'similarity_analysis': {
                'avg_similarity_score': sum(similarity_scores) / len(similarity_scores) if similarity_scores else 0.0,
                'similarity_scores': similarity_scores
            },
            'score_distribution': score_distribution,
            'ranking_quality': ranking_quality,
            'quality_assessment': self._assess_quality(relevance_metrics),
            'recommendations': self._generate_recommendations(relevance_metrics)
        }

        return analysis

    def analyze_validation_run(self, validation_run: ValidationRun) -> Dict[str, Any]:
        """
        Analyze a complete validation run.

        Args:
            validation_run: ValidationRun object to analyze

        Returns:
            Dictionary with analysis of the entire validation run
        """
        all_analyses = []
        total_queries = len(validation_run.test_queries)
        successful_queries = 0
        total_results = 0
        relevant_results = 0

        for i, result in enumerate(validation_run.results):
            if i < len(validation_run.test_queries):
                query = validation_run.test_queries[i]
                analysis = self.analyze_relevance_quality(
                    result.retrieved_results,
                    query,
                    result.query.expected_result_ids if hasattr(result, 'query') and result.query else None
                )
                all_analyses.append(analysis)

                if result.retrieved_results:  # Consider it successful if we got results
                    successful_queries += 1
                total_results += len(result.retrieved_results)
                relevant_results += result.relevant_results if hasattr(result, 'relevant_results') else 0

        # Calculate aggregate metrics
        aggregate_metrics = self._calculate_aggregate_metrics(all_analyses)

        run_analysis = {
            'run_id': validation_run.id,
            'total_queries': total_queries,
            'successful_queries': successful_queries,
            'success_rate': successful_queries / total_queries if total_queries > 0 else 0,
            'total_results': total_results,
            'relevant_results': relevant_results,
            'aggregate_metrics': aggregate_metrics,
            'individual_analyses': all_analyses,
            'run_quality_assessment': self._assess_run_quality(aggregate_metrics),
            'run_recommendations': self._generate_run_recommendations(aggregate_metrics)
        }

        return run_analysis

    def analyze_multiple_runs(self, validation_runs: List[ValidationRun]) -> Dict[str, Any]:
        """
        Analyze multiple validation runs for comparison and trend analysis.

        Args:
            validation_runs: List of ValidationRun objects to analyze

        Returns:
            Dictionary with comparative analysis of multiple runs
        """
        run_analyses = []
        for run in validation_runs:
            run_analysis = self.analyze_validation_run(run)
            run_analyses.append(run_analysis)

        # Calculate trends and comparisons
        trends = self._calculate_trends(run_analyses)

        comparative_analysis = {
            'total_runs': len(validation_runs),
            'run_analyses': run_analyses,
            'trends': trends,
            'consistency_metrics': self._calculate_consistency_metrics(run_analyses)
        }

        return comparative_analysis

    def _analyze_score_distribution(self, scores: List[float]) -> Dict[str, float]:
        """
        Analyze the distribution of similarity scores.

        Args:
            scores: List of similarity scores

        Returns:
            Dictionary with score distribution metrics
        """
        if not scores:
            return {
                'mean': 0.0,
                'median': 0.0,
                'std_dev': 0.0,
                'min': 0.0,
                'max': 0.0,
                'count': 0
            }

        import statistics
        return {
            'mean': statistics.mean(scores),
            'median': statistics.median(scores),
            'std_dev': statistics.stdev(scores) if len(scores) > 1 else 0.0,
            'min': min(scores),
            'max': max(scores),
            'count': len(scores)
        }

    def _analyze_ranking_quality(self, retrieved_results: List[RetrievedResult],
                                expected_relevant_ids: List[str]) -> Dict[str, Any]:
        """
        Analyze the quality of result ranking.

        Args:
            retrieved_results: List of retrieved results (in order of ranking)
            expected_relevant_ids: List of IDs that are expected to be relevant

        Returns:
            Dictionary with ranking quality metrics
        """
        if not expected_relevant_ids or not retrieved_results:
            return {
                'top_3_relevance': 0.0,
                'top_5_relevance': 0.0,
                'relevant_in_top_positions': [],
                'ranking_score': 0.0
            }

        # Check how many relevant items are in top positions
        top_3_ids = [r.id for r in retrieved_results[:3]]
        top_5_ids = [r.id for r in retrieved_results[:5]]

        relevant_in_top_3 = [rid for rid in expected_relevant_ids if rid in top_3_ids]
        relevant_in_top_5 = [rid for rid in expected_relevant_ids if rid in top_5_ids]

        top_3_relevance = len(relevant_in_top_3) / min(3, len(expected_relevant_ids)) if expected_relevant_ids else 0.0
        top_5_relevance = len(relevant_in_top_5) / min(5, len(expected_relevant_ids)) if expected_relevant_ids else 0.0

        # Calculate ranking score based on position of relevant items
        ranking_score = 0.0
        for i, result in enumerate(retrieved_results):
            if result.id in expected_relevant_ids:
                # Higher score for relevant items in higher positions (lower indices)
                position_score = 1.0 / (i + 1)  # Reciprocal rank
                ranking_score += position_score

        return {
            'top_3_relevance': top_3_relevance,
            'top_5_relevance': top_5_relevance,
            'relevant_in_top_positions': {
                'top_3': relevant_in_top_3,
                'top_5': relevant_in_top_5
            },
            'ranking_score': ranking_score,
            'relevant_count': len([r for r in retrieved_results if r.id in expected_relevant_ids])
        }

    def _assess_quality(self, relevance_metrics: Dict[str, float]) -> str:
        """
        Assess the overall quality based on relevance metrics.

        Args:
            relevance_metrics: Dictionary of relevance metrics

        Returns:
            String description of quality level
        """
        p_at_5 = relevance_metrics.get('precision_at_5', 0)
        hit_rate = relevance_metrics.get('hit_rate', 0)
        mrr = relevance_metrics.get('mean_reciprocal_rank', 0)

        if p_at_5 >= 0.8 and hit_rate >= 0.9 and mrr >= 0.7:
            return "High"
        elif p_at_5 >= 0.5 and hit_rate >= 0.7 and mrr >= 0.4:
            return "Medium"
        elif p_at_5 >= 0.2 and hit_rate >= 0.5 and mrr >= 0.2:
            return "Low"
        else:
            return "Very Low"

    def _generate_recommendations(self, relevance_metrics: Dict[str, float]) -> List[str]:
        """
        Generate recommendations based on relevance metrics.

        Args:
            relevance_metrics: Dictionary of relevance metrics

        Returns:
            List of recommendation strings
        """
        recommendations = []

        p_at_5 = relevance_metrics.get('precision_at_5', 0)
        hit_rate = relevance_metrics.get('hit_rate', 0)
        mrr = relevance_metrics.get('mean_reciprocal_rank', 0)

        if p_at_5 < 0.5:
            recommendations.append("Consider improving query processing to increase precision of top results")
        if hit_rate < 0.7:
            recommendations.append("Consider expanding search scope to improve recall")
        if mrr < 0.5:
            recommendations.append("Consider improving ranking algorithm to place relevant results higher")

        if not recommendations:
            recommendations.append("Current performance is good. Consider maintaining current approach.")

        return recommendations

    def _calculate_aggregate_metrics(self, analyses: List[Dict[str, Any]]) -> Dict[str, float]:
        """
        Calculate aggregate metrics from multiple individual analyses.

        Args:
            analyses: List of individual analysis results

        Returns:
            Dictionary with aggregate metrics
        """
        if not analyses:
            return {}

        # Collect metrics from all analyses
        p_at_5_values = []
        p_at_3_values = []
        hit_rates = []
        mrr_values = []
        avg_similarity_scores = []

        for analysis in analyses:
            metrics = analysis.get('relevance_metrics', {})
            p_at_5_values.append(metrics.get('precision_at_5', 0))
            p_at_3_values.append(metrics.get('precision_at_3', 0))
            hit_rates.append(metrics.get('hit_rate', 0))
            mrr_values.append(metrics.get('mean_reciprocal_rank', 0))

            sim_analysis = analysis.get('similarity_analysis', {})
            avg_similarity_scores.append(sim_analysis.get('avg_similarity_score', 0))

        # Calculate aggregate metrics
        aggregate = {}
        if p_at_5_values:
            aggregate['avg_precision_at_5'] = sum(p_at_5_values) / len(p_at_5_values)
        if p_at_3_values:
            aggregate['avg_precision_at_3'] = sum(p_at_3_values) / len(p_at_3_values)
        if hit_rates:
            aggregate['avg_hit_rate'] = sum(hit_rates) / len(hit_rates)
        if mrr_values:
            aggregate['avg_mrr'] = sum(mrr_values) / len(mrr_values)
        if avg_similarity_scores:
            aggregate['avg_similarity_score'] = sum(avg_similarity_scores) / len(avg_similarity_scores)

        return aggregate

    def _assess_run_quality(self, aggregate_metrics: Dict[str, float]) -> str:
        """
        Assess the quality of an entire validation run.

        Args:
            aggregate_metrics: Dictionary of aggregate metrics

        Returns:
            String description of run quality level
        """
        avg_p_at_5 = aggregate_metrics.get('avg_precision_at_5', 0)
        avg_hit_rate = aggregate_metrics.get('avg_hit_rate', 0)
        avg_mrr = aggregate_metrics.get('avg_mrr', 0)

        if avg_p_at_5 >= 0.8 and avg_hit_rate >= 0.9 and avg_mrr >= 0.7:
            return "High"
        elif avg_p_at_5 >= 0.5 and avg_hit_rate >= 0.7 and avg_mrr >= 0.4:
            return "Medium"
        elif avg_p_at_5 >= 0.2 and avg_hit_rate >= 0.5 and avg_mrr >= 0.2:
            return "Low"
        else:
            return "Very Low"

    def _generate_run_recommendations(self, aggregate_metrics: Dict[str, float]) -> List[str]:
        """
        Generate recommendations for an entire validation run.

        Args:
            aggregate_metrics: Dictionary of aggregate metrics

        Returns:
            List of recommendation strings
        """
        recommendations = []

        avg_p_at_5 = aggregate_metrics.get('avg_precision_at_5', 0)
        avg_hit_rate = aggregate_metrics.get('avg_hit_rate', 0)
        avg_mrr = aggregate_metrics.get('avg_mrr', 0)

        if avg_p_at_5 < 0.5:
            recommendations.append("Average precision at 5 is low (<0.5). Consider improving the retrieval model or query processing.")
        if avg_hit_rate < 0.7:
            recommendations.append("Average hit rate is low (<0.7). Consider expanding the search scope or improving recall.")
        if avg_mrr < 0.5:
            recommendations.append("Average mean reciprocal rank is low (<0.5). Consider improving the ranking algorithm.")

        if not recommendations:
            recommendations.append("Overall validation run performance is good. Consider continuing with current approach.")

        return recommendations

    def _calculate_trends(self, run_analyses: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Calculate trends across multiple validation runs.

        Args:
            run_analyses: List of validation run analyses

        Returns:
            Dictionary with trend analysis
        """
        if len(run_analyses) < 2:
            return {}

        # Extract metrics from each run
        run_metrics = []
        for analysis in run_analyses:
            run_metrics.append(analysis.get('aggregate_metrics', {}))

        # Calculate trends for key metrics
        metrics_of_interest = ['avg_precision_at_5', 'avg_hit_rate', 'avg_mrr']
        trends = {}

        for metric in metrics_of_interest:
            values = [m.get(metric, 0) for m in run_metrics if metric in m]
            if len(values) >= 2:
                # Calculate if the trend is increasing, decreasing, or stable
                if len(values) >= 2:
                    recent_change = values[-1] - values[-2]
                    overall_trend = "stable"
                    if recent_change > 0.05:
                        overall_trend = "increasing"
                    elif recent_change < -0.05:
                        overall_trend = "decreasing"

                    trends[metric] = {
                        'current': values[-1],
                        'previous': values[-2] if len(values) >= 2 else None,
                        'trend': overall_trend,
                        'all_values': values
                    }

        return {
            'metric_trends': trends,
            'improvement_areas': self._identify_improvement_areas(trends),
            'consistency': self._calculate_run_consistency(run_metrics)
        }

    def _identify_improvement_areas(self, trends: Dict[str, Any]) -> List[str]:
        """
        Identify areas for improvement based on trends.

        Args:
            trends: Dictionary of metric trends

        Returns:
            List of improvement area descriptions
        """
        improvement_areas = []

        for metric, trend_data in trends.items():
            if trend_data.get('trend') == 'decreasing':
                improvement_areas.append(f"{metric.replace('_', ' ').title()} is decreasing and needs attention")

        return improvement_areas

    def _calculate_run_consistency(self, run_metrics: List[Dict[str, Any]]) -> Dict[str, float]:
        """
        Calculate consistency metrics across runs.

        Args:
            run_metrics: List of metrics from different runs

        Returns:
            Dictionary with consistency metrics
        """
        if not run_metrics:
            return {}

        # Calculate consistency for key metrics
        metrics_of_interest = ['avg_precision_at_5', 'avg_hit_rate', 'avg_mrr']
        consistency = {}

        for metric in metrics_of_interest:
            values = [m.get(metric, 0) for m in run_metrics if metric in m]
            if values:
                import statistics
                if len(values) > 1:
                    std_dev = statistics.stdev(values)
                    mean_val = statistics.mean(values)
                    cv = std_dev / mean_val if mean_val != 0 else 0
                    consistency[f'{metric}_consistency'] = 1 - cv  # Higher is more consistent
                    consistency[f'{metric}_std_dev'] = std_dev
                else:
                    consistency[f'{metric}_consistency'] = 1.0  # Perfect consistency with one value
                    consistency[f'{metric}_std_dev'] = 0.0

        return consistency

    def validate_analyzer(self) -> bool:
        """
        Validate that the analyzer is working correctly.

        Returns:
            bool: True if validation passes, False otherwise
        """
        try:
            # Create dummy data for testing
            dummy_results = [
                RetrievedResult(id="1", text="test result one", score=0.9, metadata={}, query_id="test"),
                RetrievedResult(id="2", text="test result two", score=0.7, metadata={}, query_id="test"),
                RetrievedResult(id="3", text="test result three", score=0.5, metadata={}, query_id="test")
            ]
            dummy_query = TestQuery(id="test", text="test query")

            # Test basic analysis
            analysis = self.analyze_relevance_quality(dummy_results, dummy_query)

            # Check that analysis contains expected keys
            expected_keys = ['query_id', 'relevance_metrics', 'quality_assessment']
            if not all(key in analysis for key in expected_keys):
                logger.error("Analyzer validation failed: missing expected keys in analysis")
                return False

            logger.info("Validation result analyzer validation passed")
            return True

        except Exception as e:
            logger.error(f"Error validating analyzer: {str(e)}")
            return False


# Global instance for convenience
validation_result_analyzer = ValidationResultAnalyzer()