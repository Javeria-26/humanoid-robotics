"""
Relevance scoring algorithms for the retrieval validation system.

This module provides functionality to calculate relevance scores
and assess the quality of retrieved results.
"""
import logging
from typing import List, Dict, Any, Optional
from .base import RetrievedResult, TestQuery
from .logger import validation_logger as logger


class RelevanceScoringService:
    """
    Service class for calculating relevance scores and assessing result quality.
    """

    def __init__(self):
        pass

    def calculate_precision_at_k(self, retrieved_results: List[RetrievedResult],
                                 relevant_ids: List[str], k: int = None) -> float:
        """
        Calculate precision at k (P@K) for the retrieved results.

        Args:
            retrieved_results: List of retrieved results
            relevant_ids: List of IDs that are known to be relevant
            k: Number of top results to consider (if None, uses all results)

        Returns:
            Precision at k value (0.0 to 1.0)
        """
        if not retrieved_results:
            return 0.0

        k = k or len(retrieved_results)
        k = min(k, len(retrieved_results))

        top_k_results = retrieved_results[:k]
        top_k_ids = [result.id for result in top_k_results]

        # Count how many of the top-k results are relevant
        relevant_retrieved = sum(1 for result_id in top_k_ids if result_id in relevant_ids)

        precision_at_k = relevant_retrieved / k if k > 0 else 0.0
        return precision_at_k

    def calculate_recall_at_k(self, retrieved_results: List[RetrievedResult],
                              relevant_ids: List[str], k: int = None) -> float:
        """
        Calculate recall at k (R@K) for the retrieved results.

        Args:
            retrieved_results: List of retrieved results
            relevant_ids: List of IDs that are known to be relevant
            k: Number of top results to consider (if None, uses all results)

        Returns:
            Recall at k value (0.0 to 1.0)
        """
        if not relevant_ids:
            return 1.0 if not retrieved_results else 0.0

        k = k or len(retrieved_results)
        k = min(k, len(retrieved_results))

        top_k_results = retrieved_results[:k]
        top_k_ids = [result.id for result in top_k_results]

        # Count how many relevant items were retrieved
        relevant_retrieved = sum(1 for result_id in top_k_ids if result_id in relevant_ids)
        total_relevant = len(relevant_ids)

        recall_at_k = relevant_retrieved / total_relevant if total_relevant > 0 else 0.0
        return recall_at_k

    def calculate_mean_reciprocal_rank(self, retrieved_results: List[RetrievedResult],
                                      relevant_ids: List[str]) -> float:
        """
        Calculate mean reciprocal rank (MRR) for the retrieved results.

        Args:
            retrieved_results: List of retrieved results (in order of relevance)
            relevant_ids: List of IDs that are known to be relevant

        Returns:
            Mean reciprocal rank value (0.0 to 1.0)
        """
        if not relevant_ids or not retrieved_results:
            return 0.0

        # Find the rank (1-indexed) of the first relevant item
        for rank, result in enumerate(retrieved_results, 1):
            if result.id in relevant_ids:
                # Return reciprocal of the rank of the first relevant item
                return 1.0 / rank

        # No relevant items found
        return 0.0

    def calculate_hit_rate(self, retrieved_results: List[RetrievedResult],
                           relevant_ids: List[str]) -> float:
        """
        Calculate hit rate (whether at least one relevant item was retrieved).

        Args:
            retrieved_results: List of retrieved results
            relevant_ids: List of IDs that are known to be relevant

        Returns:
            Hit rate value (0.0 or 1.0)
        """
        if not relevant_ids or not retrieved_results:
            return 0.0

        retrieved_ids = [result.id for result in retrieved_results]
        hit = any(result_id in relevant_ids for result_id in retrieved_ids)
        return 1.0 if hit else 0.0

    def calculate_normalized_discounted_cumulative_gain(self, retrieved_results: List[RetrievedResult],
                                                       relevant_ids: List[str],
                                                       relevance_scores: Dict[str, float] = None) -> float:
        """
        Calculate normalized discounted cumulative gain (NDCG) for the retrieved results.

        Args:
            retrieved_results: List of retrieved results (in order of relevance)
            relevant_ids: List of IDs that are known to be relevant
            relevance_scores: Optional mapping of result ID to relevance score

        Returns:
            NDCG value (0.0 to 1.0)
        """
        if not retrieved_results:
            return 0.0

        # Calculate DCG (Discounted Cumulative Gain)
        dcg = 0.0
        for i, result in enumerate(retrieved_results):
            rank = i + 1  # 1-indexed
            relevance = 1.0 if result.id in relevant_ids else 0.0
            if relevance_scores and result.id in relevance_scores:
                relevance = relevance_scores[result.id]

            # Apply discount based on rank: relevance / log2(rank + 1)
            dcg += relevance / (rank * 1.0)  # Using linear rank discount for simplicity

        # Calculate ideal DCG (IDCG) - results ordered by relevance
        ideal_results = sorted(retrieved_results,
                              key=lambda r: (1.0 if r.id in relevant_ids else 0.0),
                              reverse=True)
        idcg = 0.0
        for i, result in enumerate(ideal_results):
            rank = i + 1  # 1-indexed
            relevance = 1.0 if result.id in relevant_ids else 0.0
            if relevance_scores and result.id in relevance_scores:
                relevance = relevance_scores[result.id]

            idcg += relevance / (rank * 1.0)

        # Calculate NDCG
        if idcg == 0:
            return 0.0
        ndcg = dcg / idcg
        return ndcg

    def assess_relevance_by_content_similarity(self, query: TestQuery,
                                              retrieved_results: List[RetrievedResult]) -> List[float]:
        """
        Assess relevance based on content similarity to the query.

        Args:
            query: The original query
            retrieved_results: List of retrieved results

        Returns:
            List of relevance scores for each result (0.0 to 1.0)
        """
        if not retrieved_results:
            return []

        relevance_scores = []
        query_text_lower = query.text.lower()
        query_words = set(query_text_lower.split())

        for result in retrieved_results:
            result_text_lower = result.text.lower()
            result_words = set(result_text_lower.split())

            # Calculate Jaccard similarity between query and result
            intersection = query_words.intersection(result_words)
            union = query_words.union(result_words)

            if len(union) == 0:
                jaccard_similarity = 0.0
            else:
                jaccard_similarity = len(intersection) / len(union)

            # Also consider overlap in metadata
            metadata_overlap = 0.0
            if hasattr(result, 'metadata') and result.metadata:
                metadata_text = " ".join(str(v) for v in result.metadata.values()).lower()
                meta_words = set(metadata_text.split())
                meta_intersection = query_words.intersection(meta_words)
                if query_words:  # Avoid division by zero
                    metadata_overlap = len(meta_intersection) / len(query_words)

            # Combine content and metadata similarity
            combined_score = 0.7 * jaccard_similarity + 0.3 * metadata_overlap
            relevance_scores.append(min(1.0, combined_score))  # Ensure score is <= 1.0

        return relevance_scores

    def calculate_relevance_metrics(self, retrieved_results: List[RetrievedResult],
                                   relevant_ids: List[str] = None,
                                   query: TestQuery = None) -> Dict[str, float]:
        """
        Calculate comprehensive relevance metrics for the retrieved results.

        Args:
            retrieved_results: List of retrieved results
            relevant_ids: List of IDs that are known to be relevant (optional)
            query: Original query for content-based relevance assessment (optional)

        Returns:
            Dictionary with various relevance metrics
        """
        relevant_ids = relevant_ids or []

        metrics = {
            'precision_at_1': self.calculate_precision_at_k(retrieved_results, relevant_ids, 1),
            'precision_at_3': self.calculate_precision_at_k(retrieved_results, relevant_ids, 3),
            'precision_at_5': self.calculate_precision_at_k(retrieved_results, relevant_ids, 5),
            'precision_at_10': self.calculate_precision_at_k(retrieved_results, relevant_ids, 10),
            'recall_at_1': self.calculate_recall_at_k(retrieved_results, relevant_ids, 1),
            'recall_at_3': self.calculate_recall_at_k(retrieved_results, relevant_ids, 3),
            'recall_at_5': self.calculate_recall_at_k(retrieved_results, relevant_ids, 5),
            'recall_at_10': self.calculate_recall_at_k(retrieved_results, relevant_ids, 10),
            'mean_reciprocal_rank': self.calculate_mean_reciprocal_rank(retrieved_results, relevant_ids),
            'hit_rate': self.calculate_hit_rate(retrieved_results, relevant_ids),
            'avg_similarity_score': 0.0,  # Will calculate if query is provided
        }

        # Calculate content-based similarity if query is provided
        if query:
            similarity_scores = self.assess_relevance_by_content_similarity(query, retrieved_results)
            if similarity_scores:
                avg_similarity = sum(similarity_scores) / len(similarity_scores)
                metrics['avg_similarity_score'] = avg_similarity

        return metrics

    def calculate_validation_metrics(self, retrieved_results: List[RetrievedResult],
                                   relevant_ids: List[str] = None,
                                   query: TestQuery = None) -> Dict[str, float]:
        """
        Calculate comprehensive validation metrics for precision and accuracy.

        Args:
            retrieved_results: List of retrieved results
            relevant_ids: List of IDs that are known to be relevant (optional)
            query: Original query for content-based relevance assessment (optional)

        Returns:
            Dictionary with comprehensive validation metrics
        """
        relevant_ids = relevant_ids or []

        # Calculate relevance metrics
        relevance_metrics = self.calculate_relevance_metrics(retrieved_results, relevant_ids, query)

        # Calculate additional validation metrics
        validation_metrics = {
            'precision_at_k': relevance_metrics['precision_at_5'],  # Using P@5 as primary precision metric
            'recall_at_k': relevance_metrics['recall_at_5'],       # Using R@5 as primary recall metric
            'mean_reciprocal_rank': relevance_metrics['mean_reciprocal_rank'],
            'hit_rate': relevance_metrics['hit_rate'],
            'avg_similarity_score': relevance_metrics['avg_similarity_score'],
            'total_results': len(retrieved_results),
            'relevant_results': len([r for r in retrieved_results if r.id in relevant_ids]) if relevant_ids else 0
        }

        return validation_metrics

    def validate_relevance_scoring(self) -> bool:
        """
        Validate that relevance scoring functions are working correctly.

        Returns:
            bool: True if validation passes, False otherwise
        """
        try:
            # Create dummy results for testing
            dummy_results = [
                RetrievedResult(id="1", text="test result one", score=0.9, metadata={}, query_id="test"),
                RetrievedResult(id="2", text="test result two", score=0.7, metadata={}, query_id="test"),
                RetrievedResult(id="3", text="test result three", score=0.5, metadata={}, query_id="test")
            ]

            # Test basic scoring functions
            precision = self.calculate_precision_at_k(dummy_results, ["1"], 1)
            recall = self.calculate_recall_at_k(dummy_results, ["1"], 3)
            mrr = self.calculate_mean_reciprocal_rank(dummy_results, ["1"])
            hit_rate = self.calculate_hit_rate(dummy_results, ["1"])

            # Basic validation: all values should be between 0 and 1
            if not all(0.0 <= val <= 1.0 for val in [precision, recall, mrr, hit_rate]):
                logger.error("Relevance scoring validation failed: values not in [0,1] range")
                return False

            logger.info("Relevance scoring validation passed")
            return True

        except Exception as e:
            logger.error(f"Error validating relevance scoring: {str(e)}")
            return False


# Global instance for convenience
relevance_scoring_service = RelevanceScoringService()