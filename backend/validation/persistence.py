"""
Persistence module for the retrieval validation system.

This module provides functionality to persist validation results
for repeatable runs and historical analysis.
"""
import json
import os
import logging
from datetime import datetime
from typing import List, Dict, Any, Optional
from .models import ValidationRun, ValidationResultModel
from .base import TestQuery, RetrievedResult
from .logger import validation_logger as logger


class ValidationPersistenceService:
    """
    Service class for persisting validation results and managing historical data.
    """

    def __init__(self, storage_path: str = "./validation_storage"):
        self.storage_path = storage_path
        self._ensure_storage_directory()

    def _ensure_storage_directory(self):
        """Ensure the storage directory exists."""
        os.makedirs(self.storage_path, exist_ok=True)

    def save_validation_run(self, validation_run: ValidationRun) -> bool:
        """
        Save a validation run to persistent storage.

        Args:
            validation_run: ValidationRun object to save

        Returns:
            bool: True if save was successful, False otherwise
        """
        try:
            # Create a serializable representation
            run_data = {
                'id': validation_run.id,
                'test_queries': [
                    {
                        'id': q.id,
                        'text': q.text,
                        'category': q.category,
                        'expected_result_ids': q.expected_result_ids,
                        'created_at': q.created_at.isoformat() if q.created_at else None
                    }
                    for q in validation_run.test_queries
                ],
                'results': [
                    {
                        'id': r.id,
                        'query_id': r.query_id,
                        'total_results': r.total_results,
                        'relevant_results': r.relevant_results,
                        'precision': r.precision,
                        'metadata_accuracy': r.metadata_accuracy,
                        'validation_timestamp': r.validation_timestamp.isoformat() if r.validation_timestamp else None,
                        'retrieval_time_ms': r.retrieval_time_ms,
                        'details': r.details,
                        'retrieved_results': [
                            {
                                'id': res.id,
                                'text': res.text,
                                'score': res.score,
                                'metadata': res.metadata,
                                'query_id': res.query_id
                            }
                            for res in r.retrieved_results
                        ]
                    }
                    for r in validation_run.results
                ],
                'overall_metrics': validation_run.overall_metrics,
                'start_time': validation_run.start_time.isoformat() if validation_run.start_time else None,
                'end_time': validation_run.end_time.isoformat() if validation_run.end_time else None,
                'completed': validation_run.completed
            }

            # Save to file
            filename = f"validation_run_{validation_run.id}.json"
            filepath = os.path.join(self.storage_path, filename)

            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(run_data, f, indent=2, ensure_ascii=False)

            logger.info(f"Saved validation run {validation_run.id} to {filepath}")
            return True

        except Exception as e:
            logger.error(f"Error saving validation run {validation_run.id}: {str(e)}")
            return False

    def load_validation_run(self, run_id: str) -> Optional[ValidationRun]:
        """
        Load a validation run from persistent storage.

        Args:
            run_id: ID of the validation run to load

        Returns:
            ValidationRun object or None if not found
        """
        try:
            filename = f"validation_run_{run_id}.json"
            filepath = os.path.join(self.storage_path, filename)

            if not os.path.exists(filepath):
                logger.warning(f"Validation run {run_id} not found at {filepath}")
                return None

            with open(filepath, 'r', encoding='utf-8') as f:
                run_data = json.load(f)

            # Reconstruct the ValidationRun object
            test_queries = []
            for q_data in run_data['test_queries']:
                query = TestQuery(
                    id=q_data['id'],
                    text=q_data['text'],
                    category=q_data.get('category'),
                    expected_result_ids=q_data.get('expected_result_ids'),
                    created_at=datetime.fromisoformat(q_data['created_at']) if q_data['created_at'] else None
                )
                test_queries.append(query)

            results = []
            for r_data in run_data['results']:
                retrieved_results = []
                for res_data in r_data.get('retrieved_results', []):
                    retrieved_result = RetrievedResult(
                        id=res_data['id'],
                        text=res_data['text'],
                        score=res_data['score'],
                        metadata=res_data['metadata'],
                        query_id=res_data['query_id']
                    )
                    retrieved_results.append(retrieved_result)

                result = ValidationResultModel(
                    id=r_data['id'],
                    query_id=r_data['query_id'],
                    total_results=r_data['total_results'],
                    relevant_results=r_data['relevant_results'],
                    precision=r_data['precision'],
                    metadata_accuracy=r_data['metadata_accuracy'],
                    validation_timestamp=datetime.fromisoformat(r_data['validation_timestamp']) if r_data['validation_timestamp'] else None,
                    retrieval_time_ms=r_data['retrieval_time_ms'],
                    details=r_data.get('details')
                )
                result.retrieved_results = retrieved_results  # Add the retrieved results
                results.append(result)

            validation_run = ValidationRun(
                id=run_data['id'],
                test_queries=test_queries,
                results=results,
                overall_metrics=run_data['overall_metrics'],
                start_time=datetime.fromisoformat(run_data['start_time']) if run_data['start_time'] else None,
                end_time=datetime.fromisoformat(run_data['end_time']) if run_data['end_time'] else None,
                completed=run_data['completed']
            )

            logger.info(f"Loaded validation run {run_id} from {filepath}")
            return validation_run

        except Exception as e:
            logger.error(f"Error loading validation run {run_id}: {str(e)}")
            return None

    def list_validation_runs(self) -> List[Dict[str, Any]]:
        """
        List all available validation runs.

        Returns:
            List of dictionaries with run information
        """
        try:
            run_files = [f for f in os.listdir(self.storage_path) if f.startswith("validation_run_") and f.endswith(".json")]
            runs_info = []

            for filename in run_files:
                filepath = os.path.join(self.storage_path, filename)
                run_id = filename.replace("validation_run_", "").replace(".json", "")

                # Get file modification time for timestamp
                mod_time = datetime.fromtimestamp(os.path.getmtime(filepath))

                runs_info.append({
                    'id': run_id,
                    'filename': filename,
                    'modified_at': mod_time.isoformat(),
                    'size': os.path.getsize(filepath)
                })

            # Sort by modification time (newest first)
            runs_info.sort(key=lambda x: x['modified_at'], reverse=True)

            logger.info(f"Found {len(runs_info)} validation runs")
            return runs_info

        except Exception as e:
            logger.error(f"Error listing validation runs: {str(e)}")
            return []

    def delete_validation_run(self, run_id: str) -> bool:
        """
        Delete a validation run from storage.

        Args:
            run_id: ID of the validation run to delete

        Returns:
            bool: True if deletion was successful, False otherwise
        """
        try:
            filename = f"validation_run_{run_id}.json"
            filepath = os.path.join(self.storage_path, filename)

            if os.path.exists(filepath):
                os.remove(filepath)
                logger.info(f"Deleted validation run {run_id}")
                return True
            else:
                logger.warning(f"Validation run {run_id} not found for deletion")
                return False

        except Exception as e:
            logger.error(f"Error deleting validation run {run_id}: {str(e)}")
            return False

    def save_validation_results(self, run_id: str, results: List[ValidationResultModel]) -> bool:
        """
        Save just the validation results for a run (for updating existing runs).

        Args:
            run_id: ID of the validation run
            results: List of ValidationResultModel objects to save

        Returns:
            bool: True if save was successful, False otherwise
        """
        # Load existing run
        existing_run = self.load_validation_run(run_id)
        if not existing_run:
            logger.error(f"Cannot update results for non-existent run: {run_id}")
            return False

        # Update results
        existing_run.results = results

        # Save updated run
        return self.save_validation_run(existing_run)

    def get_validation_run_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Get a history of recent validation runs.

        Args:
            limit: Maximum number of runs to return

        Returns:
            List of recent validation run information
        """
        runs_info = self.list_validation_runs()
        return runs_info[:limit]

    def get_validation_run_summary(self, run_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a summary of a validation run without loading the full data.

        Args:
            run_id: ID of the validation run

        Returns:
            Dictionary with summary information or None if not found
        """
        try:
            filename = f"validation_run_{run_id}.json"
            filepath = os.path.join(self.storage_path, filename)

            if not os.path.exists(filepath):
                return None

            # Just read basic information without loading full results
            with open(filepath, 'r', encoding='utf-8') as f:
                run_data = json.load(f)

            # Calculate some summary metrics
            total_queries = len(run_data.get('test_queries', []))
            total_results = sum(len(r.get('retrieved_results', [])) for r in run_data.get('results', []))
            completed = run_data.get('completed', False)

            # Calculate average metrics
            if run_data.get('results'):
                avg_precision = sum(r['precision'] for r in run_data['results']) / len(run_data['results'])
                avg_metadata_accuracy = sum(r['metadata_accuracy'] for r in run_data['results']) / len(run_data['results'])
                avg_retrieval_time = sum(r['retrieval_time_ms'] for r in run_data['results']) / len(run_data['results'])
            else:
                avg_precision = 0.0
                avg_metadata_accuracy = 0.0
                avg_retrieval_time = 0.0

            summary = {
                'id': run_data['id'],
                'total_queries': total_queries,
                'total_results': total_results,
                'completed': completed,
                'start_time': run_data.get('start_time'),
                'end_time': run_data.get('end_time'),
                'avg_precision': avg_precision,
                'avg_metadata_accuracy': avg_metadata_accuracy,
                'avg_retrieval_time_ms': avg_retrieval_time,
                'overall_metrics': run_data.get('overall_metrics', {}),
                'file_size': os.path.getsize(filepath)
            }

            return summary

        except Exception as e:
            logger.error(f"Error getting summary for validation run {run_id}: {str(e)}")
            return None

    def validate_persistence_service(self) -> bool:
        """
        Validate that the persistence service is working correctly.

        Returns:
            bool: True if validation passes, False otherwise
        """
        try:
            # Create a dummy validation run for testing
            dummy_run = ValidationRun(
                id="test-persistence",
                test_queries=[TestQuery(id="q1", text="test query")],
                results=[],
                overall_metrics={},
                start_time=datetime.now(),
                end_time=datetime.now(),
                completed=True
            )

            # Test save
            save_success = self.save_validation_run(dummy_run)
            if not save_success:
                logger.error("Persistence service validation failed: save operation failed")
                return False

            # Test load
            loaded_run = self.load_validation_run("test-persistence")
            if not loaded_run:
                logger.error("Persistence service validation failed: load operation failed")
                return False

            # Test list
            runs_list = self.list_validation_runs()
            if not any(run['id'] == 'test-persistence' for run in runs_list):
                logger.error("Persistence service validation failed: run not found in list")
                return False

            # Test delete
            delete_success = self.delete_validation_run("test-persistence")
            if not delete_success:
                logger.error("Persistence service validation failed: delete operation failed")
                return False

            logger.info("Validation persistence service validation passed")
            return True

        except Exception as e:
            logger.error(f"Error validating persistence service: {str(e)}")
            return False


# Global instance for convenience
persistence_service = ValidationPersistenceService()