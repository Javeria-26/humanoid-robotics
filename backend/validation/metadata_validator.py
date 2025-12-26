"""
Metadata integrity validation for the retrieval validation system.

This module provides functionality to validate the integrity and accuracy
of metadata in retrieved results.
"""
import logging
from typing import List, Dict, Any, Optional
from .base import RetrievedResult, MetadataIntegrityCheck
from .logger import validation_logger as logger


class MetadataValidatorService:
    """
    Service class for validating metadata integrity and accuracy.
    """

    def __init__(self):
        pass

    def validate_metadata_integrity(self, retrieved_result: RetrievedResult,
                                   expected_metadata: Dict[str, Any] = None) -> List[MetadataIntegrityCheck]:
        """
        Validate the integrity of metadata in a retrieved result.

        Args:
            retrieved_result: RetrievedResult object to validate
            expected_metadata: Optional expected metadata values for comparison

        Returns:
            List of MetadataIntegrityCheck objects detailing the validation results
        """
        checks = []

        # Check if metadata exists
        if not retrieved_result.metadata:
            check = MetadataIntegrityCheck(
                result_id=retrieved_result.id,
                field_name="metadata",
                expected_value="non-empty dictionary",
                actual_value="None or empty",
                is_valid=False,
                error_message="Metadata is missing or empty"
            )
            checks.append(check)
            return checks

        # Check for required fields
        required_fields = ['url', 'title', 'section', 'text', 'ingestion_timestamp', 'content_hash']
        for field in required_fields:
            actual_value = retrieved_result.metadata.get(field, None)
            expected_value = expected_metadata.get(field, "any non-empty value") if expected_metadata else "any non-empty value"

            if actual_value is None or actual_value == "":
                check = MetadataIntegrityCheck(
                    result_id=retrieved_result.id,
                    field_name=field,
                    expected_value=expected_value,
                    actual_value=str(actual_value),
                    is_valid=False,
                    error_message=f"Required field '{field}' is missing or empty"
                )
                checks.append(check)
            else:
                # For required fields, if they exist and are not empty, consider them valid
                check = MetadataIntegrityCheck(
                    result_id=retrieved_result.id,
                    field_name=field,
                    expected_value=expected_value,
                    actual_value=str(actual_value),
                    is_valid=True
                )
                checks.append(check)

        # Check URL format
        url = retrieved_result.metadata.get('url', '')
        if url:
            is_valid_url = self._validate_url_format(url)
            expected_value = "valid URL format"
            actual_value = url
            is_valid = is_valid_url

            check = MetadataIntegrityCheck(
                result_id=retrieved_result.id,
                field_name="url_format",
                expected_value=expected_value,
                actual_value=actual_value,
                is_valid=is_valid,
                error_message="URL format is invalid" if not is_valid else ""
            )
            checks.append(check)

        # Check content hash integrity (if expected content is provided)
        if expected_metadata and 'text' in expected_metadata:
            expected_text = expected_metadata['text']
            actual_content_hash = retrieved_result.metadata.get('content_hash', '')
            import hashlib
            expected_content_hash = hashlib.sha256(expected_text.encode('utf-8')).hexdigest()

            is_valid_hash = actual_content_hash == expected_content_hash
            check = MetadataIntegrityCheck(
                result_id=retrieved_result.id,
                field_name="content_hash",
                expected_value=expected_content_hash,
                actual_value=actual_content_hash,
                is_valid=is_valid_hash,
                error_message="Content hash does not match expected value" if not is_valid_hash else ""
            )
            checks.append(check)

        return checks

    def validate_multiple_results_metadata(self, retrieved_results: List[RetrievedResult],
                                         expected_metadata_list: List[Dict[str, Any]] = None) -> List[List[MetadataIntegrityCheck]]:
        """
        Validate metadata integrity for multiple retrieved results.

        Args:
            retrieved_results: List of RetrievedResult objects to validate
            expected_metadata_list: Optional list of expected metadata values

        Returns:
            List of lists, where each inner list contains MetadataIntegrityCheck objects for the corresponding result
        """
        all_checks = []
        for i, result in enumerate(retrieved_results):
            expected_meta = expected_metadata_list[i] if expected_metadata_list and i < len(expected_metadata_list) else None
            checks = self.validate_metadata_integrity(result, expected_meta)
            all_checks.append(checks)

        logger.info(f"Validated metadata integrity for {len(retrieved_results)} results")
        return all_checks

    def calculate_metadata_accuracy(self, all_checks: List[List[MetadataIntegrityCheck]]) -> Dict[str, float]:
        """
        Calculate metadata accuracy metrics from validation checks.

        Args:
            all_checks: List of lists of MetadataIntegrityCheck objects

        Returns:
            Dictionary with metadata accuracy metrics
        """
        if not all_checks:
            return {
                'overall_accuracy': 0.0,
                'field_accuracy': {},
                'total_checks': 0,
                'valid_checks': 0,
                'invalid_checks': 0
            }

        total_checks = 0
        valid_checks = 0
        field_counts = {}
        field_valid_counts = {}

        for result_checks in all_checks:
            for check in result_checks:
                total_checks += 1
                field_name = check.field_name

                # Initialize field counts if not already present
                if field_name not in field_counts:
                    field_counts[field_name] = 0
                    field_valid_counts[field_name] = 0

                field_counts[field_name] += 1
                if check.is_valid:
                    valid_checks += 1
                    field_valid_counts[field_name] += 1

        # Calculate accuracy by field
        field_accuracy = {}
        for field_name in field_counts:
            if field_counts[field_name] > 0:
                field_accuracy[field_name] = field_valid_counts[field_name] / field_counts[field_name]
            else:
                field_accuracy[field_name] = 0.0

        overall_accuracy = valid_checks / total_checks if total_checks > 0 else 0.0

        return {
            'overall_accuracy': overall_accuracy,
            'field_accuracy': field_accuracy,
            'total_checks': total_checks,
            'valid_checks': valid_checks,
            'invalid_checks': total_checks - valid_checks
        }

    def calculate_metadata_completeness(self, retrieved_results: List[RetrievedResult]) -> Dict[str, float]:
        """
        Calculate metadata completeness metrics.

        Args:
            retrieved_results: List of RetrievedResult objects to analyze

        Returns:
            Dictionary with metadata completeness metrics
        """
        if not retrieved_results:
            return {
                'completeness_percentage': 0.0,
                'required_fields_present': {},
                'average_field_count': 0.0
            }

        required_fields = ['url', 'title', 'section', 'text', 'ingestion_timestamp', 'content_hash']
        total_results = len(retrieved_results)
        field_presence = {field: 0 for field in required_fields}
        total_fields_count = 0

        for result in retrieved_results:
            if result.metadata:
                for field in required_fields:
                    if field in result.metadata and result.metadata[field]:
                        field_presence[field] += 1
                total_fields_count += len(result.metadata)

        # Calculate completeness
        total_required_field_instances = total_results * len(required_fields)
        present_required_field_instances = sum(field_presence.values())
        completeness_percentage = present_required_field_instances / total_required_field_instances if total_required_field_instances > 0 else 0.0

        # Calculate field-wise presence
        field_presence_percentage = {
            field: count / total_results if total_results > 0 else 0.0
            for field, count in field_presence.items()
        }

        average_field_count = total_fields_count / total_results if total_results > 0 else 0.0

        return {
            'completeness_percentage': completeness_percentage,
            'required_fields_present': field_presence_percentage,
            'average_field_count': average_field_count,
            'total_results': total_results
        }

    def _validate_url_format(self, url: str) -> bool:
        """
        Validate if the given string has a valid URL format.

        Args:
            url: URL string to validate

        Returns:
            True if URL format is valid, False otherwise
        """
        import re
        # Simple regex for URL validation
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)
        return url_pattern.match(url) is not None

    def validate_metadata_consistency(self, retrieved_results: List[RetrievedResult]) -> Dict[str, Any]:
        """
        Validate consistency of metadata across multiple results.

        Args:
            retrieved_results: List of RetrievedResult objects to validate

        Returns:
            Dictionary with consistency validation results
        """
        if not retrieved_results:
            return {
                'consistent_fields': {},
                'inconsistent_fields': {},
                'consistency_issues': []
            }

        consistency_issues = []
        consistent_fields = {}
        inconsistent_fields = {}

        # Get the first result as a reference
        if retrieved_results:
            reference_metadata = retrieved_results[0].metadata or {}
            reference_keys = set(reference_metadata.keys())

            for i, result in enumerate(retrieved_results[1:], 1):
                current_metadata = result.metadata or {}
                current_keys = set(current_metadata.keys())

                # Check for missing fields in current result
                missing_fields = reference_keys - current_keys
                if missing_fields:
                    consistency_issues.append({
                        'result_id': result.id,
                        'issue': f'Missing fields: {list(missing_fields)}',
                        'compared_to': retrieved_results[0].id
                    })

                # Check for extra fields in current result
                extra_fields = current_keys - reference_keys
                if extra_fields:
                    consistency_issues.append({
                        'result_id': result.id,
                        'issue': f'Extra fields: {list(extra_fields)}',
                        'compared_to': retrieved_results[0].id
                    })

        return {
            'consistent_fields': consistent_fields,
            'inconsistent_fields': inconsistent_fields,
            'consistency_issues': consistency_issues
        }

    def validate_metadata_accuracy_for_field(self, retrieved_results: List[RetrievedResult],
                                           field_name: str,
                                           expected_values: List[str] = None) -> Dict[str, Any]:
        """
        Validate accuracy of a specific metadata field across results.

        Args:
            retrieved_results: List of RetrievedResult objects to validate
            field_name: Name of the field to validate
            expected_values: Optional list of expected values for comparison

        Returns:
            Dictionary with validation results for the specific field
        """
        if not retrieved_results:
            return {
                'field_name': field_name,
                'accuracy': 0.0,
                'valid_count': 0,
                'total_count': 0,
                'accuracy_details': []
            }

        valid_count = 0
        accuracy_details = []

        for i, result in enumerate(retrieved_results):
            actual_value = result.metadata.get(field_name) if result.metadata else None
            expected_value = expected_values[i] if expected_values and i < len(expected_values) else None

            is_valid = True
            error_message = ""

            if actual_value is None:
                is_valid = False
                error_message = f"Field '{field_name}' is missing"
            elif expected_value is not None and str(actual_value) != str(expected_value):
                is_valid = False
                error_message = f"Field '{field_name}' value mismatch: expected '{expected_value}', got '{actual_value}'"
            elif not actual_value and field_name in ['url', 'title', 'section', 'text']:
                # For required fields, empty values are invalid
                is_valid = False
                error_message = f"Field '{field_name}' is empty"

            if is_valid:
                valid_count += 1

            accuracy_details.append({
                'result_id': result.id,
                'field_name': field_name,
                'actual_value': actual_value,
                'expected_value': expected_value,
                'is_valid': is_valid,
                'error_message': error_message
            })

        accuracy = valid_count / len(retrieved_results) if retrieved_results else 0.0

        return {
            'field_name': field_name,
            'accuracy': accuracy,
            'valid_count': valid_count,
            'total_count': len(retrieved_results),
            'accuracy_details': accuracy_details
        }

    def validate_all_metadata(self, retrieved_results: List[RetrievedResult],
                             expected_metadata_list: List[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Perform comprehensive metadata validation on all results.

        Args:
            retrieved_results: List of RetrievedResult objects to validate
            expected_metadata_list: Optional list of expected metadata values

        Returns:
            Dictionary with comprehensive validation results
        """
        # Perform all validations
        all_checks = self.validate_multiple_results_metadata(retrieved_results, expected_metadata_list)
        accuracy_metrics = self.calculate_metadata_accuracy(all_checks)
        completeness_metrics = self.calculate_metadata_completeness(retrieved_results)
        consistency_metrics = self.validate_metadata_consistency(retrieved_results)

        return {
            'accuracy': accuracy_metrics,
            'completeness': completeness_metrics,
            'consistency': consistency_metrics,
            'total_results_validated': len(retrieved_results),
            'timestamp': __import__('datetime').datetime.now().isoformat()
        }


# Global instance for convenience
metadata_validator_service = MetadataValidatorService()