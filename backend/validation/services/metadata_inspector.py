"""
Metadata inspection service for the retrieval validation system.

This module provides functionality to inspect and validate metadata
from Qdrant vectors for quality and completeness.
"""
import logging
from typing import List, Dict, Any, Optional
from ..qdrant_client import qdrant_client
from .vector_loader import vector_loader_service
from ..base import ValidationResult
from ..logger import validation_logger as logger


class MetadataInspectorService:
    """
    Service class for inspecting and validating metadata from Qdrant vectors.
    """

    def __init__(self):
        self.qdrant_client = qdrant_client
        self.vector_loader = vector_loader_service

    def initialize(self) -> bool:
        """
        Initialize the service by ensuring dependencies are ready.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        return self.vector_loader.initialize()

    def inspect_metadata_sample(self, limit: int = 10) -> List[Dict[str, Any]]:
        """
        Inspect a sample of metadata from vectors in Qdrant.

        Args:
            limit: Maximum number of metadata entries to inspect

        Returns:
            List of metadata entries with inspection results
        """
        if not self.initialize():
            logger.error("Service not initialized properly")
            return []

        try:
            vectors = self.vector_loader.load_vectors_sample(limit)
            metadata_samples = []

            for vector in vectors:
                payload = vector.get('payload', {})
                metadata_info = {
                    'id': vector.get('id'),
                    'payload_keys': list(payload.keys()),
                    'payload': payload,
                    'has_url': 'url' in payload,
                    'has_title': 'title' in payload,
                    'has_section': 'section' in payload,
                    'has_text': 'text' in payload,
                    'has_ingestion_timestamp': 'ingestion_timestamp' in payload,
                    'has_content_hash': 'content_hash' in payload
                }
                metadata_samples.append(metadata_info)

            logger.info(f"Inspected metadata for {len(metadata_samples)} vectors")
            return metadata_samples

        except Exception as e:
            logger.error(f"Error inspecting metadata samples: {str(e)}")
            return []

    def validate_metadata_completeness(self, limit: int = 100) -> ValidationResult:
        """
        Validate the completeness of metadata fields across a sample of vectors.

        Args:
            limit: Maximum number of vectors to check for completeness

        Returns:
            ValidationResult indicating metadata completeness status
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            vectors = self.vector_loader.load_vectors_sample(limit)
            if not vectors:
                return ValidationResult(
                    success=False,
                    message="No vectors found to inspect for metadata completeness"
                )

            required_fields = ['url', 'title', 'section', 'text', 'ingestion_timestamp', 'content_hash']
            total_vectors = len(vectors)
            complete_vectors = 0
            field_counts = {field: 0 for field in required_fields}

            for vector in vectors:
                payload = vector.get('payload', {})
                is_complete = True

                for field in required_fields:
                    if field in payload and payload[field]:  # Check if field exists and is not empty
                        field_counts[field] += 1
                    else:
                        is_complete = False

                if is_complete:
                    complete_vectors += 1

            completeness_percentage = (complete_vectors / total_vectors) * 100 if total_vectors > 0 else 0

            field_percentages = {
                field: (count / total_vectors) * 100 if total_vectors > 0 else 0
                for field, count in field_counts.items()
            }

            message = f"Metadata completeness: {completeness_percentage:.2f}% ({complete_vectors}/{total_vectors} vectors have all required fields)"

            details = {
                'total_vectors': total_vectors,
                'complete_vectors': complete_vectors,
                'completeness_percentage': completeness_percentage,
                'field_percentages': field_percentages,
                'field_counts': field_counts
            }

            # Consider it successful if at least 90% of vectors have complete metadata
            success = completeness_percentage >= 90.0

            return ValidationResult(
                success=success,
                message=message,
                details=details
            )

        except Exception as e:
            logger.error(f"Error validating metadata completeness: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error validating metadata completeness: {str(e)}"
            )

    def validate_metadata_accuracy(self, sample_size: int = 10) -> ValidationResult:
        """
        Validate the accuracy of metadata by checking for expected patterns/values.

        Args:
            sample_size: Number of metadata entries to validate for accuracy

        Returns:
            ValidationResult indicating metadata accuracy status
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            metadata_samples = self.inspect_metadata_sample(sample_size)
            if not metadata_samples:
                return ValidationResult(
                    success=False,
                    message="No metadata samples found to validate for accuracy"
                )

            total_samples = len(metadata_samples)
            valid_samples = 0

            for sample in metadata_samples:
                payload = sample.get('payload', {})
                is_valid = True

                # Validate URL format
                url = payload.get('url', '')
                if url and not url.startswith(('http://', 'https://')):
                    is_valid = False

                # Validate that text field is not empty
                text = payload.get('text', '')
                if not text.strip():
                    is_valid = False

                # Validate that title is not empty
                title = payload.get('title', '')
                if not title.strip():
                    is_valid = False

                if is_valid:
                    valid_samples += 1

            accuracy_percentage = (valid_samples / total_samples) * 100 if total_samples > 0 else 0

            message = f"Metadata accuracy: {accuracy_percentage:.2f}% ({valid_samples}/{total_samples} samples are valid)"

            details = {
                'total_samples': total_samples,
                'valid_samples': valid_samples,
                'accuracy_percentage': accuracy_percentage
            }

            # Consider it successful if at least 95% of samples are accurate
            success = accuracy_percentage >= 95.0

            return ValidationResult(
                success=success,
                message=message,
                details=details
            )

        except Exception as e:
            logger.error(f"Error validating metadata accuracy: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error validating metadata accuracy: {str(e)}"
            )

    def get_metadata_field_analysis(self) -> ValidationResult:
        """
        Analyze the distribution and quality of different metadata fields.

        Returns:
            ValidationResult with detailed field analysis
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            # Get a larger sample for field analysis
            vectors = self.vector_loader.load_vectors_sample(limit=1000)
            if not vectors:
                return ValidationResult(
                    success=False,
                    message="No vectors found for metadata field analysis"
                )

            # Analyze field presence across all vectors
            field_analysis = {}
            total_vectors = len(vectors)

            for vector in vectors:
                payload = vector.get('payload', {})
                for field, value in payload.items():
                    if field not in field_analysis:
                        field_analysis[field] = {
                            'count': 0,
                            'empty_count': 0,
                            'example_values': []
                        }

                    field_analysis[field]['count'] += 1

                    if not value or (isinstance(value, str) and not value.strip()):
                        field_analysis[field]['empty_count'] += 1

                    # Store example values (limit to 5 examples)
                    if len(field_analysis[field]['example_values']) < 5:
                        field_analysis[field]['example_values'].append(str(value)[:100])  # Limit length

            # Calculate percentages
            for field, analysis in field_analysis.items():
                analysis['presence_percentage'] = (analysis['count'] / total_vectors) * 100
                analysis['empty_percentage'] = (analysis['empty_count'] / analysis['count']) * 100 if analysis['count'] > 0 else 0

            message = f"Analyzed {len(field_analysis)} metadata fields across {total_vectors} vectors"

            details = {
                'total_vectors': total_vectors,
                'field_analysis': field_analysis
            }

            return ValidationResult(
                success=True,
                message=message,
                details=details
            )

        except Exception as e:
            logger.error(f"Error in metadata field analysis: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error in metadata field analysis: {str(e)}"
            )

    def validate_expected_fields_present(self, expected_fields: List[str] = None) -> ValidationResult:
        """
        Validate that expected metadata fields are present in the vectors.

        Args:
            expected_fields: List of field names that should be present in metadata

        Returns:
            ValidationResult indicating if expected fields are present
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        if expected_fields is None:
            expected_fields = ['url', 'title', 'section', 'text', 'ingestion_timestamp', 'content_hash']

        try:
            # Get a sample to check field presence
            metadata_samples = self.inspect_metadata_sample(limit=10)
            if not metadata_samples:
                return ValidationResult(
                    success=False,
                    message="No metadata samples found to validate expected fields"
                )

            # Check the first sample's payload for expected fields
            if metadata_samples:
                first_payload = metadata_samples[0].get('payload', {})
                missing_fields = [field for field in expected_fields if field not in first_payload]

                if missing_fields:
                    message = f"Missing expected metadata fields: {missing_fields}"
                    return ValidationResult(
                        success=False,
                        message=message,
                        details={'missing_fields': missing_fields, 'available_fields': list(first_payload.keys())}
                    )
                else:
                    message = f"All expected metadata fields are present: {expected_fields}"
                    return ValidationResult(
                        success=True,
                        message=message,
                        details={'expected_fields': expected_fields, 'available_fields': list(first_payload.keys())}
                    )
            else:
                return ValidationResult(
                    success=False,
                    message="No metadata samples available to validate expected fields"
                )

        except Exception as e:
            logger.error(f"Error validating expected metadata fields: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error validating expected metadata fields: {str(e)}"
            )


# Global instance for convenience
metadata_inspector_service = MetadataInspectorService()