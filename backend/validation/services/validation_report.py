"""
Validation report service for the retrieval validation system.

This module provides functionality to generate validation reports
for vector and metadata access.
"""
import logging
from datetime import datetime
from typing import Dict, Any, List
from ...validation.services.vector_loader import vector_loader_service
from ...validation.services.metadata_inspector import metadata_inspector_service
from ...validation.base import ValidationResult
from ...validation.logger import validation_logger as logger


class ValidationReportService:
    """
    Service class for generating validation reports for vector and metadata access.
    """

    def __init__(self):
        self.vector_loader = vector_loader_service
        self.metadata_inspector = metadata_inspector_service

    def initialize(self) -> bool:
        """
        Initialize the service by ensuring dependencies are ready.

        Returns:
            bool: True if initialization was successful, False otherwise
        """
        return self.vector_loader.initialize() and self.metadata_inspector.initialize()

    def generate_basic_access_report(self) -> ValidationResult:
        """
        Generate a basic validation report for vector and metadata access.

        Returns:
            ValidationResult containing the validation report
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            # Collect all validation information
            validation_results = {}

            # 1. Check Qdrant connection
            connection_valid = self.vector_loader.qdrant_client.validate_connection()
            validation_results['connection_status'] = connection_valid

            # 2. Get vector count
            vector_count = self.vector_loader.get_vector_count()
            validation_results['vector_count'] = vector_count

            # 3. Validate collection exists
            collection_validation = self.vector_loader.validate_collection_exists()
            validation_results['collection_validation'] = collection_validation

            # 4. Check metadata completeness
            metadata_completeness = self.metadata_inspector.validate_metadata_completeness(limit=100)
            validation_results['metadata_completeness'] = metadata_completeness

            # 5. Validate expected fields are present
            expected_fields_validation = self.metadata_inspector.validate_expected_fields_present()
            validation_results['expected_fields_validation'] = expected_fields_validation

            # 6. Get sample metadata inspection
            metadata_samples = self.metadata_inspector.inspect_metadata_sample(limit=5)
            validation_results['metadata_samples'] = metadata_samples

            # Determine overall success based on key metrics
            overall_success = (
                connection_valid and
                vector_count > 0 and
                collection_validation.success and
                metadata_completeness.success
            )

            message = (
                f"Basic validation report generated. "
                f"Connection: {'✓' if connection_valid else '✗'}, "
                f"Vectors: {vector_count}, "
                f"Collection: {'✓' if collection_validation.success else '✗'}, "
                f"Metadata completeness: {'✓' if metadata_completeness.success else '✗'}"
            )

            return ValidationResult(
                success=overall_success,
                message=message,
                details=validation_results
            )

        except Exception as e:
            logger.error(f"Error generating basic validation report: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error generating basic validation report: {str(e)}"
            )

    def generate_detailed_access_report(self) -> ValidationResult:
        """
        Generate a detailed validation report for vector and metadata access.

        Returns:
            ValidationResult containing the detailed validation report
        """
        if not self.initialize():
            return ValidationResult(
                success=False,
                message="Service not initialized properly"
            )

        try:
            # Collect detailed validation information
            report_data = {
                'timestamp': datetime.now().isoformat(),
                'components': {}
            }

            # Component 1: Connection validation
            connection_valid = self.vector_loader.qdrant_client.validate_connection()
            report_data['components']['connection'] = {
                'status': 'connected' if connection_valid else 'disconnected',
                'success': connection_valid
            }

            # Component 2: Collection information
            collection_info = self.vector_loader.qdrant_client.get_collection_info()
            report_data['components']['collection'] = {
                'exists': collection_info is not None,
                'info': collection_info,
                'success': collection_info is not None
            }

            # Component 3: Vector statistics
            vector_count = self.vector_loader.get_vector_count()
            report_data['components']['vectors'] = {
                'count': vector_count,
                'has_vectors': vector_count > 0,
                'success': vector_count >= 0  # Count can be 0 but still successful
            }

            # Component 4: Metadata completeness
            metadata_completeness = self.metadata_inspector.validate_metadata_completeness(limit=1000)
            report_data['components']['metadata_completeness'] = {
                'result': metadata_completeness,
                'success': metadata_completeness.success
            }

            # Component 5: Metadata accuracy
            metadata_accuracy = self.metadata_inspector.validate_metadata_accuracy(sample_size=50)
            report_data['components']['metadata_accuracy'] = {
                'result': metadata_accuracy,
                'success': metadata_accuracy.success
            }

            # Component 6: Field analysis
            field_analysis = self.metadata_inspector.get_metadata_field_analysis()
            report_data['components']['field_analysis'] = {
                'result': field_analysis,
                'success': field_analysis.success
            }

            # Component 7: Sample inspection
            metadata_samples = self.metadata_inspector.inspect_metadata_sample(limit=10)
            report_data['components']['sample_inspection'] = {
                'count': len(metadata_samples),
                'samples': metadata_samples[:3],  # Include first 3 samples as examples
                'success': len(metadata_samples) > 0
            }

            # Calculate overall success
            success_components = [
                report_data['components']['connection']['success'],
                report_data['components']['collection']['success'],
                report_data['components']['vectors']['success'],
                report_data['components']['metadata_completeness']['success'],
                report_data['components']['metadata_accuracy']['success'],
            ]

            overall_success = all(success_components)

            message = f"Detailed validation report generated with {len(success_components)} components checked"

            return ValidationResult(
                success=overall_success,
                message=message,
                details=report_data
            )

        except Exception as e:
            logger.error(f"Error generating detailed validation report: {str(e)}")
            return ValidationResult(
                success=False,
                message=f"Error generating detailed validation report: {str(e)}"
            )

    def export_report_to_dict(self, include_detailed: bool = False) -> Dict[str, Any]:
        """
        Export the validation report as a dictionary.

        Args:
            include_detailed: Whether to include detailed report or just basic

        Returns:
            Dictionary containing the validation report
        """
        if include_detailed:
            result = self.generate_detailed_access_report()
        else:
            result = self.generate_basic_access_report()

        return {
            'success': result.success,
            'message': result.message,
            'timestamp': datetime.now().isoformat(),
            'details': result.details
        }


# Global instance for convenience
validation_report_service = ValidationReportService()