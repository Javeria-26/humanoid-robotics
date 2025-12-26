"""
Command-line interface for the retrieval validation system.

This module provides a CLI for running validation tasks.
"""
import argparse
import sys
import json
from datetime import datetime
from .services.vector_loader import vector_loader_service
from .services.metadata_inspector import metadata_inspector_service
from .services.validation_report import validation_report_service
from .qdrant_client import qdrant_client
from .logger import validation_logger as logger


def validate_connection(args):
    """Validate connection to Qdrant."""
    print("Validating Qdrant connection...")
    success = qdrant_client.validate_connection()
    if success:
        print("✓ Qdrant connection is valid")
        return True
    else:
        print("✗ Qdrant connection failed")
        return False


def validate_vectors(args):
    """Validate vector access and count."""
    print("Validating vector access...")
    if not vector_loader_service.initialize():
        print("✗ Failed to initialize vector loader service")
        return False

    count = vector_loader_service.get_vector_count()
    print(f"✓ Found {count} vectors in collection")
    return count >= 0


def validate_metadata(args):
    """Validate metadata completeness and accuracy."""
    print("Validating metadata...")
    if not metadata_inspector_service.initialize():
        print("✗ Failed to initialize metadata inspector service")
        return False

    # Validate metadata completeness
    completeness_result = metadata_inspector_service.validate_metadata_completeness(limit=100)
    print(f"✓ Metadata completeness: {completeness_result.message}")

    if completeness_result.details:
        completeness_pct = completeness_result.details.get('completeness_percentage', 0)
        print(f"  - {completeness_pct:.2f}% of vectors have complete metadata")

    # Validate metadata accuracy
    accuracy_result = metadata_inspector_service.validate_metadata_accuracy(sample_size=50)
    print(f"✓ Metadata accuracy: {accuracy_result.message}")

    if accuracy_result.details:
        accuracy_pct = accuracy_result.details.get('accuracy_percentage', 0)
        print(f"  - {accuracy_pct:.2f}% of samples have accurate metadata")

    return completeness_result.success and accuracy_result.success


def generate_report(args):
    """Generate a validation report."""
    print("Generating validation report...")
    if not validation_report_service.initialize():
        print("✗ Failed to initialize validation report service")
        return False

    if args.detailed:
        result = validation_report_service.generate_detailed_access_report()
    else:
        result = validation_report_service.generate_basic_access_report()

    print(f"Report generated: {'✓' if result.success else '✗'}")
    print(f"Message: {result.message}")

    if result.details:
        if args.output:
            # Save to file
            with open(args.output, 'w') as f:
                json.dump(result.details, f, indent=2, default=str)
            print(f"Report saved to {args.output}")
        else:
            print("Details:")
            print(json.dumps(result.details, indent=2, default=str))

    return result.success


def run_all_validations(args):
    """Run all validation checks."""
    print("Running all validations...")
    print("="*50)

    results = []

    # Validate connection
    print("\n1. Connection Validation:")
    results.append(validate_connection(args))

    # Validate vectors
    print("\n2. Vector Validation:")
    results.append(validate_vectors(args))

    # Validate metadata
    print("\n3. Metadata Validation:")
    results.append(validate_metadata(args))

    # Generate report
    print("\n4. Report Generation:")
    args.detailed = getattr(args, 'detailed', False)
    args.output = getattr(args, 'output', None)
    results.append(generate_report(args))

    print("\n" + "="*50)
    print(f"Summary: {sum(results)}/{len(results)} checks passed")

    overall_success = all(results)
    print(f"Overall result: {'✓ All validations passed' if overall_success else '✗ Some validations failed'}")

    return overall_success


def main():
    """Main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Retrieval Validation CLI - Validate Qdrant vector retrieval system"
    )

    # Create subparsers for different commands
    subparsers = parser.add_subparsers(dest='command', help='Available commands')

    # Connection validation command
    conn_parser = subparsers.add_parser('connection', help='Validate Qdrant connection')
    conn_parser.set_defaults(func=validate_connection)

    # Vector validation command
    vector_parser = subparsers.add_parser('vectors', help='Validate vector access')
    vector_parser.set_defaults(func=validate_vectors)

    # Metadata validation command
    metadata_parser = subparsers.add_parser('metadata', help='Validate metadata quality')
    metadata_parser.set_defaults(func=validate_metadata)

    # Report generation command
    report_parser = subparsers.add_parser('report', help='Generate validation report')
    report_parser.add_argument('--detailed', action='store_true', help='Generate detailed report')
    report_parser.add_argument('--output', type=str, help='Output file for the report')
    report_parser.set_defaults(func=generate_report)

    # All validations command
    all_parser = subparsers.add_parser('all', help='Run all validations')
    all_parser.add_argument('--detailed', action='store_true', help='Generate detailed report')
    all_parser.add_argument('--output', type=str, help='Output file for the report')
    all_parser.set_defaults(func=run_all_validations)

    # If no arguments provided, show help
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    # Parse arguments and run the appropriate function
    args = parser.parse_args()

    try:
        success = args.func(args)
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"Error running command: {str(e)}")
        logger.error(f"CLI error: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()