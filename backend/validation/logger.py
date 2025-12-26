"""
Logging and error handling infrastructure for the retrieval validation system.

This module provides standardized logging and error handling utilities
following patterns from main.py.
"""
import logging
import sys
from typing import Any, Dict, Optional
from datetime import datetime


def setup_logging(name: str = __name__, level: int = logging.INFO) -> logging.Logger:
    """
    Set up logging for the validation module.

    Args:
        name: Name for the logger
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid adding multiple handlers if logger already has handlers
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(console_handler)

    return logger


def log_api_error(error: Exception, context: str = "") -> None:
    """
    Log API errors with context.

    Args:
        error: The exception that occurred
        context: Context information about where the error occurred
    """
    logger = setup_logging(__name__)
    logger.error(f"API Error in {context}: {str(error)}")


def handle_validation_error(error: Exception, context: str = "") -> Dict[str, Any]:
    """
    Handle validation errors and return standardized error response.

    Args:
        error: The exception that occurred
        context: Context information about where the error occurred

    Returns:
        Dictionary with error information
    """
    logger = setup_logging(__name__)
    logger.error(f"Validation Error in {context}: {str(error)}")

    return {
        "error": str(error),
        "context": context,
        "timestamp": datetime.now().isoformat(),
        "success": False
    }


def validate_and_log(func):
    """
    Decorator to add error handling and logging to validation functions.

    Args:
        func: Function to wrap with error handling

    Returns:
        Wrapped function with error handling and logging
    """
    def wrapper(*args, **kwargs):
        logger = setup_logging(func.__module__)
        try:
            logger.debug(f"Starting execution of {func.__name__}")
            result = func(*args, **kwargs)
            logger.debug(f"Completed execution of {func.__name__}")
            return result
        except Exception as e:
            logger.error(f"Error in {func.__name__}: {str(e)}")
            return handle_validation_error(e, func.__name__)
    return wrapper


class ValidationError(Exception):
    """
    Custom exception for validation-related errors.
    """
    def __init__(self, message: str, error_code: Optional[str] = None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)

    def __str__(self):
        if self.error_code:
            return f"[{self.error_code}] {self.message}"
        return self.message


class QdrantConnectionError(ValidationError):
    """
    Exception raised when there are issues connecting to Qdrant.
    """
    def __init__(self, message: str = "Failed to connect to Qdrant"):
        super().__init__(message, "QDRANT_CONN_ERROR")


class CohereAPIError(ValidationError):
    """
    Exception raised when there are issues with the Cohere API.
    """
    def __init__(self, message: str = "Cohere API error occurred"):
        super().__init__(message, "COHERE_API_ERROR")


class ConfigurationError(ValidationError):
    """
    Exception raised when there are configuration issues.
    """
    def __init__(self, message: str = "Configuration error occurred"):
        super().__init__(message, "CONFIG_ERROR")


# Global logger instance for the validation module
validation_logger = setup_logging("validation")