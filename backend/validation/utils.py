"""
Utility functions for the retrieval validation system.

This module provides utility functions for content hashing, text processing,
and other common operations following patterns from main.py.
"""
import hashlib
import re
from typing import List, Tuple
import time


def generate_content_hash(content: str) -> str:
    """
    Generate a hash for content to detect duplicates.

    Args:
        content: Content string to hash

    Returns:
        SHA256 hash of the content
    """
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def is_content_similar(content1: str, content2: str, threshold: float = 0.9) -> bool:
    """
    Check if two content strings are similar based on a threshold.

    Args:
        content1: First content string
        content2: Second content string
        threshold: Similarity threshold (0.0 to 1.0)

    Returns:
        True if content is similar based on the threshold
    """
    if not content1 or not content2:
        return False

    # Simple similarity check using common words
    words1 = set(content1.lower().split())
    words2 = set(content2.lower().split())

    if not words1 or not words2:
        return False

    common_words = words1.intersection(words2)
    similarity = len(common_words) / min(len(words1), len(words2))

    return similarity >= threshold


def clean_text_content(text: str) -> str:
    """
    Clean extracted text content by removing extra whitespace, special characters, etc.

    Args:
        text: Raw text content to clean

    Returns:
        Cleaned text content
    """
    if not text:
        return ""

    # Remove extra whitespace and normalize
    cleaned = re.sub(r'\s+', ' ', text)
    cleaned = cleaned.strip()

    # Remove special characters if needed, but preserve essential punctuation
    # This is a basic cleaning - can be enhanced based on specific requirements
    cleaned = re.sub(r'[ \t]+', ' ', cleaned)  # Multiple spaces/tabs to single space
    cleaned = re.sub(r'\n\s*\n', '\n\n', cleaned)  # Multiple newlines to max 2

    return cleaned


def validate_chunk_size(chunk: str, min_words: int = 50, max_words: int = 1000) -> Tuple[bool, str]:
    """
    Validate that a chunk meets size requirements.

    Args:
        chunk: Text chunk to validate
        min_words: Minimum number of words required
        max_words: Maximum number of words allowed

    Returns:
        Tuple of (is_valid, message)
    """
    word_count = len(chunk.split())

    if word_count < min_words:
        return False, f"Chunk has only {word_count} words, less than minimum {min_words}"
    elif word_count > max_words:
        return False, f"Chunk has {word_count} words, exceeding maximum {max_words}"
    else:
        return True, f"Chunk has {word_count} words, within range [{min_words}, {max_words}]"


def chunk_text(text: str, max_words: int = 800, min_words: int = 100) -> List[str]:
    """
    Split text into chunks of appropriate size for embedding generation.

    Args:
        text: Text to chunk
        max_words: Maximum number of words per chunk (default 800, within 500-1000 range)
        min_words: Minimum number of words before forcing a chunk (default 100)

    Returns:
        List of text chunks
    """
    if not text:
        return []

    # Split text into sentences
    sentences = re.split(r'(?<=[.!?]) +', text)
    chunks = []
    current_chunk = []
    current_word_count = 0

    for sentence in sentences:
        sentence_words = len(sentence.split())

        # If adding this sentence would exceed max words, start a new chunk
        if current_word_count + sentence_words > max_words and current_chunk:
            chunk_text = ' '.join(current_chunk)
            if len(chunk_text.strip()) > 0:  # Only add non-empty chunks
                chunks.append(chunk_text)
            current_chunk = [sentence]
            current_word_count = sentence_words
        else:
            current_chunk.append(sentence)
            current_word_count += sentence_words

    # Add the last chunk if it has content
    if current_chunk:
        chunk_text = ' '.join(current_chunk)
        if len(chunk_text.strip()) > 0:
            chunks.append(chunk_text)

    # Further split any chunks that are still too large
    final_chunks = []
    for chunk in chunks:
        if len(chunk.split()) > max_words:
            # If chunk is still too big, split by paragraphs
            paragraphs = chunk.split('\n\n')
            temp_chunk = ""
            temp_word_count = 0

            for paragraph in paragraphs:
                para_word_count = len(paragraph.split())

                if temp_word_count + para_word_count <= max_words:
                    temp_chunk += paragraph + "\n\n"
                    temp_word_count += para_word_count
                else:
                    if temp_chunk.strip():
                        final_chunks.append(temp_chunk.strip())
                    temp_chunk = paragraph + "\n\n"
                    temp_word_count = para_word_count

            if temp_chunk.strip():
                final_chunks.append(temp_chunk.strip())
        else:
            final_chunks.append(chunk)

    # Filter out any empty chunks and validate size
    validated_chunks = []
    for chunk in final_chunks:
        is_valid, message = validate_chunk_size(chunk, min_words=50, max_words=1000)
        if is_valid:
            validated_chunks.append(chunk)
        else:
            print(f"Discarding chunk: {message}")  # Using print instead of logger to avoid circular import

    print(f"Split text into {len(validated_chunks)} validated chunks (target size 50-1000 words)")
    return validated_chunks


def format_time_delta(seconds: float) -> str:
    """
    Format a time duration in seconds to a human-readable format.

    Args:
        seconds: Time duration in seconds

    Returns:
        Formatted time string (e.g., "1.23s", "456ms", "78ms")
    """
    if seconds >= 1:
        return f"{seconds:.2f}s"
    elif seconds >= 0.001:
        return f"{seconds * 1000:.0f}ms"
    else:
        return f"{seconds * 1000000:.0f}μs"


def time_function(func):
    """
    Decorator to time function execution.

    Args:
        func: Function to time

    Returns:
        Wrapped function that returns (result, execution_time_in_seconds)
    """
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        return result, execution_time
    return wrapper