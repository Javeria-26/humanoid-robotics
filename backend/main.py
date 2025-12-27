"""
Document Ingestion System for Docusaurus Content

This system crawls Docusaurus book content, processes it,
generates embeddings using Cohere, and stores them in Qdrant.
"""

import os
import logging
import hashlib
import time
import requests
from urllib.parse import urljoin, urlparse
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import re

# Third-party imports
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ContentChunk:
    """Represents a chunk of content with metadata"""
    id: str
    text: str
    url: str
    title: str
    section: str
    ingestion_timestamp: str
    content_hash: str


# Constants
TARGET_URL = os.getenv("TARGET_URL", "https://humanoid-robotics-dun.vercel.app/")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "rag_embedding")


def exponential_backoff(retries: int, base_delay: float = 1.0) -> float:
    """Calculate delay for exponential backoff"""
    import random
    return base_delay * (2 ** retries) + random.uniform(0, 1)


def handle_api_error(error: Exception, context: str = "") -> None:
    """Log API errors with context"""
    logger.error(f"API Error in {context}: {str(error)}")
    raise error


# Rate limiting tracking
_last_request_time = {}
_min_request_interval = 1.0  # Minimum 1 second between requests to same domain


def respect_rate_limit(url: str) -> None:
    """Implement rate limiting to respect the server"""
    domain = urlparse(url).netloc

    if domain in _last_request_time:
        elapsed = time.time() - _last_request_time[domain]
        if elapsed < _min_request_interval:
            sleep_time = _min_request_interval - elapsed
            logger.debug(f"Rate limiting: sleeping {sleep_time:.2f}s before requesting {url}")
            time.sleep(sleep_time)

    _last_request_time[domain] = time.time()


def safe_request(url: str, **kwargs) -> Optional[requests.Response]:
    """Make a request with retry logic, rate limiting and error handling"""
    max_retries = kwargs.pop('max_retries', 3)
    timeout = kwargs.pop('timeout', 30)

    # Respect rate limiting
    respect_rate_limit(url)

    for attempt in range(max_retries):
        try:
            response = requests.get(url, timeout=timeout, **kwargs)
            response.raise_for_status()
            return response
        except requests.exceptions.RequestException as e:
            if attempt == max_retries - 1:
                logger.error(f"Failed to fetch {url} after {max_retries} attempts: {str(e)}")
                return None
            else:
                delay = exponential_backoff(attempt)
                logger.warning(f"Attempt {attempt + 1} failed for {url}, retrying in {delay:.2f}s: {str(e)}")
                time.sleep(delay)

    return None


def generate_content_hash(content: str) -> str:
    """Generate a hash for content to detect duplicates"""
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def is_content_similar(content1: str, content2: str, threshold: float = 0.9) -> bool:
    """Check if two content strings are similar based on a threshold"""
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
            logger.warning(f"Discarding chunk: {message}")

    logger.info(f"Split text into {len(validated_chunks)} validated chunks (target size 50-1000 words)")
    return validated_chunks


def initialize_cohere_client() -> cohere.Client:
    """
    Initialize and return a Cohere API client.

    Returns:
        Cohere client instance
    """
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        raise ValueError("COHERE_API_KEY environment variable is required")

    logger.info("Initializing Cohere client")
    return cohere.Client(api_key=api_key)


def embed(texts: List[str], model: str = "multilingual-22-12") -> List[List[float]]:
    """
    Generate embeddings for a list of texts using Cohere API.

    Args:
        texts: List of text strings to embed
        model: Cohere model to use for embeddings

    Returns:
        List of embedding vectors (each a list of floats)
    """
    if not texts:
        logger.warning("Empty text list provided to embed function")
        return []

    try:
        cohere_client = initialize_cohere_client()

        # Cohere API has limits on batch size, so we'll process in chunks if needed
        # The maximum batch size for Cohere embeddings is typically 96
        batch_size = 96
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            logger.info(f"Generating embeddings for batch {i//batch_size + 1} of {len(texts)} texts")

            # Apply retry logic for the Cohere API call
            max_retries = 3
            retry_count = 0

            while retry_count < max_retries:
                try:
                    response = cohere_client.embed(
                        texts=batch,
                        model=model,
                        input_type="search_document"  # Using search_document as default input type
                    )

                    batch_embeddings = [embedding for embedding in response.embeddings]
                    all_embeddings.extend(batch_embeddings)
                    break  # Success, exit retry loop

                except Exception as e:
                    retry_count += 1
                    if retry_count >= max_retries:
                        logger.error(f"Failed to generate embeddings for batch {i//batch_size + 1} after {max_retries} attempts: {str(e)}")
                        raise e
                    else:
                        delay = exponential_backoff(retry_count - 1)
                        logger.warning(f"Attempt {retry_count} failed for batch {i//batch_size + 1}, retrying in {delay:.2f}s: {str(e)}")
                        time.sleep(delay)

        logger.info(f"Successfully generated embeddings for {len(all_embeddings)} texts")
        return all_embeddings

    except Exception as e:
        logger.error(f"Error in embed function: {str(e)}")
        raise e


def initialize_qdrant_client() -> QdrantClient:
    """
    Initialize and return a Qdrant client.

    Returns:
        Qdrant client instance
    """
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_local_path = os.getenv("QDRANT_LOCAL_PATH")

    # If local path is specified, use local Qdrant instance
    if qdrant_local_path:
        logger.info(f"Initializing local Qdrant client with path: {qdrant_local_path}")
        client = QdrantClient(path=qdrant_local_path)  # Local instance
    elif not qdrant_url:
        logger.warning("QDRANT_URL not set, using local instance at http://localhost:6333")
        qdrant_url = "http://localhost:6333"
        logger.info(f"Initializing Qdrant client with URL: {qdrant_url}")

        if qdrant_api_key:
            client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=10
            )
        else:
            client = QdrantClient(
                url=qdrant_url,
                timeout=10
            )
    else:
        logger.info(f"Initializing Qdrant client with URL: {qdrant_url}")

        if qdrant_api_key:
            client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
                timeout=10
            )
        else:
            client = QdrantClient(
                url=qdrant_url,
                timeout=10
            )

    # Test the connection
    try:
        client.get_collections()
        logger.info("Successfully connected to Qdrant")
    except Exception as e:
        logger.error(f"Failed to connect to Qdrant: {str(e)}")
        raise e

    return client


def create_collection(client: QdrantClient, collection_name: str = COLLECTION_NAME, vector_size: int = 768) -> bool:
    """
    Create a Qdrant collection for storing embeddings if it doesn't exist.

    Args:
        client: Initialized Qdrant client
        collection_name: Name of the collection to create
        vector_size: Size of the embedding vectors (depends on the model used)

    Returns:
        True if collection was created or already exists, False otherwise
    """
    try:
        # Check if collection already exists
        collections = client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if collection_name in collection_names:
            logger.info(f"Collection '{collection_name}' already exists")
            return True

        # Create the collection with appropriate vector configuration
        # Cohere's multilingual model returns 768-dim vectors
        client.create_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(
                size=vector_size,  # Cohere embeddings are typically 768 dimensions
                distance=models.Distance.COSINE  # Cosine distance is common for embeddings
            )
        )

        # Create a payload index for content_hash to enable filtering
        client.create_payload_index(
            collection_name=collection_name,
            field_name="content_hash",
            field_schema=models.PayloadSchemaType.KEYWORD
        )

        logger.info(f"Successfully created collection '{collection_name}' with vector size {vector_size} and content_hash index")
        return True

    except Exception as e:
        logger.error(f"Error creating collection '{collection_name}': {str(e)}")
        return False


def save_chunk_to_qdrant(client: QdrantClient, chunk: ContentChunk, embedding: List[float], collection_name: str = COLLECTION_NAME) -> bool:
    """
    Save a content chunk with its embedding to Qdrant.

    Args:
        client: Initialized Qdrant client
        chunk: ContentChunk object containing text and metadata
        embedding: Embedding vector for the chunk
        collection_name: Name of the collection to save to

    Returns:
        True if successful, False otherwise
    """
    try:
        # Prepare the point for Qdrant
        points = [
            models.PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "url": chunk.url,
                    "title": chunk.title,
                    "section": chunk.section,
                    "text": chunk.text,
                    "ingestion_timestamp": chunk.ingestion_timestamp,
                    "content_hash": chunk.content_hash
                }
            )
        ]

        # Upsert the point into the collection
        client.upsert(
            collection_name=collection_name,
            points=points
        )

        logger.info(f"Successfully saved chunk to Qdrant: {chunk.id} from {chunk.url}")
        return True

    except Exception as e:
        logger.error(f"Error saving chunk {chunk.id} to Qdrant: {str(e)}")
        return False


def check_content_exists(client: QdrantClient, content_hash: str, collection_name: str = COLLECTION_NAME) -> bool:
    """
    Check if content with the given hash already exists in Qdrant.

    Args:
        client: Initialized Qdrant client
        content_hash: Hash of the content to check
        collection_name: Name of the collection to search in

    Returns:
        True if content exists, False otherwise
    """
    try:
        # Search for points with the specific content hash
        # Updated to use the correct filter format for Qdrant
        hits = client.scroll(
            collection_name=collection_name,
            scroll_filter=models.Filter(
                must=[
                    models.FieldCondition(
                        key="content_hash",
                        match=models.Match(value=content_hash)
                    )
                ]
            ),
            limit=1
        )

        exists = len(hits[0]) > 0  # hits[0] contains the found points
        if exists:
            logger.info(f"Content with hash {content_hash} already exists in collection {collection_name}")
        else:
            logger.info(f"Content with hash {content_hash} not found, will be added")

        return exists

    except Exception as e:
        logger.error(f"Error checking if content exists in Qdrant: {str(e)}")
        # In case of error, assume it doesn't exist to avoid missing content
        return False


def save_chunks_to_qdrant(client: QdrantClient, chunks: List[ContentChunk], embeddings: List[List[float]], collection_name: str = COLLECTION_NAME, skip_duplicates: bool = True) -> bool:
    """
    Save multiple content chunks with their embeddings to Qdrant.

    Args:
        client: Initialized Qdrant client
        chunks: List of ContentChunk objects containing text and metadata
        embeddings: List of embedding vectors corresponding to the chunks
        collection_name: Name of the collection to save to
        skip_duplicates: Whether to skip chunks that already exist based on content hash

    Returns:
        True if successful, False otherwise
    """
    if len(chunks) != len(embeddings):
        logger.error(f"Number of chunks ({len(chunks)}) doesn't match number of embeddings ({len(embeddings)})")
        return False

    try:
        # Filter out duplicates if requested
        if skip_duplicates:
            unique_chunks_and_embeddings = []
            for chunk, embedding in zip(chunks, embeddings):
                if not check_content_exists(client, chunk.content_hash, collection_name):
                    unique_chunks_and_embeddings.append((chunk, embedding))
                else:
                    logger.info(f"Skipping duplicate content for URL: {chunk.url}")

            if len(unique_chunks_and_embeddings) < len(chunks):
                logger.info(f"Filtered out {len(chunks) - len(unique_chunks_and_embeddings)} duplicate chunks")

            chunks, embeddings = zip(*unique_chunks_and_embeddings) if unique_chunks_and_embeddings else ([], [])

            # Convert back to lists
            chunks = list(chunks)
            embeddings = list(embeddings)

        # If no chunks to save after filtering, return True
        if not chunks:
            logger.info("No new chunks to save after duplicate filtering")
            return True

        # Prepare all points for Qdrant
        points = []
        for chunk, embedding in zip(chunks, embeddings):
            point = models.PointStruct(
                id=chunk.id,
                vector=embedding,
                payload={
                    "url": chunk.url,
                    "title": chunk.title,
                    "section": chunk.section,
                    "text": chunk.text,
                    "ingestion_timestamp": chunk.ingestion_timestamp,
                    "content_hash": chunk.content_hash
                }
            )
            points.append(point)

        # Upsert all points into the collection
        client.upsert(
            collection_name=collection_name,
            points=points
        )

        logger.info(f"Successfully saved {len(chunks)} chunks to Qdrant in collection: {collection_name}")
        return True

    except Exception as e:
        logger.error(f"Error saving {len(chunks)} chunks to Qdrant: {str(e)}")
        return False


def extract_nav_urls(soup: BeautifulSoup, base_url: str) -> List[str]:
    """
    Extract URLs from navigation elements in a Docusaurus page.

    Args:
        soup: BeautifulSoup object of the page content
        base_url: Base URL for resolving relative links

    Returns:
        List of navigation URLs
    """
    nav_urls = set()

    # Look for common navigation elements in Docusaurus sites
    nav_selectors = [
        'nav a[href]',  # Navigation elements
        '.navbar a[href]',  # Navbar elements
        '.sidebar a[href]',  # Sidebar navigation
        '.menu a[href]',  # Menu items
        '.table-of-contents a[href]',  # Table of contents
        'header a[href]',  # Header links
        '.pagination a[href]',  # Pagination links
    ]

    base_domain = urlparse(base_url).netloc

    for selector in nav_selectors:
        for link in soup.select(selector):
            href = link.get('href', '')
            if href:
                # Convert relative URLs to absolute URLs
                absolute_url = urljoin(base_url, href)

                # Only add URLs from the same domain
                if urlparse(absolute_url).netloc == base_domain:
                    # Only add URLs that look like content pages
                    parsed = urlparse(absolute_url)
                    if (parsed.path.endswith(('.html', '/')) or not parsed.path) and \
                       not absolute_url.endswith(('.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.rar')):
                        nav_urls.add(absolute_url)

    return list(nav_urls)


def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl the Docusaurus site and discover all accessible pages.

    Args:
        base_url: The base URL of the Docusaurus site to crawl

    Returns:
        List of all discovered URLs
    """
    logger.info(f"Starting to crawl site: {base_url}")
    urls = set()
    visited = set()
    to_visit = [base_url]

    # Parse the base domain for validation
    base_domain = urlparse(base_url).netloc

    while to_visit:
        current_url = to_visit.pop(0)

        # Skip if already visited
        if current_url in visited:
            continue

        visited.add(current_url)
        logger.info(f"Visiting: {current_url}")

        # Fetch the page
        response = safe_request(current_url)
        if not response:
            logger.warning(f"Failed to fetch: {current_url}")
            continue

        # Parse the HTML content
        soup = BeautifulSoup(response.content, 'html.parser')

        # Add current URL to the list
        urls.add(current_url)

        # Extract navigation URLs specifically
        nav_urls = extract_nav_urls(soup, current_url)
        for nav_url in nav_urls:
            if nav_url not in visited and nav_url not in to_visit:
                to_visit.append(nav_url)

        # Find all other links in the page as backup
        for link in soup.find_all('a', href=True):
            href = link['href']

            # Convert relative URLs to absolute URLs
            absolute_url = urljoin(current_url, href)

            # Only add URLs from the same domain that aren't already captured as nav URLs
            if urlparse(absolute_url).netloc == base_domain:
                # Only add URLs that look like content pages (not external links)
                parsed = urlparse(absolute_url)
                if (parsed.path.endswith(('.html', '/')) or not parsed.path) and \
                   not absolute_url.endswith(('.pdf', '.jpg', '.jpeg', '.png', '.gif', '.zip', '.rar')):
                    if absolute_url not in visited and absolute_url not in to_visit:
                        to_visit.append(absolute_url)

    logger.info(f"Found {len(urls)} unique URLs")
    return list(urls)


def extract_text_from_urls(urls: List[str]) -> List[Dict[str, str]]:
    """
    Extract clean text from each URL in the list.

    Args:
        urls: List of URLs to extract text from

    Returns:
        List of dictionaries containing URL, title, and extracted text
    """
    extracted_content = []

    for url in urls:
        logger.info(f"Extracting text from: {url}")

        response = safe_request(url)
        if not response:
            logger.warning(f"Failed to fetch content from: {url}")
            continue

        try:
            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer"]):
                script.decompose()

            # Try to find the main content area in Docusaurus sites
            # Common selectors for Docusaurus content areas
            content_selectors = [
                'main article',  # Main article content
                '.main-wrapper',  # Main wrapper
                '.container',  # Container
                '.theme-doc-markdown',  # Docusaurus markdown content
                '.markdown',  # Markdown content
                '.doc-content',  # Documentation content
                '.content',  # General content area
                'article',  # Article tag
                '.post-content',  # Post content
                '.docs-content',  # Documentation content
            ]

            content_text = ""
            title = ""

            # Get the page title
            title_tag = soup.find('title')
            if title_tag:
                title = title_tag.get_text().strip()
            else:
                # Try to find a h1 tag as the title
                h1_tag = soup.find('h1')
                if h1_tag:
                    title = h1_tag.get_text().strip()

            # Try each selector to find the main content
            for selector in content_selectors:
                content_element = soup.select_one(selector)
                if content_element:
                    content_text = content_element.get_text(separator=' ', strip=True)
                    break

            # If no specific content area found, get the body text
            if not content_text:
                body = soup.find('body')
                if body:
                    content_text = body.get_text(separator=' ', strip=True)

            # Clean up the text
            content_text = re.sub(r'\s+', ' ', content_text).strip()

            if content_text:
                extracted_content.append({
                    'url': url,
                    'title': title,
                    'text': content_text
                })
                logger.info(f"Successfully extracted {len(content_text)} characters from {url}")
            else:
                logger.warning(f"No content extracted from {url}")

        except Exception as e:
            logger.error(f"Error extracting content from {url}: {str(e)}")
            continue

    return extracted_content


def test_url_extraction(base_url: str = TARGET_URL) -> bool:
    """
    Test function to verify URL extraction from a sample Docusaurus site.

    Args:
        base_url: The base URL to test extraction on

    Returns:
        True if test passes, False otherwise
    """
    logger.info(f"Testing URL extraction from: {base_url}")

    try:
        # Get URLs from the site
        urls = get_all_urls(base_url)

        logger.info(f"Found {len(urls)} URLs from {base_url}")

        if len(urls) == 0:
            logger.error("No URLs found - test failed")
            return False

        # Test extracting text from the first few URLs
        sample_urls = urls[:3]  # Just test with first 3 URLs to save time
        extracted_content = extract_text_from_urls(sample_urls)

        logger.info(f"Successfully extracted content from {len(extracted_content)} URLs")

        if len(extracted_content) == 0:
            logger.error("No content extracted - test failed")
            return False

        # Verify that we have meaningful content
        for item in extracted_content:
            if len(item['text']) < 10:  # At least 10 characters of content
                logger.warning(f"URL {item['url']} has very little content: {len(item['text'])} chars")

        logger.info("URL extraction test completed successfully")
        return True

    except Exception as e:
        logger.error(f"Error during URL extraction test: {str(e)}")
        return False


def test_content_cleaning_and_chunking() -> bool:
    """
    Test function to verify content is cleaned and chunked appropriately.

    Returns:
        True if test passes, False otherwise
    """
    logger.info("Testing content cleaning and chunking...")

    # Sample text with various issues that need cleaning
    sample_text = """
    This is a sample text with    multiple   spaces\t\tand special characters!

    It has multiple paragraphs.

    This is the second paragraph with more content. We want to make sure it handles various punctuation marks correctly... Yes, indeed!

    And this is the third paragraph that might be quite long and need to be split into smaller chunks for processing. It contains enough words to exceed the chunking threshold if needed.
    """

    try:
        # Test cleaning
        cleaned = clean_text_content(sample_text)
        logger.info(f"Original length: {len(sample_text)}, Cleaned length: {len(cleaned)}")

        if len(cleaned) > len(sample_text):
            logger.error("Cleaning increased text length unexpectedly")
            return False

        # Test chunking
        chunks = chunk_text(cleaned, max_words=50, min_words=10)
        logger.info(f"Split into {len(chunks)} chunks")

        if len(chunks) == 0:
            logger.error("No chunks created")
            return False

        # Validate each chunk
        for i, chunk in enumerate(chunks):
            is_valid, message = validate_chunk_size(chunk, min_words=10, max_words=100)
            logger.info(f"Chunk {i+1}: {message}")

            if not is_valid:
                logger.error(f"Chunk {i+1} failed validation: {message}")
                return False

        logger.info("Content cleaning and chunking test completed successfully")
        return True

    except Exception as e:
        logger.error(f"Error during content cleaning and chunking test: {str(e)}")
        return False


def test_embedding_and_storage() -> bool:
    """
    Test function to verify embeddings are generated and stored correctly.

    Returns:
        True if test passes, False otherwise
    """
    logger.info("Testing embedding generation and storage...")

    try:
        # Test embedding generation
        test_texts = [
            "This is a test document for embedding.",
            "Another test document with different content.",
            "A third document to test the embedding functionality."
        ]

        # Generate embeddings
        embeddings = embed(test_texts)
        logger.info(f"Generated {len(embeddings)} embeddings")

        if len(embeddings) != len(test_texts):
            logger.error(f"Number of embeddings ({len(embeddings)}) doesn't match number of texts ({len(test_texts)})")
            return False

        if len(embeddings) > 0 and len(embeddings[0]) == 0:
            logger.error("Embedding vectors are empty")
            return False

        # Test Qdrant connection and storage
        try:
            qdrant_client = initialize_qdrant_client()
            logger.info("Qdrant client initialized successfully")

            # Test creating collection
            collection_created = create_collection(qdrant_client, COLLECTION_NAME)
            if not collection_created:
                logger.error("Failed to create collection")
                return False

            # Create test ContentChunk objects
            import uuid
            from datetime import datetime
            test_chunks = []
            for i, text in enumerate(test_texts):
                chunk = ContentChunk(
                    id=str(uuid.uuid4()),
                    text=text,
                    url=f"https://test.com/page{i}",
                    title=f"Test Page {i}",
                    section="Test Section",
                    ingestion_timestamp=datetime.now().isoformat(),
                    content_hash=generate_content_hash(text)
                )
                test_chunks.append(chunk)

            # Test storing chunks
            storage_success = save_chunks_to_qdrant(qdrant_client, test_chunks, embeddings)
            if not storage_success:
                logger.error("Failed to save chunks to Qdrant")
                return False

            logger.info("Embedding and storage test completed successfully")
            return True

        except Exception as e:
            logger.error(f"Error during Qdrant operations: {str(e)}")
            return False

    except Exception as e:
        logger.error(f"Error during embedding and storage test: {str(e)}")
        return False


def sample_query_validation(query_text: str = "test search query", top_k: int = 3) -> bool:
    """
    Create sample queries to validate vector retrieval from Qdrant.

    Args:
        query_text: Text to use for the sample query
        top_k: Number of top results to retrieve

    Returns:
        True if query executes successfully and returns results, False otherwise
    """
    logger.info(f"Testing sample query retrieval: '{query_text}'")

    try:
        # Initialize clients
        qdrant_client = initialize_qdrant_client()

        # Generate embedding for the query
        query_embedding = embed([query_text])

        if not query_embedding or len(query_embedding) == 0:
            logger.error("Failed to generate embedding for query")
            return False

        query_vector = query_embedding[0]

        # Perform the search in Qdrant - using correct method name
        search_results = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=top_k,
            with_payload=True
        ).points

        logger.info(f"Query returned {len(search_results)} results")

        # Validate results
        if len(search_results) == 0:
            logger.warning("Query returned no results, but this may be expected if collection is empty")
            return True  # Return True as the query executed successfully

        # Log some information about the results
        for i, result in enumerate(search_results):
            score = result.score
            payload = result.payload
            text_preview = payload.get('text', '')[:100]  # First 100 chars
            logger.info(f"Result {i+1}: Score {score:.4f}, Text preview: '{text_preview}...'")

        logger.info("Sample query validation completed successfully")
        return True

    except Exception as e:
        logger.error(f"Error during sample query validation: {str(e)}")
        return False


def validate_embedding_quality(texts: List[str], embeddings: List[List[float]], similarity_threshold: float = 0.1) -> bool:
    """
    Validate embedding quality by checking basic properties.

    Args:
        texts: Original texts that were embedded
        embeddings: Generated embeddings
        similarity_threshold: Threshold for similarity checks

    Returns:
        True if embeddings meet quality criteria, False otherwise
    """
    logger.info(f"Validating quality of {len(embeddings)} embeddings")

    if len(texts) != len(embeddings):
        logger.error(f"Text and embedding count mismatch: {len(texts)} vs {len(embeddings)}")
        return False

    if not embeddings:
        logger.warning("No embeddings to validate")
        return True

    # Check that all embeddings have the same dimension
    expected_dim = len(embeddings[0])
    for i, emb in enumerate(embeddings):
        if len(emb) != expected_dim:
            logger.error(f"Embedding {i} has dimension {len(emb)}, expected {expected_dim}")
            return False

    logger.info(f"All embeddings have consistent dimension of {expected_dim}")

    # Check that embeddings are not all zeros
    for i, emb in enumerate(embeddings):
        if all(val == 0.0 for val in emb):
            logger.error(f"Embedding {i} is all zeros")
            return False

    logger.info("Embedding quality validation completed successfully")
    return True


def validate_metadata_accuracy(chunks: List[ContentChunk]) -> bool:
    """
    Validate that metadata in content chunks is accurate and complete.

    Args:
        chunks: List of ContentChunk objects to validate

    Returns:
        True if metadata is accurate, False otherwise
    """
    logger.info(f"Validating metadata accuracy for {len(chunks)} chunks")

    if not chunks:
        logger.warning("No chunks to validate")
        return True

    for i, chunk in enumerate(chunks):
        # Validate required fields are not empty
        if not chunk.id:
            logger.error(f"Chunk {i} has empty ID")
            return False

        if not chunk.text:
            logger.error(f"Chunk {i} has empty text")
            return False

        if not chunk.url:
            logger.error(f"Chunk {i} has empty URL")
            return False

        if not chunk.title:
            logger.warning(f"Chunk {i} has empty title (this may be acceptable)")

        if not chunk.section:
            logger.warning(f"Chunk {i} has empty section (this may be acceptable)")

        if not chunk.ingestion_timestamp:
            logger.error(f"Chunk {i} has empty ingestion timestamp")
            return False

        if not chunk.content_hash:
            logger.error(f"Chunk {i} has empty content hash")
            return False

        # Validate content hash matches the actual content
        expected_hash = generate_content_hash(chunk.text)
        if chunk.content_hash != expected_hash:
            logger.error(f"Chunk {i} content hash mismatch: stored={chunk.content_hash}, calculated={expected_hash}")
            return False

    logger.info("Metadata accuracy validation completed successfully")
    return True


def measure_ingestion_performance(target_url: str = TARGET_URL) -> bool:
    """
    Measure ingestion speed and verify completion within 2 hours.

    Args:
        target_url: The URL to test ingestion performance on

    Returns:
        True if ingestion completes within the time limit, False otherwise
    """
    import time
    start_time = time.time()
    logger.info(f"Starting ingestion performance test for: {target_url}")

    try:
        # Step 1: Get all URLs from the target site
        logger.info("Step 1: Discovering URLs...")
        urls_start = time.time()
        urls = get_all_urls(target_url)
        urls_time = time.time() - urls_start
        logger.info(f"Discovered {len(urls)} URLs in {urls_time:.2f} seconds")

        if not urls:
            logger.error("No URLs found to process")
            return False

        # Step 2: Extract text content from all URLs
        logger.info("Step 2: Extracting text content...")
        extract_start = time.time()
        extracted_content = extract_text_from_urls(urls)
        extract_time = time.time() - extract_start
        logger.info(f"Extracted content from {len(extracted_content)} pages in {extract_time:.2f} seconds")

        if not extracted_content:
            logger.error("No content extracted from URLs")
            return False

        # Step 3: Clean and chunk the content
        logger.info("Step 3: Cleaning and chunking content...")
        chunk_start = time.time()
        all_chunks = []

        for item in extracted_content:
            text = item['text']
            title = item['title']
            url = item['url']

            # Clean the text
            cleaned_text = clean_text_content(text)

            # Chunk the text
            text_chunks = chunk_text(cleaned_text)

            # Create ContentChunk objects
            for i, text_chunk in enumerate(text_chunks):
                import uuid
                chunk_id = str(uuid.uuid4())  # Use UUID for valid Qdrant point ID
                content_hash = generate_content_hash(text_chunk)

                chunk = ContentChunk(
                    id=chunk_id,
                    text=text_chunk,
                    url=url,
                    title=title,
                    section=title,  # Using title as section for now
                    ingestion_timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ'),
                    content_hash=content_hash
                )
                all_chunks.append(chunk)

        chunk_time = time.time() - chunk_start
        logger.info(f"Created {len(all_chunks)} content chunks in {chunk_time:.2f} seconds")

        if not all_chunks:
            logger.error("No content chunks created")
            return False

        # Step 4: Generate embeddings
        logger.info("Step 4: Generating embeddings...")
        embed_start = time.time()
        chunk_texts = [chunk.text for chunk in all_chunks]
        embeddings = embed(chunk_texts)
        embed_time = time.time() - embed_start
        logger.info(f"Generated {len(embeddings)} embeddings in {embed_time:.2f} seconds")

        if len(embeddings) != len(all_chunks):
            logger.error(f"Embedding count mismatch: {len(embeddings)} embeddings for {len(all_chunks)} chunks")
            return False

        # Step 5: Initialize Qdrant client and create collection
        logger.info("Step 5: Initializing Qdrant and creating collection...")
        qdrant_start = time.time()
        qdrant_client = initialize_qdrant_client()
        collection_created = create_collection(qdrant_client, COLLECTION_NAME)
        if not collection_created:
            logger.error("Failed to create collection")
            return False
        qdrant_init_time = time.time() - qdrant_start
        logger.info(f"Qdrant initialization completed in {qdrant_init_time:.2f} seconds")

        # Step 6: Save chunks to Qdrant
        logger.info("Step 6: Saving chunks to Qdrant...")
        save_start = time.time()
        storage_success = save_chunks_to_qdrant(qdrant_client, all_chunks, embeddings)
        save_time = time.time() - save_start
        logger.info(f"Saved chunks to Qdrant in {save_time:.2f} seconds")

        if not storage_success:
            logger.error("Failed to save chunks to Qdrant")
            return False

        # Calculate total time
        total_time = time.time() - start_time
        logger.info(f"Ingestion completed successfully!")
        logger.info(f"Total time: {total_time:.2f} seconds ({total_time/60:.2f} minutes)")
        logger.info(f"Performance summary - URLs: {urls_time:.2f}s, Extract: {extract_time:.2f}s, Chunk: {chunk_time:.2f}s, Embed: {embed_time:.2f}s, Qdrant: {qdrant_init_time+save_time:.2f}s")

        # Check if within 2-hour limit (7200 seconds)
        time_limit = 2 * 60 * 60  # 2 hours in seconds
        if total_time <= time_limit:
            logger.info(f"Ingestion completed within time limit: {total_time:.2f}s < {time_limit}s")
            return True
        else:
            logger.error(f"Ingestion exceeded time limit: {total_time:.2f}s > {time_limit}s")
            return False

    except Exception as e:
        logger.error(f"Error during ingestion performance test: {str(e)}")
        return False


def test_edge_cases() -> bool:
    """
    Test edge cases like empty content, network failures, etc.

    Returns:
        True if all edge case tests pass, False otherwise
    """
    logger.info("Testing edge cases...")

    try:
        # Test 1: Empty text handling
        logger.info("Testing empty text handling...")
        empty_texts = ["", "   ", "\n\n", " \t "]
        for empty_text in empty_texts:
            chunks = chunk_text(empty_text)
            if len(chunks) != 0:
                logger.error(f"Empty text '{empty_text}' resulted in {len(chunks)} chunks, expected 0")
                return False
        logger.info("Empty text handling test passed")

        # Test 2: Very large text handling
        logger.info("Testing large text handling...")
        large_text = "This is a test sentence. " * 10000  # Very large text
        chunks = chunk_text(large_text, max_words=500)  # Use larger chunk size for this test
        if len(chunks) == 0:
            logger.error("Large text resulted in 0 chunks")
            return False
        logger.info(f"Large text test passed, created {len(chunks)} chunks")

        # Test 3: Text with special characters
        logger.info("Testing text with special characters...")
        special_text = "Test with special chars: \x00\x01\x02 and unicode: ñáéíóú"
        cleaned = clean_text_content(special_text)
        if not isinstance(cleaned, str):
            logger.error("Special character text cleaning failed")
            return False
        logger.info("Special character handling test passed")

        # Test 4: Invalid URL handling in safe_request
        logger.info("Testing invalid URL handling...")
        invalid_response = safe_request("http://invalid-url-that-should-not-exist-12345.com", max_retries=1)
        if invalid_response is not None:
            logger.warning("Invalid URL somehow succeeded (this is unexpected but not necessarily an error)")
        else:
            logger.info("Invalid URL correctly returned None")

        # Test 5: Empty embeddings list
        logger.info("Testing empty embeddings list...")
        try:
            result = embed([])
            if len(result) != 0:
                logger.error("Empty text list should return empty embeddings list")
                return False
            logger.info("Empty embeddings list test passed")
        except Exception as e:
            logger.error(f"Error testing empty embeddings: {str(e)}")
            return False

        logger.info("All edge case tests completed successfully")
        return True

    except Exception as e:
        logger.error(f"Error during edge case testing: {str(e)}")
        return False


def main():
    """
    Main function to orchestrate the complete ingestion pipeline.
    """
    logger.info("Starting document ingestion pipeline...")

    try:
        # Step 1: Get all URLs from the target site
        logger.info(f"Discovering URLs from: {TARGET_URL}")
        urls = get_all_urls(TARGET_URL)

        if not urls:
            logger.error("No URLs found to process")
            return False

        logger.info(f"Discovered {len(urls)} URLs to process")

        # Step 2: Extract text content from all URLs
        logger.info("Extracting text content from URLs...")
        extracted_content = extract_text_from_urls(urls)

        if not extracted_content:
            logger.error("No content extracted from URLs")
            return False

        logger.info(f"Extracted content from {len(extracted_content)} pages")

        # Step 3: Clean and chunk the content
        logger.info("Cleaning and chunking content...")
        all_chunks = []

        for item in extracted_content:
            text = item['text']
            title = item['title']
            url = item['url']

            # Clean the text
            cleaned_text = clean_text_content(text)

            # Chunk the text
            text_chunks = chunk_text(cleaned_text)

            # Create ContentChunk objects
            for i, text_chunk in enumerate(text_chunks):
                import uuid
                chunk_id = str(uuid.uuid4())  # Use UUID for valid Qdrant point ID
                content_hash = generate_content_hash(text_chunk)

                chunk = ContentChunk(
                    id=chunk_id,
                    text=text_chunk,
                    url=url,
                    title=title,
                    section=title,  # Using title as section for now
                    ingestion_timestamp=time.strftime('%Y-%m-%dT%H:%M:%SZ'),
                    content_hash=content_hash
                )
                all_chunks.append(chunk)

        logger.info(f"Created {len(all_chunks)} content chunks")

        if not all_chunks:
            logger.error("No content chunks created")
            return False

        # Step 4: Extract text from chunks for embedding
        chunk_texts = [chunk.text for chunk in all_chunks]

        # Step 5: Generate embeddings
        logger.info("Generating embeddings...")
        embeddings = embed(chunk_texts)

        if len(embeddings) != len(all_chunks):
            logger.error(f"Embedding count mismatch: {len(embeddings)} embeddings for {len(all_chunks)} chunks")
            return False

        logger.info(f"Generated embeddings for {len(embeddings)} chunks")

        # Step 6: Initialize Qdrant client and create collection
        logger.info("Initializing Qdrant client...")
        qdrant_client = initialize_qdrant_client()

        logger.info(f"Creating collection: {COLLECTION_NAME}")
        collection_created = create_collection(qdrant_client, COLLECTION_NAME)
        if not collection_created:
            logger.error("Failed to create collection")
            return False

        # Step 7: Save chunks to Qdrant
        logger.info(f"Saving {len(all_chunks)} chunks to Qdrant...")
        storage_success = save_chunks_to_qdrant(qdrant_client, all_chunks, embeddings)
        if not storage_success:
            logger.error("Failed to save chunks to Qdrant")
            return False

        logger.info("Document ingestion pipeline completed successfully!")
        return True

    except Exception as e:
        logger.error(f"Error in main ingestion pipeline: {str(e)}")
        return False


if __name__ == "__main__":
    success = main()
    if success:
        logger.info("Pipeline completed successfully!")
        exit(0)
    else:
        logger.error("Pipeline failed!")
        exit(1)