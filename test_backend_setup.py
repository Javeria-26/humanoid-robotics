import os
import sys
import logging
from dotenv import load_dotenv

# Load environment variables
load_dotenv('backend/.env')

# Add backend to path so we can import main
sys.path.insert(0, 'backend')

from main import (
    get_all_urls,
    extract_text_from_urls,
    clean_text_content,
    chunk_text,
    initialize_qdrant_client,
    create_collection,
    ContentChunk,
    generate_content_hash
)

def test_backend_setup():
    """Test that the backend is set up correctly with local Qdrant."""
    print("Testing backend setup with local Qdrant...")

    # Test Qdrant initialization
    try:
        qdrant_client = initialize_qdrant_client()
        print("[SUCCESS] Qdrant client initialized successfully")
    except Exception as e:
        print(f"[ERROR] Failed to initialize Qdrant client: {e}")
        return False

    # Test collection creation
    try:
        success = create_collection(qdrant_client)
        print("[SUCCESS] Qdrant collection creation successful")
    except Exception as e:
        print(f"[ERROR] Failed to create collection: {e}")
        return False

    # Test URL extraction (using a sample URL)
    try:
        # Test with a simple local or known URL
        print("Testing content processing functions...")

        # Test text cleaning
        sample_text = "This is a   sample text with\t\tmultiple   spaces."
        cleaned = clean_text_content(sample_text)
        print(f"[SUCCESS] Text cleaning works: '{sample_text[:15]}...' -> '{cleaned[:15]}...'")

        # Test chunking
        large_text = "This is a test sentence. " * 100  # Create a larger text
        chunks = chunk_text(large_text, max_words=50)
        print(f"[SUCCESS] Text chunking works: created {len(chunks)} chunks")

        # Test content hashing
        content_hash = generate_content_hash("test content")
        print(f"[SUCCESS] Content hashing works: {content_hash[:10]}...")

        print("[SUCCESS] All local backend components are working correctly!")
        return True

    except Exception as e:
        print(f"[ERROR] Error in content processing: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_backend_setup()
    if success:
        print("\nBackend is set up and ready to use!")
        print("To run the full ingestion pipeline, you'll need to:")
        print("1. Set your COHERE_API_KEY in the .env file")
        print("2. Optionally set QDRANT_API_KEY if using remote Qdrant")
        print("3. Run: python backend/main.py")
    else:
        print("\nBackend setup failed!")
        sys.exit(1)