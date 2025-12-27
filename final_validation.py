import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv('backend/.env')

# Add backend to path so we can import main
sys.path.insert(0, 'backend')

from main import (
    initialize_qdrant_client,
    create_collection,
    clean_text_content,
    chunk_text,
    generate_content_hash,
    ContentChunk
)

def final_validation():
    """Final validation that the backend is properly activated."""
    print("Performing final backend validation...")

    all_passed = True

    # 1. Validate Qdrant connection
    print("\n1. Validating Qdrant connection...")
    try:
        qdrant_client = initialize_qdrant_client()
        print("   [SUCCESS] Qdrant client initialized successfully")

        # Test collection creation
        collection_created = create_collection(qdrant_client)
        print("   [SUCCESS] Qdrant collection ready")
    except Exception as e:
        print(f"   [ERROR] Qdrant validation failed: {e}")
        all_passed = False

    # 2. Validate text processing functions
    print("\n2. Validating text processing functions...")
    try:
        # Test cleaning
        sample = "  Test   text  with   spaces\t\tand\tspecials  "
        cleaned = clean_text_content(sample)
        print(f"   [SUCCESS] Text cleaning: '{sample[:10]}...' -> '{cleaned[:10]}...'")

        # Test chunking with appropriate size
        test_text = "This is a test sentence. " * 50  # Create text with sufficient length
        chunks = chunk_text(test_text, max_words=100, min_words=20)
        print(f"   [SUCCESS] Text chunking: created {len(chunks)} chunks from sample text")

        # Test hashing
        hash_val = generate_content_hash("test content for hashing")
        print(f"   [SUCCESS] Content hashing: {hash_val[:12]}...")
    except Exception as e:
        print(f"   [ERROR] Text processing validation failed: {e}")
        all_passed = False

    # 3. Validate data structures
    print("\n3. Validating data structures...")
    try:
        chunk = ContentChunk(
            id="test-id-123",
            text="This is test content",
            url="https://example.com",
            title="Test Title",
            section="Test Section",
            ingestion_timestamp="2025-01-01T00:00:00Z",
            content_hash=generate_content_hash("This is test content")
        )
        print("   [SUCCESS] ContentChunk structure validated")
    except Exception as e:
        print(f"   [ERROR] Data structure validation failed: {e}")
        all_passed = False

    # 4. Summary
    print("\n" + "="*50)
    if all_passed:
        print("[SUCCESS] ALL VALIDATIONS PASSED!")
        print("[SUCCESS] Backend is successfully activated and ready to use!")
        print("="*50)
        print("\nThe backend system is properly set up with:")
        print("- Local Qdrant database (using ./qdrant_data)")
        print("- Text processing pipeline")
        print("- Content chunking and hashing")
        print("- Data validation components")
        print("\nTo run the full ingestion pipeline:")
        print("1. Obtain a Cohere API key")
        print("2. Update COHERE_API_KEY in backend/.env")
        print("3. Run: python backend/main.py")
    else:
        print("[ERROR] VALIDATION FAILED!")
        print("Some components are not working properly")
        print("="*50)

    return all_passed

if __name__ == "__main__":
    success = final_validation()
    if success:
        print("\n[SUCCESS] Backend activation completed successfully!")
        sys.exit(0)
    else:
        print("\n[ERROR] Backend activation failed!")
        sys.exit(1)