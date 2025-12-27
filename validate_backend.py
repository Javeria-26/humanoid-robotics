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
    test_content_cleaning_and_chunking,
    test_edge_cases,
    validate_embedding_quality,
    validate_metadata_accuracy,
    ContentChunk
)

def validate_backend():
    """Comprehensive validation of the backend system."""
    print("Performing comprehensive backend validation...")

    validation_results = []

    # 1. Validate Qdrant connection and collection
    print("\n1. Testing Qdrant connection and collection...")
    try:
        qdrant_client = initialize_qdrant_client()
        collection_created = create_collection(qdrant_client)
        if collection_created:
            print("   [SUCCESS] Qdrant connection and collection validated")
            validation_results.append(("Qdrant Connection", True))
        else:
            print("   [ERROR] Failed to create collection")
            validation_results.append(("Qdrant Connection", False))
    except Exception as e:
        print(f"   [ERROR] Qdrant validation failed: {e}")
        validation_results.append(("Qdrant Connection", False))

    # 2. Test content processing functions
    print("\n2. Testing content processing functions...")
    try:
        success = test_content_cleaning_and_chunking()
        if success:
            print("   [SUCCESS] Content cleaning and chunking validated")
            validation_results.append(("Content Processing", True))
        else:
            print("   [ERROR] Content processing validation failed")
            validation_results.append(("Content Processing", False))
    except Exception as e:
        print(f"   [ERROR] Content processing test failed: {e}")
        validation_results.append(("Content Processing", False))

    # 3. Test edge cases
    print("\n3. Testing edge cases...")
    try:
        success = test_edge_cases()
        if success:
            print("   [SUCCESS] Edge cases validated")
            validation_results.append(("Edge Cases", True))
        else:
            print("   [ERROR] Edge cases validation failed")
            validation_results.append(("Edge Cases", False))
    except Exception as e:
        print(f"   [ERROR] Edge cases test failed: {e}")
        validation_results.append(("Edge Cases", False))

    # 4. Test metadata validation with mock data
    print("\n4. Testing metadata validation...")
    try:
        # Create mock content chunks for validation
        mock_chunks = [
            ContentChunk(
                id="test-1",
                text="This is a test content chunk for validation.",
                url="https://test.com/page1",
                title="Test Page 1",
                section="Test Section",
                ingestion_timestamp="2025-01-01T00:00:00Z",
                content_hash="test-hash-1"
            )
        ]
        success = validate_metadata_accuracy(mock_chunks)
        if success:
            print("   [SUCCESS] Metadata validation passed")
            validation_results.append(("Metadata Validation", True))
        else:
            print("   [ERROR] Metadata validation failed")
            validation_results.append(("Metadata Validation", False))
    except Exception as e:
        print(f"   [ERROR] Metadata validation test failed: {e}")
        validation_results.append(("Metadata Validation", False))

    # 5. Test embedding validation with mock data
    print("\n5. Testing embedding validation...")
    try:
        # Test with mock embeddings data
        mock_texts = ["Test embedding validation"]
        mock_embeddings = [[0.1, 0.2, 0.3]]  # Mock embedding vector
        success = validate_embedding_quality(mock_texts, mock_embeddings)
        if success:
            print("   [SUCCESS] Embedding validation passed")
            validation_results.append(("Embedding Validation", True))
        else:
            print("   [ERROR] Embedding validation failed")
            validation_results.append(("Embedding Validation", False))
    except Exception as e:
        print(f"   [ERROR] Embedding validation test failed: {e}")
        validation_results.append(("Embedding Validation", False))

    # Summary
    print("\n" + "="*50)
    print("VALIDATION SUMMARY:")
    print("="*50)

    passed = 0
    total = len(validation_results)

    for test_name, result in validation_results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"{status} {test_name}")
        if result:
            passed += 1

    print(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 ALL VALIDATIONS PASSED! Backend is fully operational.")
        print("\nTo run the full ingestion pipeline:")
        print("1. Set your COHERE_API_KEY in backend/.env")
        print("2. Run: python backend/main.py")
        return True
    else:
        print(f"\n[ERROR] {total - passed} validation(s) failed. Backend needs attention.")
        return False

if __name__ == "__main__":
    success = validate_backend()
    if success:
        print("\nBackend validation completed successfully!")
        sys.exit(0)
    else:
        print("\nBackend validation failed!")
        sys.exit(1)