import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv('backend/.env')

# Get Qdrant URL from environment or use default
qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
qdrant_api_key = os.getenv("QDRANT_API_KEY")

print(f"Attempting to connect to Qdrant at: {qdrant_url}")

try:
    if qdrant_api_key:
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, timeout=10)
    else:
        client = QdrantClient(url=qdrant_url, timeout=10)

    # Test connection by getting collections
    collections = client.get_collections()
    print("✓ Successfully connected to Qdrant!")
    print(f"Available collections: {[c.name for c in collections.collections]}")

except Exception as e:
    print(f"✗ Failed to connect to Qdrant: {e}")
    print("\nTo run Qdrant locally, you have a few options:")
    print("1. Install Docker Desktop and run: docker run -p 6333:6333 qdrant/qdrant")
    print("2. Download Qdrant from: https://qdrant.tech/documentation/quick-start/")
    print("3. Use Qdrant Cloud: https://cloud.qdrant.io/")