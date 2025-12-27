import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

# Load environment variables
load_dotenv('backend/.env')

# Get Qdrant configuration from environment
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_local_path = os.getenv("QDRANT_LOCAL_PATH")

print("Qdrant Configuration:")
print(f"  QDRANT_URL: {qdrant_url}")
print(f"  QDRANT_API_KEY: {'Set' if qdrant_api_key else 'Not set'}")
print(f"  QDRANT_LOCAL_PATH: {qdrant_local_path}")

try:
    # If local path is specified, use local Qdrant instance
    if qdrant_local_path:
        print(f"Attempting to connect to local Qdrant at: {qdrant_local_path}")
        client = QdrantClient(path=qdrant_local_path)
    elif not qdrant_url:
        print("QDRANT_URL not set, attempting to connect to default local instance at http://localhost:6333")
        if qdrant_api_key:
            client = QdrantClient(url="http://localhost:6333", api_key=qdrant_api_key, timeout=10)
        else:
            client = QdrantClient(url="http://localhost:6333", timeout=10)
    else:
        print(f"Attempting to connect to Qdrant at: {qdrant_url}")
        if qdrant_api_key:
            client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key, timeout=10)
        else:
            client = QdrantClient(url=qdrant_url, timeout=10)

    # Test connection by getting collections
    collections = client.get_collections()
    print("Successfully connected to Qdrant!")
    print(f"Available collections: {[c.name for c in collections.collections]}")

except Exception as e:
    print(f"Failed to connect to Qdrant: {e}")
    print("Make sure you have:")
    print("1. Qdrant running locally (if using URL) OR")
    print("2. A valid local path specified (if using local storage)")
    print("For local storage, ensure QDRANT_LOCAL_PATH is set in your .env file.")