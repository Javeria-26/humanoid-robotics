# Quickstart Guide: Website Ingestion & Vector Indexing

## Prerequisites

- Python 3.11+
- uv package manager
- Cohere API key
- Qdrant instance (local or cloud)

## Setup

1. **Create the project structure**:
   ```bash
   mkdir backend
   cd backend
   uv init
   ```

2. **Install dependencies**:
   ```bash
   uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
   ```

3. **Set up environment variables**:
   Create a `.env` file in the backend directory:
   ```
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here  # if using cloud
   ```

## Configuration

The ingestion system expects the following environment variables:
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (optional if using local instance)

## Running the Ingestion Pipeline

1. **Create the main.py file** with the implementation as specified

2. **Execute the pipeline**:
   ```bash
   cd backend
   python main.py
   ```

## Expected Output

- Crawls all pages from the target Docusaurus site (https://humanoid-robotics-dun.vercel.app/)
- Processes content into semantic chunks
- Generates embeddings and stores them in Qdrant collection named 'rag_embedding'
- Provides progress updates during execution
- Validates results with sample queries

## Verification

After running the pipeline, you can verify:
- Qdrant collection 'rag_embedding' contains the expected number of vectors
- Each vector has associated metadata (URL, title, section)
- Sample queries return relevant results
- No duplicate content was processed unnecessarily