# Quickstart Guide: RAG Agent & API Backend

## Prerequisites

- Python 3.11+
- pip package manager
- Access to OpenAI API key
- Qdrant vector database (local or cloud instance)

## Setup

### 1. Clone and Navigate to Project
```bash
git clone <repository-url>
cd backend
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
# Or install required packages:
pip install fastapi openai qdrant-client pydantic python-dotenv uvicorn
```

### 4. Set Environment Variables
Create a `.env` file with the following:
```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here  # if using cloud
QDRANT_HOST=localhost  # if using local instance
QDRANT_PORT=6333
```

## Running the Service

### 1. Start the API Server
```bash
uvicorn src.main:app --reload --port 8000
```

### 2. Verify Service is Running
Visit `http://localhost:8000/health` to check if the service is running.

## Using the API

### Query Example
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of embodied cognition?",
    "include_citations": true,
    "max_results": 5
  }'
```

### Expected Response
```json
{
  "query": "What are the key principles of embodied cognition?",
  "answer": "The key principles of embodied cognition include...",
  "citations": [
    {
      "document_id": "doc_123",
      "source": "Chapter 3: Embodied Cognition, Book Title",
      "text_snippet": "Embodied cognition suggests that cognitive processes are not confined to the brain..."
    }
  ],
  "retrieved_documents_count": 3,
  "processing_time": 1250
}
```

## Development

### Running Tests
```bash
pytest tests/
```

### API Documentation
The API documentation is automatically available at:
- `http://localhost:8000/docs` (Swagger UI)
- `http://localhost:8000/redoc` (ReDoc)

## Configuration

The RAG agent can be configured through environment variables:

- `OPENAI_MODEL`: OpenAI model to use (default: gpt-4-turbo)
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: book_content)
- `MAX_TOKENS`: Maximum tokens for response generation (default: 1000)
- `TEMPERATURE`: Temperature for response generation (default: 0.3)