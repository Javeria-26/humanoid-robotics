# Quickstart Guide: Retrieval Pipeline Validation

**Feature**: Retrieval Pipeline Validation
**Date**: 2025-12-25

## Prerequisites

1. Python 3.9+ installed
2. Backend dependencies installed (from backend/requirements.txt)
3. Environment variables configured in backend/.env:
   - QDRANT_URL: Your Qdrant Cloud instance URL
   - QDRANT_API_KEY: Your Qdrant Cloud API key
   - COHERE_API_KEY: Your Cohere API key

## Setup

1. **Navigate to backend directory:**
   ```bash
   cd backend
   ```

2. **Install dependencies (if not already installed):**
   ```bash
   pip install -r requirements.txt
   ```

3. **Activate virtual environment (if using one):**
   ```bash
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

4. **Verify environment variables:**
   ```bash
   python -c "import os; print('Qdrant URL:', os.getenv('QDRANT_URL')); print('Cohere Key Present:', bool(os.getenv('COHERE_API_KEY')))"
   ```

## Running Retrieval Validation

### Option 1: Using the validation module (to be implemented)

```bash
python -m validation.run_validation
```

### Option 2: Direct function calls in Python

```python
from validation.retrieval_validator import RetrievalValidator

# Initialize the validator
validator = RetrievalValidator()

# Run comprehensive validation
results = validator.run_comprehensive_validation()

# Print results
print(results.summary())
```

### Option 3: Run sample queries to test retrieval

The existing `sample_query_validation` function can be used for basic testing:

```python
from main import sample_query_validation

# Run a test query
sample_query_validation("your test query here", top_k=5)
```

## Configuration

The validation system will use the same configuration as the ingestion system:

- **Collection Name**: Defaults to "rag_embedding" (from COLLECTION_NAME environment variable)
- **Embedding Model**: Cohere "multilingual-22-12" model
- **Qdrant Connection**: Uses the same connection parameters as ingestion

## Sample Test Queries

Start with these test queries to validate retrieval:

1. **General topic query**: "What is humanoid robotics?"
2. **Specific concept query**: "Explain inverse kinematics"
3. **Technical term query**: "What is ROS 2?"
4. **Book content query**: "How does the perception system work?"

## Expected Output

The validation will produce:

1. **Relevance Metrics**: Precision scores for top-k results
2. **Metadata Integrity Report**: Accuracy of URL, title, section fields
3. **Performance Metrics**: Retrieval times and throughput
4. **Validation Summary**: Overall pass/fail status against success criteria