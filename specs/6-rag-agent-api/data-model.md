# Data Model: RAG Agent & API Backend

## Entities

### Query
- **Description**: A natural language question from a user seeking information from book content
- **Fields**:
  - `query_text` (string): The user's question or query text
  - `query_id` (string, optional): Unique identifier for the query
  - `user_context` (object, optional): Additional context about the user's session or preferences
  - `timestamp` (datetime): When the query was submitted

### RetrievedDocuments
- **Description**: Book passages or sections identified as relevant to the user's query
- **Fields**:
  - `documents` (array): List of document objects containing retrieved content
  - `document_id` (string): Unique identifier for the document
  - `content` (string): The actual text content of the document
  - `source` (string): Reference to the original book or document location
  - `score` (number): Relevance score from the retrieval system
  - `metadata` (object): Additional metadata about the document (book title, chapter, page, etc.)

### GeneratedResponse
- **Description**: The AI-generated answer that is grounded in the retrieved documents
- **Fields**:
  - `response_text` (string): The generated answer text
  - `query_id` (string): Reference to the original query
  - `citations` (array): List of source documents used in generation
  - `confidence_score` (number, optional): Confidence level in the response
  - `timestamp` (datetime): When the response was generated
  - `status` (string): Status of the generation (success, partial, error)

### Citation
- **Description**: Reference information that indicates which specific documents were used to generate the response
- **Fields**:
  - `document_id` (string): Reference to the source document
  - `source` (string): Human-readable source reference (book title, chapter, etc.)
  - `text_snippet` (string): Relevant text excerpt from the source
  - `relevance_score` (number): How relevant this citation was to the response

### QueryRequest
- **Description**: API request object for the query endpoint
- **Fields**:
  - `query` (string): The user's question (required)
  - `include_citations` (boolean, optional): Whether to include citation information (default: true)
  - `max_results` (number, optional): Maximum number of documents to retrieve (default: 5)

### QueryResponse
- **Description**: API response object from the query endpoint
- **Fields**:
  - `query` (string): The original user query
  - `answer` (string): The AI-generated answer
  - `citations` (array): List of citations used to generate the answer
  - `retrieved_documents_count` (number): Number of documents retrieved
  - `processing_time` (number): Time taken to process the query in milliseconds

## Relationships
- A `Query` generates one `GeneratedResponse`
- A `GeneratedResponse` references multiple `RetrievedDocuments` through `Citation` objects
- Multiple `Query` objects may reference the same `RetrievedDocuments` if they retrieve similar content

## Validation Rules
- `Query.query_text` must be between 1 and 1000 characters
- `QueryResponse.answer` must not be empty when status is success
- `Citation` objects must reference valid `RetrievedDocuments`
- `QueryRequest.query` is required and must not be empty