# Data Model: Website Ingestion & Vector Indexing

## Content Chunk Entity

**Description**: A segment of cleaned text extracted from a Docusaurus page with associated metadata

**Fields**:
- `id` (string): Unique identifier for the chunk (UUID or hash)
- `text` (string): The cleaned, chunked text content
- `url` (string): Original URL where the content was found
- `title` (string): Page title from the source
- `section` (string): Section or heading under which content appears
- `ingestion_timestamp` (datetime): When the chunk was processed
- `content_hash` (string): Hash of content for duplicate detection

**Validation Rules**:
- `text` must be non-empty and less than 1000 words
- `url` must be a valid URL format
- `content_hash` must be unique within the system

## Vector Embedding Entity

**Description**: Numerical representation of content chunk for semantic search

**Fields**:
- `chunk_id` (string): Reference to the associated content chunk
- `vector` (array of floats): The embedding vector values
- `vector_size` (integer): Dimension of the embedding vector
- `embedding_model` (string): Model used to generate the embedding

**Validation Rules**:
- `vector` must match expected dimension size for the model
- `chunk_id` must reference an existing content chunk

## Qdrant Payload Structure

**Description**: How data is stored in Qdrant collection

**Fields**:
- `id` (string): Point ID in Qdrant (same as content chunk ID)
- `vector` (array of floats): The embedding vector
- `payload` (object): Contains metadata:
  - `url` (string): Original source URL
  - `title` (string): Page title
  - `section` (string): Content section
  - `text` (string): Original text content (for retrieval)
  - `ingestion_timestamp` (string): When ingested (ISO format)

## Ingestion State Entity

**Description**: Tracks the state of the ingestion process for re-runnable execution

**Fields**:
- `url` (string): The URL being processed
- `content_hash` (string): Hash of the content at that URL
- `last_ingested` (datetime): When this URL was last processed
- `status` (string): Current status (pending, processing, completed, failed)

**Validation Rules**:
- `content_hash` enables detection of unchanged content
- `status` helps manage retries and error handling