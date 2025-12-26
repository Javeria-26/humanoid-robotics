# Data Model: Frontend-Backend Integration

**Feature**: Frontend-Backend Integration
**Date**: 2025-12-25

## User Query

**Description**: Represents a question submitted by the user through the chatbot interface

**Fields**:
- `content`: string - The text of the user's question
- `context`: object (optional) - Additional context such as selected text or current page
  - `selected_text`: string (optional) - Text selected by the user on the page
  - `page_url`: string (optional) - URL of the current page for context
  - `page_title`: string (optional) - Title of the current page
- `timestamp`: datetime - When the query was submitted
- `session_id`: string (optional) - ID of the current chat session
- `user_id`: string (optional) - Identifier for the user (if available)

## Backend Response

**Description**: The response from the backend system to a user query

**Fields**:
- `answer`: string - The AI-generated answer to the user's question
- `citations`: array - List of sources referenced in the answer
  - `title`: string - Title of the source
  - `url`: string - URL to the source (if applicable)
  - `page_number`: number (optional) - Page number in the book
- `confidence`: number (0-1) - Confidence score for the response relevance
- `query_id`: string - ID linking back to the original query
- `session_id`: string - ID of the current chat session
- `timestamp`: datetime - When the response was generated

## Chat Session

**Description**: Maintains conversation context between the user and the system

**Fields**:
- `session_id`: string - Unique identifier for the session
- `messages`: array - List of messages in the conversation
  - `type`: string (input|output) - Type of message
  - `content`: string - The message content
  - `timestamp`: datetime - When the message was created
- `created_at`: datetime - When the session was created
- `updated_at`: datetime - When the session was last updated
- `user_context`: object (optional) - Additional user context

## API Request Payload

**Description**: Structure of data sent from frontend to backend

**Fields**:
- `query`: string - The user's question
- `context`: object (optional) - Additional context information
  - `selected_text`: string (optional) - Text selected by the user
  - `page_info`: object (optional) - Information about the current page
- `session_id`: string (optional) - ID of the current session
- `metadata`: object (optional) - Additional metadata for the request

## API Response Payload

**Description**: Structure of data returned from backend to frontend

**Fields**:
- `answer`: string - The AI-generated answer
- `citations`: array - Sources referenced in the answer
- `session_id`: string - Session identifier (may be new or existing)
- `query_id`: string - Unique identifier for this query
- `error`: object (optional) - Error information if the request failed
  - `type`: string - Type of error
  - `message`: string - Human-readable error message