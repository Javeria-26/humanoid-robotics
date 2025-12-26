# Agent Context: Frontend-Backend Integration

## Technology Stack

- **Frontend**: Docusaurus framework with React components
- **Backend**: FastAPI-based RAG system
- **Communication**: REST API with JSON payloads
- **UI Components**: Custom React chat interface
- **Security**: Environment variable-based configuration

## Implementation Notes

### Docusaurus Integration
- Chatbot component should be implemented as a React component
- Component can be added globally via Docusaurus layout or per-page via MDX
- Must maintain Docusaurus theme consistency
- Should be lightweight and not impact page load times

### API Communication
- Use standard fetch API for communication with backend
- Implement proper error handling and loading states
- Include request/response validation
- Handle timeouts gracefully (target: <10 seconds as per spec)

### Selected Text Feature
- Use browser's Selection API to capture user-selected text
- Include selected text as context in API requests
- Fallback to page context if no text is selected
- Consider text length limits for optimal performance

### Security Considerations
- Store API endpoints and keys in environment variables
- Implement proper CORS handling
- Validate all inputs before sending to backend
- Handle sensitive data appropriately

## Data Models

### User Query
- content: string (user's question)
- context: object (selected text, page info)
- session_id: string (conversation tracking)

### Backend Response
- answer: string (AI-generated response)
- citations: array (source references)
- confidence: number (0-1 relevance score)

## API Endpoints

### POST /api/query
- Request: { query: string, context?: object, session_id?: string }
- Response: { answer: string, citations: array, session_id: string, confidence?: number }

### GET /api/health
- Response: { status: "ok", timestamp: datetime }

## Error Handling

- Network errors: Show user-friendly message
- Backend errors: Display appropriate feedback
- Timeout errors: Implement retry logic
- Validation errors: Provide clear input guidance

## Testing Points

- Chatbot appears on all book pages
- Queries return relevant responses (90% accuracy target)
- Selected text queries work properly
- Error states are handled gracefully
- API communication completes within 10 seconds