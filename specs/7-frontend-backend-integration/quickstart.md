# Quickstart Guide: Frontend-Backend Integration

**Feature**: Frontend-Backend Integration
**Date**: 2025-12-25

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Access to the backend API endpoint
- Environment variables configured for API access

## Setup Instructions

### 1. Clone and Install Dependencies

```bash
git clone <repository-url>
cd <project-directory>
npm install
```

### 2. Configure Environment Variables

Create a `.env` file in the root directory with the following variables:

```env
REACT_APP_BACKEND_API_URL=https://your-backend-api.com
REACT_APP_API_KEY=your-api-key-if-required
```

### 3. Add the Chatbot Component

The chatbot component should be integrated into your Docusaurus layout. Add the component to your `src/components/` directory and import it into your layout.

### 4. Build and Run

For development:
```bash
npm start
```

For production build:
```bash
npm run build
npm run serve
```

## Integration Points

### Docusaurus Integration

1. Create the chatbot component in `src/components/Chatbot/Chatbot.js`
2. Add the component to your Docusaurus layout in `src/theme/Layout/index.js`
3. The component will be available on all pages

### API Communication

1. The component communicates with the backend via the `/query` endpoint
2. Requests include the user query and optional context
3. Responses are displayed in the chat interface with citations

## Testing

### Manual Testing

1. Navigate to any page in the book
2. Activate the chatbot UI
3. Submit a question about the book content
4. Verify the response is relevant and includes citations
5. Test with selected text queries
6. Verify error handling works properly

### API Testing

Use the provided OpenAPI specification to test the backend endpoints.

## Troubleshooting

- If the chatbot doesn't appear, check that the component is properly imported in your layout
- If API calls fail, verify your environment variables are correctly set
- If selected text queries don't work, ensure the selection API is properly implemented