# Chatbot Component

The Chatbot component provides an AI-powered assistant for users to ask questions about the book content.

## Features

- In-page question answering
- Selected text queries
- Session management
- Error handling
- Loading states
- Responsive design

## Architecture

- `Chatbot.jsx`: Main component with UI and state management
- `ChatbotContext.js`: Global state management for chat sessions
- `api.js`: Service for backend communication
- `Message.jsx`: Component for displaying messages
- `QueryInput.jsx`: Component for user input with text selection
- `TextSelectionHandler.js`: Utility for handling text selection
- `LoadingSpinner.jsx`: Loading state indicator
- `ErrorMessage.jsx`: Error message display
- `Chatbot.module.css`: Component-specific styles

## Usage

The chatbot is integrated globally via the Layout wrapper and appears as a floating button on all pages.

## API Integration

- Queries are sent to the backend API at `/api/query`
- Responses include citations and context
- Session management maintains conversation flow
- Caching improves performance for repeated queries

## Security

- Environment variables manage API endpoints
- Request validation prevents malformed queries
- Error handling prevents information disclosure
- Timeout handling prevents hanging requests