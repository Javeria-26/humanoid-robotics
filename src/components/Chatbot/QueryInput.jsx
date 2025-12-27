import React, { useState, useEffect } from 'react';
import { useChatbot } from '../../contexts/ChatbotContext';
import apiService from '../../services/api';
import TextSelectionHandler from './TextSelectionHandler';
import './Chatbot.module.css';

const QueryInput = () => {
  const [inputValue, setInputValue] = useState('');
  const {
    isLoading,
    error,
    sessionId,
    selectedText,
    setSelectedText,
    setLoading,
    setError,
    addMessage,
    setSessionId
  } = useChatbot();

  // Effect to handle text selection
  useEffect(() => {
    // Add selection listener when component mounts
    const removeListener = TextSelectionHandler.addSelectionListener((selectionInfo) => {
      if (TextSelectionHandler.validateSelection(selectionInfo.text)) {
        setSelectedText(selectionInfo.text);
      } else {
        setSelectedText(null);
      }
    });

    // Cleanup listener when component unmounts
    return () => {
      if (removeListener && typeof removeListener === 'function') {
        removeListener();
      }
    };
  }, [setSelectedText]);

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!inputValue.trim()) return;

    try {
      // Validate the query
      apiService.validateQuery(inputValue);

      // Add user message to the chat
      const userMessage = {
        type: 'user',
        content: inputValue,
        timestamp: new Date().toISOString(),
      };
      addMessage(userMessage);

      // Set loading state
      setLoading(true);

      // Prepare context for the query with query type classification
      const context = {
        query_type: selectedText ? 'selected_text_query' : 'full_book_query',
        source: 'chatbot_ui'
      };
      if (selectedText) {
        context.selected_text = selectedText;
        context.selected_text_length = selectedText.length;
      }

      // Include page information if available
      if (typeof window !== 'undefined') {
        context.page_info = {
          url: window.location.href,
          title: document.title,
          hostname: window.location.hostname
        };
      }

      // Submit the query to the backend with enhanced error handling and caching
      const response = await apiService.submitQueryWithErrorHandling(
        inputValue,
        context,
        sessionId
      );

      // Update session ID if it was returned
      if (response.session_id && response.session_id !== sessionId) {
        setSessionId(response.session_id);
      }

      // Add bot response to the chat
      const botMessage = {
        type: 'bot',
        content: response.answer,
        citations: response.citations || [],
        timestamp: new Date().toISOString(),
      };
      addMessage(botMessage);

      // Clear the input and selected text
      setInputValue('');
      setSelectedText(null);

    } catch (err) {
      console.error('Error submitting query:', err);
      setError(err.message || 'An error occurred while processing your query.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <form className="query-input-form" onSubmit={handleSubmit}>
      <textarea
        className="query-input"
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        placeholder={selectedText
          ? `Ask about selected text: "${selectedText.substring(0, 50)}${selectedText.length > 50 ? '...' : ''}"`
          : "Ask a question about the book content..."}
        disabled={isLoading}
        rows="3"
      />
      <button
        type="submit"
        className="submit-button"
        disabled={isLoading || !inputValue.trim()}
      >
        {isLoading ? 'Sending...' : 'Send'}
      </button>
    </form>
  );
};

export default QueryInput;