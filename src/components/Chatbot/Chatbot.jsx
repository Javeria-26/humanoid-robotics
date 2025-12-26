import React, { useState, useEffect, useCallback } from 'react';
import { useChatbot } from '../../contexts/ChatbotContext';
import Message from './Message';
import QueryInput from './QueryInput';
import LoadingSpinner from './LoadingSpinner';
import ErrorMessage from './ErrorMessage';
import SelectedTextIndicator from './SelectedTextIndicator';
import './Chatbot.module.css';

const Chatbot = () => {
  const {
    messages,
    isLoading,
    error,
    selectedText,
    setSelectedText,
    clearMessages,
    resetChat,
  } = useChatbot();
  const [isOpen, setIsOpen] = useState(false);

  // Handle keyboard shortcuts
  const handleKeyDown = useCallback((event) => {
    // Toggle chat with Ctrl/Cmd + Shift + C
    if ((event.ctrlKey || event.metaKey) && event.shiftKey && event.key === 'C') {
      event.preventDefault();
      setIsOpen(prev => !prev);
    }

    // Close chat with Escape when open
    if (event.key === 'Escape' && isOpen) {
      setIsOpen(false);
    }
  }, [isOpen]);

  // Add keyboard event listeners
  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleKeyDown]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleClearChat = () => {
    clearMessages();
    setSelectedText(null);
  };

  const handleResetChat = () => {
    resetChat();
    setSelectedText(null);
  };

  return (
    <div
      className="chatbot-container"
      role="complementary"
      aria-label="Book assistant chatbot"
    >
      {/* Chatbot toggle button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
        title="Open chat assistant (Ctrl+Shift+C)"
        aria-expanded={isOpen}
      >
        {isOpen ? '✕' : '🤖'}
      </button>

      {/* Chatbot panel */}
      {isOpen && (
        <div
          className="chatbot-panel"
          role="dialog"
          aria-modal="true"
          aria-label="Chat with book assistant"
        >
          <div className="chatbot-header" role="banner">
            <h3 id="chatbot-title">Book Assistant</h3>
            <div className="chatbot-actions" role="toolbar">
              <button
                className="clear-btn"
                onClick={handleClearChat}
                title="Clear chat history"
                aria-label="Clear chat history"
                aria-describedby="chatbot-title"
              >
                🗑️
              </button>
              <button
                className="reset-btn"
                onClick={handleResetChat}
                title="Reset conversation"
                aria-label="Reset conversation"
                aria-describedby="chatbot-title"
              >
                🔄
              </button>
              <button
                className="close-btn"
                onClick={toggleChat}
                title="Close chat (Esc)"
                aria-label="Close chat (Esc)"
                aria-describedby="chatbot-title"
              >
                ✕
              </button>
            </div>
          </div>

          <div
            className="chatbot-messages"
            role="log"
            aria-live="polite"
            aria-relevant="additions"
          >
            {error && <ErrorMessage message={error} />}

            {selectedText && (
              <div
                className="selected-text-preview"
                role="status"
                aria-live="polite"
              >
                <strong>Selected text:</strong> "{selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText}"
              </div>
            )}

            {messages.length === 0 ? (
              <div
                className="welcome-message"
                role="status"
                aria-live="polite"
              >
                <p>Hello! I'm your book assistant. Ask me anything about the content you're reading.</p>
                <p>You can also select text on the page and ask questions about it.</p>
              </div>
            ) : (
              messages.map((message, index) => (
                <Message
                  key={index}
                  message={message}
                  aria-label={`Message ${index + 1} of ${messages.length}`}
                />
              ))
            )}

            {isLoading && <LoadingSpinner />}
          </div>

          <div className="chatbot-input-area" role="form">
            <QueryInput />
          </div>
        </div>
      )}
    </div>
  );
};

export default Chatbot;