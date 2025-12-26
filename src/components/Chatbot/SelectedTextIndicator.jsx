import React from 'react';
import { useChatbot } from '../../contexts/ChatbotContext';
import './Chatbot.module.css';

const SelectedTextIndicator = () => {
  const { selectedText, setSelectedText } = useChatbot();

  const handleUseSelection = () => {
    if (selectedText) {
      // This would typically trigger the input field to be pre-filled
      // For now, we'll just clear the selection
      setSelectedText(null);
    }
  };

  const handleClearSelection = () => {
    setSelectedText(null);
  };

  if (!selectedText) {
    return null;
  }

  return (
    <div className="selected-text-indicator">
      <div className="selected-text-content">
        <strong>Selected Text:</strong> "{selectedText.length > 100 ? selectedText.substring(0, 100) + '...' : selectedText}"
      </div>
      <div className="selected-text-actions">
        <button
          className="use-selection-btn"
          onClick={handleUseSelection}
          title="Use this selection in your query"
        >
          ✅ Use
        </button>
        <button
          className="clear-selection-btn"
          onClick={handleClearSelection}
          title="Clear selection"
        >
          ❌ Clear
        </button>
      </div>
    </div>
  );
};

export default SelectedTextIndicator;