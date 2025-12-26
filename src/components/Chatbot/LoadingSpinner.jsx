import React from 'react';
import './Chatbot.module.css';

const LoadingSpinner = () => {
  return (
    <div className="loading-container">
      <div className="loading-spinner">
        <div></div>
        <div></div>
        <div></div>
      </div>
      <span className="loading-text">Thinking...</span>
    </div>
  );
};

export default LoadingSpinner;