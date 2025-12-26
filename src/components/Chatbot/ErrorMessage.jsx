import React from 'react';
import './Chatbot.module.css';

const ErrorMessage = ({ message }) => {
  return (
    <div className="error-message">
      <div className="error-icon">⚠️</div>
      <div className="error-content">
        <p>{message}</p>
        <small>Please try again or check your connection.</small>
      </div>
    </div>
  );
};

export default ErrorMessage;