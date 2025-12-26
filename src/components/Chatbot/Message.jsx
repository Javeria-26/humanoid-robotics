import React from 'react';
import './Chatbot.module.css';

const Message = ({ message }) => {
  const { type, content, citations, timestamp } = message;

  const formatCitations = (citations) => {
    if (!citations || citations.length === 0) return null;

    return (
      <div className="message-citations">
        <h4>Sources:</h4>
        <ul>
          {citations.map((citation, index) => (
            <li key={index}>
              {citation.url ? (
                <a href={citation.url} target="_blank" rel="noopener noreferrer">
                  {citation.title || `Source ${index + 1}`}
                </a>
              ) : (
                <span>{citation.title || `Source ${index + 1}`}</span>
              )}
              {citation.page_number && <span> (Page {citation.page_number})</span>}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className={`message ${type}`}>
      <div className="message-content">
        {content}
      </div>
      {formatCitations(citations)}
      {timestamp && (
        <div className="message-timestamp">
          {new Date(timestamp).toLocaleTimeString()}
        </div>
      )}
    </div>
  );
};

export default Message;