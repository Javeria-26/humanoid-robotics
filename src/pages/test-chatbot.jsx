import React from 'react';
import { ChatbotProvider } from '../contexts/ChatbotContext';
import Chatbot from '../components/Chatbot/Chatbot';

const TestChatbotPage = () => {
  return (
    <ChatbotProvider>
      <div style={{ padding: '20px', minHeight: '100vh' }}>
        <h1>Chatbot Test Page</h1>
        <p>This page is testing the chatbot component in isolation.</p>
        <div style={{ marginTop: '20px' }}>
          <Chatbot />
        </div>
      </div>
    </ChatbotProvider>
  );
};

export default TestChatbotPage;