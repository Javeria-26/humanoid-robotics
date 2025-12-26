import React from 'react';
import Layout from '@theme-original/Layout';
import { ChatbotProvider } from '../contexts/ChatbotContext';
import Chatbot from '../components/Chatbot/Chatbot';

export default function LayoutWrapper(props) {
  return (
    <ChatbotProvider>
      <Layout {...props}>
        {props.children}
        <Chatbot />
      </Layout>
    </ChatbotProvider>
  );
}