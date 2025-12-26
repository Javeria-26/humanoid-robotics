import React, { createContext, useContext, useReducer, useEffect } from 'react';

// Define initial state
const initialState = {
  messages: [],
  sessionId: null,
  isLoading: false,
  error: null,
  selectedText: null,
};

// Define reducer to handle state changes
const chatbotReducer = (state, action) => {
  switch (action.type) {
    case 'SET_LOADING':
      return {
        ...state,
        isLoading: action.payload,
      };
    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
        isLoading: false,
      };
    case 'ADD_MESSAGE':
      return {
        ...state,
        messages: [...state.messages, action.payload],
        isLoading: false,
      };
    case 'SET_SESSION_ID':
      return {
        ...state,
        sessionId: action.payload,
      };
    case 'SET_SELECTED_TEXT':
      return {
        ...state,
        selectedText: action.payload,
      };
    case 'CLEAR_MESSAGES':
      return {
        ...state,
        messages: [],
      };
    case 'RESET_CHAT':
      return {
        ...initialState,
      };
    case 'LOAD_FROM_STORAGE':
      return {
        ...state,
        messages: action.payload.messages || [],
        sessionId: action.payload.sessionId || null,
      };
    default:
      return state;
  }
};

// Create context
const ChatbotContext = createContext();

// Provider component
export const ChatbotProvider = ({ children }) => {
  const [state, dispatch] = useReducer(chatbotReducer, initialState);

  // Load chat history from localStorage on mount
  useEffect(() => {
    const savedChat = localStorage.getItem('chatbotHistory');
    if (savedChat) {
      try {
        const parsed = JSON.parse(savedChat);
        dispatch({ type: 'LOAD_FROM_STORAGE', payload: parsed });
      } catch (e) {
        console.error('Failed to load chat history from localStorage', e);
      }
    }
  }, []);

  // Save chat history to localStorage whenever messages or sessionId change
  useEffect(() => {
    try {
      const chatData = {
        messages: state.messages,
        sessionId: state.sessionId,
      };
      localStorage.setItem('chatbotHistory', JSON.stringify(chatData));
    } catch (e) {
      console.error('Failed to save chat history to localStorage', e);
    }
  }, [state.messages, state.sessionId]);

  const setLoading = (isLoading) => {
    dispatch({ type: 'SET_LOADING', payload: isLoading });
  };

  const setError = (error) => {
    dispatch({ type: 'SET_ERROR', payload: error });
  };

  const addMessage = (message) => {
    dispatch({ type: 'ADD_MESSAGE', payload: message });
  };

  const setSessionId = (sessionId) => {
    dispatch({ type: 'SET_SESSION_ID', payload: sessionId });
  };

  const setSelectedText = (selectedText) => {
    dispatch({ type: 'SET_SELECTED_TEXT', payload: selectedText });
  };

  const clearMessages = () => {
    dispatch({ type: 'CLEAR_MESSAGES' });
  };

  const resetChat = () => {
    dispatch({ type: 'RESET_CHAT' });
  };

  return (
    <ChatbotContext.Provider
      value={{
        ...state,
        setLoading,
        setError,
        addMessage,
        setSessionId,
        setSelectedText,
        clearMessages,
        resetChat,
      }}
    >
      {children}
    </ChatbotContext.Provider>
  );
};

// Custom hook to use the context
export const useChatbot = () => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error('useChatbot must be used within a ChatbotProvider');
  }
  return context;
};

export default ChatbotContext;