import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Define the shape of a chat message
interface ChatMessage {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  references?: { url: string; title?: string }[]; // Optional array of references
}

// Define the shape of the chatbot context state
interface ChatbotContextType {
  messages: ChatMessage[];
  addMessage: (message: ChatMessage) => void;
  sendMessage: (text: string, contextSelection?: string) => Promise<void>;
  isLoading: boolean;
  error: string | null;
  isOpen: boolean;
  openChatbot: () => void;
  closeChatbot: () => void;
}

// Create the context
const ChatbotContext = createContext<ChatbotContextType | undefined>(undefined);

// Define the props for the provider
interface ChatbotProviderProps {
  children: ReactNode;
}

// Chatbot provider component
export const ChatbotProvider: React.FC<ChatbotProviderProps> = ({ children }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isOpen, setIsOpen] = useState(false);

  const { siteConfig } = useDocusaurusContext();

  // Function to add a message to the chat history
  const addMessage = (message: ChatMessage) => {
    setMessages((prevMessages) => [...prevMessages, message]);
  };

  // Function to send a message to the backend and handle response
  const sendMessage = async (text: string, contextSelection: string | undefined = undefined) => {
    if (text.trim() === '') return;

    // Add user message to history immediately
    addMessage({ id: Date.now().toString(), text, sender: 'user' });
    setIsLoading(true);
    setError(null);

    try {
      const { customFields } = siteConfig;
      const fastapiBaseUrl = customFields?.fastApiBaseUrl || 'http://localhost:8000';
      console.log('Sending request to:', `${fastapiBaseUrl}/chat`); // Debug log

      const response = await fetch(`${fastapiBaseUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: text,
          context_selection: contextSelection,
          session_id: "test-session-id" // Placeholder, implement actual session management
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to get response from chatbot.');
      }

      const data = await response.json();
      addMessage({ id: Date.now().toString(), text: data.answer, sender: 'bot' });
      // TODO: Handle references from data.references
    } catch (err) {
      console.error('Chatbot API error:', err);
      setError('Failed to connect to the chatbot. Please try again later.');
      addMessage({
        id: Date.now().toString(),
        text: 'Sorry, I am unable to respond at the moment.',
        sender: 'bot'
      });
    } finally {
      setIsLoading(false);
    }
  };

  // Functions to control chatbot visibility
  const openChatbot = () => {
    setIsOpen(true);
  };

  const closeChatbot = () => {
    setIsOpen(false);
  };

  const contextValue: ChatbotContextType = {
    messages,
    addMessage,
    sendMessage,
    isLoading,
    error,
    isOpen,
    openChatbot,
    closeChatbot,
  };

  return (
    <ChatbotContext.Provider value={contextValue}>
      {children}
    </ChatbotContext.Provider>
  );
};

// Custom hook to use the chatbot context
export const useChatbot = () => {
  const context = useContext(ChatbotContext);
  if (!context) {
    throw new Error('useChatbot must be used within a ChatbotProvider');
  }
  return context;
};
