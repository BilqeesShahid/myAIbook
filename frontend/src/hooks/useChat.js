import { useState, useEffect } from 'react';
import apiService from '../services/api';

const useChat = () => {
  const [messages, setMessages] = useState([]);
  const [sessionId, setSessionId] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [isUrduMode, setIsUrduMode] = useState(false);

  // Initialize session
  useEffect(() => {
    const initSession = async () => {
      try {
        const sessionResponse = await apiService.createSession();
        setSessionId(sessionResponse.session_id);
      } catch (err) {
        console.error('Error creating session:', err);
        setError('Failed to initialize chat session');
        // Generate a client-side session ID as fallback
        setSessionId(`session_${Date.now()}`);
      }
    };

    initSession();
  }, []);

  const sendMessage = async (text, selectedText = null) => {
    if (!text.trim() && !selectedText) return;

    setIsLoading(true);
    setError(null);

    try {
      // Add user message to local state
      const userMessage = {
        id: Date.now().toString(),
        text: text || (selectedText ? `Explain this: ${selectedText}` : ''),
        sender: 'user',
        timestamp: new Date().toISOString(),
      };

      setMessages(prev => [...prev, userMessage]);

      let response;
      if (selectedText) {
        // Use selected text endpoint
        response = await apiService.askSelectedText(
          text || 'Explain this selected text',
          selectedText,
          sessionId,
          isUrduMode ? 'ur' : 'en'
        );
      } else {
        // Use regular ask endpoint
        response = await apiService.ask(
          text,
          sessionId,
          isUrduMode ? 'ur' : 'en'
        );
      }

      // Add assistant response to local state
      const assistantMessage = {
        id: `resp_${Date.now()}`,
        text: response.response,
        sender: 'assistant',
        timestamp: response.timestamp,
        sourceChunks: response.source_chunks,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Error sending message:', err);
      setError('Failed to send message');

      // Add error message to chat
      const errorMessage = {
        id: `error_${Date.now()}`,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'system',
        timestamp: new Date().toISOString(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  const toggleLanguage = () => {
    setIsUrduMode(!isUrduMode);
  };

  const loadSessionHistory = async (sessionToLoad = null) => {
    const sessionToUse = sessionToLoad || sessionId;
    if (!sessionToUse) return;

    try {
      const history = await apiService.getSessionHistory(sessionToUse);
      setMessages(history.messages || []);
    } catch (err) {
      console.error('Error loading session history:', err);
      setError('Failed to load session history');
    }
  };

  return {
    messages,
    sessionId,
    isLoading,
    error,
    isUrduMode,
    sendMessage,
    clearChat,
    toggleLanguage,
    loadSessionHistory,
  };
};

export default useChat;