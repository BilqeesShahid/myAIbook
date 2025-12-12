import React, { useState, useEffect, useRef } from 'react';
import Message from './Message';
import InputArea from './InputArea';
import SelectionHandler from './SelectionHandler';
import apiService from '../services/api';

const ChatInterface = () => {
  const [messages, setMessages] = useState([]);
  const [sessionId, setSessionId] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [isUrduMode, setIsUrduMode] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize session on component mount
  useEffect(() => {
    const initSession = async () => {
      try {
        const sessionResponse = await apiService.createSession();
        setSessionId(sessionResponse.session_id);
      } catch (error) {
        console.error('Error creating session:', error);
        // Generate a client-side session ID as fallback
        setSessionId(`session_${Date.now()}`);
      }
    };

    initSession();
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSendMessage = async (message, contextSelectedText = null) => {
    console.log('handleSendMessage called with message:', message, 'and contextSelectedText:', contextSelectedText);
    if (!message.trim() && !contextSelectedText) return;

    let textToSend = message;
    if (contextSelectedText && !message.trim()) {
      // When only selected text is provided (like from the button click), summarize it
      textToSend = `Summarize this text: ${contextSelectedText}`;
    } else if (contextSelectedText && message.trim()) {
      // When both message and selected text are provided, combine them
      textToSend = `${message} based on this selected text: ${contextSelectedText}`;
    }

    console.log('textToSend will be:', textToSend);

    // Add user message to UI immediately
    const userMessage = {
      id: Date.now().toString(),
      text: textToSend,  // Show the full message to the user
      sender: 'user',
      timestamp: new Date().toISOString(),
    };

    console.log('Adding user message:', userMessage);
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      let response;
      if (contextSelectedText) {
        console.log('Making selected text API call with query:', message.trim() || "Summarize this text", 'and selected text:', contextSelectedText);
        // Use the selected text endpoint
        // Extract the actual query instruction without the selected text content
        const actualQuery = message.trim() || "Summarize this text";

        response = await apiService.askSelectedText(
          actualQuery,
          contextSelectedText,  // This is the selected text content to summarize
          sessionId,
          isUrduMode ? 'ur' : 'en'
        );
        console.log('Selected text API response:', response);
      } else {
        console.log('Making regular ask API call with:', textToSend);
        // Use the regular ask endpoint
        response = await apiService.ask(
          textToSend,
          sessionId,
          isUrduMode ? 'ur' : 'en'
        );
        console.log('Regular ask API response:', response);
      }

      // Add assistant response to UI
      const assistantMessage = {
        id: `resp_${Date.now()}`,
        text: response.response,
        sender: 'assistant',
        timestamp: response.timestamp,
        sourceChunks: response.source_chunks,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error getting response:', error);

      // Add error message to UI
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

  const handleSelectedText = async (text) => {
    console.log('handleSelectedText called with:', text);
    if (text && text.trim()) {
      // Directly send the selected text for summarization
      try {
        await handleSendMessage('', text);
      } catch (error) {
        console.error('Error in handleSelectedText:', error);
      }
    } else {
      console.log('No valid selected text to send');
    }
  };

  const clearSelectedText = () => {
    setSelectedText('');
  };

  const toggleLanguage = () => {
    setIsUrduMode(!isUrduMode);
  };

  return (
    <div className="chat-interface">
      <div className="chat-header">
        <h2>{isUrduMode ? "چیٹ بورڈ" : "Chat Interface"}</h2>
        <button
          className={`language-toggle ${isUrduMode ? 'urdu' : 'english'}`}
          onClick={toggleLanguage}
        >
          {isUrduMode ? "EN" : "اردو"}
        </button>
      </div>

      <SelectionHandler onSelectedText={handleSelectedText} />

      <div className="messages-container">
        {messages.length === 0 ? (
          <div className="welcome-message">
            <p>{isUrduMode ? "خوش آمدید! مجھ سے کتاب کے بارے میں کوئی بھی سوال پوچھیں۔" : "Welcome! Ask me anything about the book."}</p>
          </div>
        ) : (
          messages.map((message) => (
            <Message
              key={message.id}
              message={message}
              isUrduMode={isUrduMode}
            />
          ))
        )}
        {isLoading && (
          <Message
            message={{
              id: 'loading',
              text: isUrduMode ? "لوڈ ہو رہا ہے..." : "Loading...",
              sender: 'assistant'
            }}
            isUrduMode={isUrduMode}
            isLoading={true}
          />
        )}
        <div ref={messagesEndRef} />
      </div>

      <InputArea
        onSendMessage={handleSendMessage}
        selectedText={selectedText}
        onClearSelectedText={clearSelectedText}
        isUrduMode={isUrduMode}
      />
    </div>
  );
};

export default ChatInterface;