import React from 'react';

const Message = ({ message, isUrduMode, isLoading = false }) => {
  const isUser = message.sender === 'user';
  const isSystem = message.sender === 'system';

  // Function to format source chunks if they exist
  const renderSourceChunks = () => {
    if (!message.sourceChunks || !Array.isArray(message.sourceChunks) || message.sourceChunks.length === 0) {
      return null;
    }

    return (
      <div className="source-chunks">
        <small>{isUrduMode ? "ماخذ:" : "Sources:"}</small>
        <ul>
          {message.sourceChunks.map((chunk, index) => (
            <li key={index}>
              {isUrduMode
                ? `چیپٹر ${chunk.chapter_number}: ${chunk.section_title}`
                : `Chapter ${chunk.chapter_number}: ${chunk.section_title}`}
            </li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className={`message ${isUser ? 'user-message' : 'assistant-message'}`}>
      <div className={`message-content ${isUser ? 'user' : ''}`}>
        <div className="message-text" dir={isUrduMode ? 'rtl' : 'ltr'}>
          {isLoading ? (
            <div className="loading-indicator">
              <div></div>
              <div></div>
              <div></div>
            </div>
          ) : (
            message.text
          )}
        </div>
        {message.sender === 'assistant' && renderSourceChunks()}
      </div>
      {message.timestamp && (
        <div className="message-timestamp">
          {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </div>
      )}
    </div>
  );
};

export default Message;