import React, { useState, useRef } from 'react';

const InputArea = ({ onSendMessage, selectedText, onClearSelectedText, isUrduMode }) => {
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef(null);

  const handleSubmit = (e) => {
    e.preventDefault();

    if (inputValue.trim() || selectedText) {
      // If there's selected text, combine it with the input or use it as context
      const message = inputValue.trim() || (selectedText ? `Explain this: ${selectedText}` : '');
      onSendMessage(message, selectedText);
      setInputValue('');
      if (onClearSelectedText) {
        onClearSelectedText();
      }
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <div className="input-area">
      {selectedText && (
        <div className="selected-text-preview">
          <small>Selected text:</small>
          <div className="selected-text-content">
            "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
            <button
              className="clear-selected-btn"
              onClick={onClearSelectedText}
              aria-label="Clear selected text"
            >
              ×
            </button>
          </div>
        </div>
      )}

      <form onSubmit={handleSubmit} className="input-form">
        <textarea
          ref={textareaRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={isUrduMode ? "پیغام لکھیں..." : "Type your message..."}
          className="input-textarea"
          rows="3"
        />
        <button
          type="submit"
          className="send-button"
          disabled={!inputValue.trim() && !selectedText}
        >
          {isUrduMode ? "بھیجیں" : "Send"}
        </button>
      </form>
    </div>
  );
};

export default InputArea;