import React, { useState, useEffect } from 'react';

const SelectionHandler = ({ onSelectedText }) => {
  const [selectedText, setSelectedText] = useState('');
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text) {
        // Get the position of the selection
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(text);
        setButtonPosition({ x: rect.left, y: rect.top - 10 });
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection);
    };
  }, []);

  const handleAskChatbot = () => {
    // Get the current selection at the time of click, not relying on state
    const currentSelection = window.getSelection();
    const currentText = currentSelection.toString().trim();

    console.log('handleAskChatbot called with current selection:', currentText);
    if (currentText) {
      // Directly send the selected text to be summarized
      onSelectedText(currentText);
      setShowButton(false);
      window.getSelection().removeAllRanges(); // Clear selection
    } else {
      console.log('No selected text to send');
    }
  };

  return (
    <>
      {showButton && (
        <div
          style={{
            position: 'fixed',
            left: `${buttonPosition.x}px`,
            top: `${buttonPosition.y}px`,
            zIndex: 1000,
            backgroundColor: '#4f46e5',
            color: 'white',
            padding: '5px 10px',
            borderRadius: '4px',
            cursor: 'pointer',
            fontSize: '14px',
            fontWeight: 'bold',
            boxShadow: '0 2px 6px rgba(0,0,0,0.2)',
          }}
          onClick={handleAskChatbot}
        >
          Ask Chatbot
        </div>
      )}
    </>
  );
};

export default SelectionHandler;