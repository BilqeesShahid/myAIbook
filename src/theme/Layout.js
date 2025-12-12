import React, { useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';

export default function Layout(props) {
  useEffect(() => {
    // Check if we're in the browser environment
    if (typeof window !== 'undefined') {
      const chatbotContainer = document.createElement('div');
      chatbotContainer.id = 'docusaurus-chatbot';
      chatbotContainer.innerHTML = `
        <style>
          #docusaurus-chatbot {
            position: fixed;
            bottom: 20px;
            right: 20px;
            z-index: 10000;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
          }

          .chatbot-button {
            width: 60px;
            height: 60px;
            border-radius: 50%;
            background-color: #4CAF50;
            color: white;
            border: none;
            font-size: 24px;
            cursor: pointer;
            box-shadow: 0 4px 12px rgba(76, 175, 80, 0.4);
            display: flex;
            align-items: center;
            justify-content: center;
            transition: all 0.3s ease;
          }

          .chatbot-button:hover {
            transform: scale(1.1);
            box-shadow: 0 6px 16px rgba(76, 175, 80, 0.6);
          }

          .chatbot-modal {
            position: fixed;
            bottom: 90px;
            right: 20px;
            width: 350px;
            height: 500px;
            background-color: white;
            border-radius: 12px;
            display: none;
            flex-direction: column;
            box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
            overflow: hidden;
            z-index: 10001;
          }

          .chatbot-modal.active {
            display: flex;
          }

          .chatbot-header {
            background-color: #4CAF50;
            color: white;
            padding: 12px 16px;
            display: flex;
            justify-content: space-between;
            align-items: center;
          }

          .chatbot-close {
            background: none;
            border: none;
            color: white;
            font-size: 20px;
            cursor: pointer;
          }

          .chatbot-messages {
            flex: 1;
            padding: 12px;
            overflow-y: auto;
            display: flex;
            flex-direction: column;
            gap: 10px;
            background-color: #f9f9f9;
          }

          .chatbot-message {
            max-width: 80%;
            padding: 10px 15px;
            border-radius: 18px;
            line-height: 1.4;
            word-break: break-word;
          }

          .chatbot-message.user {
            align-self: flex-end;
            background-color: #4CAF50;
            color: white;
            border-bottom-right-radius: 4px;
          }

          .chatbot-message.bot {
            align-self: flex-start;
            background-color: #e9ecef;
            color: #333;
            display: flex;
            align-items: center;
            gap: 8px;
            border-bottom-left-radius: 4px;
            font-size: 13px; /* smaller text */
          }

          .chatbot-message.bot .bot-avatar {
            font-size: 14px; /* only emoji, no bg color */
          }

          .chatbot-input-area {
            padding: 12px;
            border-top: 1px solid #eee;
            display: flex;
            gap: 6px;
            background-color: white;
          }

          .chatbot-input {
            flex: 1;
            padding: 10px 12px;
            border-radius: 24px;
            border: 1px solid #ddd;
            font-size: 14px;
            resize: none;
          }

          .chatbot-send-btn {
            padding: 8px 14px;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 20px;
            cursor: pointer;
          }

          /* Selected text ask button */
          .selected-text-ask-btn {
            position: fixed;
            z-index: 10002;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 20px;
            padding: 8px 12px;
            font-size: 12px;
            cursor: pointer;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
            display: none;
            transform: translate(-50%, -100%);
            white-space: nowrap;
          }

          .selected-text-ask-btn:hover {
            background-color: #45a049;
          }
        </style>

        <button class="chatbot-button" id="chatbotToggle">💬</button>

        <div class="selected-text-ask-btn" id="selectedTextAskBtn">Ask</div>

        <div class="chatbot-modal" id="chatbotModal">
          <div class="chatbot-header">
            <h3>🤖 Book Assistant</h3>
            <button class="chatbot-close" id="chatbotClose">×</button>
          </div>

          <div class="chatbot-messages" id="chatbotMessages">
            <div class="chatbot-message bot">
              <div class="bot-avatar">🤖</div>
              Hello! I'm your book assistant. Ask me anything about the content you're reading.
            </div>
          </div>

          <div class="chatbot-input-area">
            <textarea class="chatbot-input" id="chatbotInput" placeholder="Ask about the book..." rows="2"></textarea>
            <button class="chatbot-send-btn" id="chatbotSend">Send</button>
          </div>
        </div>
      `;

      document.body.appendChild(chatbotContainer);


      // Set up event listeners
      const toggleButton = document.getElementById('chatbotToggle');
      const modal = document.getElementById('chatbotModal');
      const closeButton = document.getElementById('chatbotClose');
      const messagesContainer = document.getElementById('chatbotMessages');
      const input = document.getElementById('chatbotInput');
      const sendButton = document.getElementById('chatbotSend');
      const selectedTextAskBtn = document.getElementById('selectedTextAskBtn');


      // Store selected text
      let selectedText = '';
      // Store the selected text that was used to trigger auto-send
      let autoSendSelectedText = null;
      // Session ID for chat persistence
      let sessionId = null;

      if (selectedTextAskBtn) {
        // Handle text selection
        document.addEventListener('mouseup', function() {
          const selection = window.getSelection();
          const selectedTextContent = selection.toString().trim();

          if (selectedTextContent) {
            selectedText = selectedTextContent;

            // Get selection position
            try {
              if (selection.rangeCount > 0) {
                const range = selection.getRangeAt(0);
                const rect = range.getBoundingClientRect();

                // Position the ask button above the selection
                selectedTextAskBtn.style.display = 'block';
                selectedTextAskBtn.style.left = (rect.left + rect.width / 2) + 'px';
                selectedTextAskBtn.style.top = rect.top + 'px';
              } else {
                selectedTextAskBtn.style.display = 'none';
              }
            } catch (e) {
              console.error('Error getting selection range:', e);
              // If no range is available (edge case), hide the button
              selectedTextAskBtn.style.display = 'none';
            }
          } else {
            selectedTextAskBtn.style.display = 'none';
            selectedText = '';
          }
        });

        // Hide the ask button when user clicks elsewhere
        document.addEventListener('mousedown', function(e) {
          if (!selectedTextAskBtn.contains(e.target) &&
              !modal.contains(e.target) &&
              !toggleButton.contains(e.target)) {
            selectedTextAskBtn.style.display = 'none';
          }
        });

        // Handle click on ask button
        selectedTextAskBtn.addEventListener('click', function() {
          if (selectedText) {
            // Show the chatbot modal
            modal.classList.add('active');

            // Focus on input and add a prompt
            // Store the selected text for auto-send processing
            autoSendSelectedText = selectedText;

            setTimeout(() => {
              input.focus();
              // Pre-fill with a summarization request for the selected text
              input.value = `Summarize this text: "${selectedText}"`;
              input.setSelectionRange(input.value.length, input.value.length); // Move cursor to end

              // Session is already available, send the message immediately
              setTimeout(() => {
                const currentValue = input.value.trim();
                if (currentValue) {
                  sendMessage();
                }
              }, 100); // Small delay to ensure UI updates
            }, 100);

            // Hide the ask button
            selectedTextAskBtn.style.display = 'none';
            selectedText = '';
          }
        });
      }

      if (toggleButton && modal && closeButton && messagesContainer && input && sendButton) {
        // Toggle chatbot
        toggleButton.addEventListener('click', () => {
          modal.classList.add('active');
        });

        // Close chatbot
        closeButton.addEventListener('click', () => {
          modal.classList.remove('active');
        });

        // Close when clicking outside
        modal.addEventListener('click', (e) => {
          if (e.target === modal) {
            modal.classList.remove('active');
          }
        });

        // Send message
        sendButton.addEventListener('click', sendMessage);
        input.addEventListener('keydown', (e) => {
          if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            sendMessage();
          }
        });

       const API_BASE_URL = window.CHATBOT_API_URL ||
                            (document.location.hostname === 'localhost' || document.location.hostname === '127.0.0.1' ?
                             'http://localhost:8000/api' :
                             'https://your-deployed-backend-url.com/api'); // Replace with your actual deployed URL

        sessionId = null;

        async function sendMessage() {
          const message = input.value.trim();
          if (!message) return;

          // Add user message
          addMessage(message, 'user');
          input.value = '';

          try {
            // Show typing indicator
            const typingDiv = document.createElement('div');
            typingDiv.className = 'chatbot-message bot';
            typingDiv.innerHTML = '<div style="display:flex;align-items:center;"><span style="margin-right:5px;">Typing</span><span>.</span><span>.</span><span>.</span></div>';
            messagesContainer.appendChild(typingDiv);
            messagesContainer.scrollTop = messagesContainer.scrollHeight;

            // Check if the message contains selected text context
            // Prioritize the autoSendSelectedText if available (for auto-send from button click)
            let isSelectedTextQuery = false;
            let query = message;
            let selectedText = null;

            // First check if this is an auto-send from the "Ask" button
            if (autoSendSelectedText) {
              isSelectedTextQuery = true;
              selectedText = autoSendSelectedText;
              query = 'Summarize this text'; // Default query for summarization
              // Clear the auto-send flag
              autoSendSelectedText = null;
            } else {
              // Fallback to the original parsing method for manual input
              isSelectedTextQuery = message.startsWith('About this text:') || message.startsWith('Summarize this text:');

              if (isSelectedTextQuery) {
                // Extract the selected text from the message (the text between the quotes)
                const quoteStart = message.indexOf('"');
                const quoteEnd = message.lastIndexOf('"');
                if (quoteStart !== -1 && quoteEnd !== -1 && quoteEnd > quoteStart) {
                  selectedText = message.substring(quoteStart + 1, quoteEnd);
                  // Use the remaining text as query or ask for clarification
                  let prefix = message.startsWith('About this text:') ? 'About this text:' : 'Summarize this text:';
                  query = message.substring(0, quoteStart).replace(prefix, '').trim() +
                          message.substring(quoteEnd + 1).trim();
                  query = query.trim() || (message.startsWith('Summarize this text:') ? 'Summarize this text' : 'Can you explain this text?');
                } else {
                  isSelectedTextQuery = false; // Treat as regular query if no quotes found
                }
              }
            }

            let requestBody = {};
            let response; // Declare response variable in the correct scope

            if (isSelectedTextQuery && selectedText) {
              requestBody = {
                query: query,
                selected_text: selectedText,
                session_id: sessionId,
                language: 'en',
              };
              response = await fetch(`${API_BASE_URL}/ask/selected-text`, {
                method: 'POST',
                headers: {
                  'Content-Type': 'application/json',
                },
                body: JSON.stringify(requestBody),
              });
            } else {
              requestBody = {
                query: message,
                session_id: sessionId,
                language: 'en',
                context_window: 5,
              };
              response = await fetch(`${API_BASE_URL}/ask`, {
                method: 'POST',
                headers: {
                  'Content-Type': 'application/json',
                },
                body: JSON.stringify(requestBody),
              });
            }

            // Remove typing indicator
            typingDiv.remove();

            if (response.ok) {
              const data = await response.json();
              addMessage(data.response, 'bot');
            } else {
              const errorText = await response.text();
              addMessage(`Sorry, I encountered an error (${response.status}): ${errorText || 'Please try again.'}`, 'bot');
            }
          } catch (error) {
            console.error('Error sending message:', error);
            // Remove typing indicator if it exists
            const typingElements = document.querySelectorAll('.chatbot-message.bot');
            if (typingElements.length > 0) {
              typingElements[typingElements.length - 1].remove();
            }
            addMessage(`Sorry, I encountered an error: ${error.message || 'Please try again.'}`, 'bot');
          }
        }

        function addMessage(text, sender) {
          const messageDiv = document.createElement('div');
          messageDiv.className = `chatbot-message ${sender}`;
          messageDiv.innerHTML = sender === 'bot' ? `<div class="bot-avatar">🤖</div>${text}` : text;
          messagesContainer.appendChild(messageDiv);

          // Scroll to bottom
          messagesContainer.scrollTop = messagesContainer.scrollHeight;
        }
      }

      // Cleanup function
      return () => {
        if (chatbotContainer && document.body.contains(chatbotContainer)) {
          document.body.removeChild(chatbotContainer);
        }
      };
    }
  }, []);

  return (
    <>
      <OriginalLayout {...props} />
    </>
  );
}