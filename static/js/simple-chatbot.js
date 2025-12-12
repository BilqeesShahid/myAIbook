// Simple Chatbot Widget for Docusaurus
(function() {
    'use strict';

    console.log('Chatbot script loaded');

    // Wait for the page to be fully loaded
    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initializeChatbot);
    } else {
        initializeChatbot();
    }

    function initializeChatbot() {
        // Create the chatbot widget
        const chatbotContainer = document.createElement('div');
        chatbotContainer.id = 'simple-chatbot';
        chatbotContainer.innerHTML = `
            <style>
                #simple-chatbot {
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
                    animation: float 3s ease-in-out infinite;
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
                    top: 0;
                    left: 0;
                    right: 0;
                    bottom: 0;
                    background-color: rgba(0, 0, 0, 0.5);
                    z-index: 10001;
                    display: none;
                    align-items: center;
                    justify-content: center;
                }

                .chatbot-modal.active {
                    display: flex;
                }

                .chatbot-window {
                    width: 80%;
                    max-width: 600px;
                    height: 70%;
                    max-height: 700px;
                    background-color: white;
                    border-radius: 12px;
                    display: flex;
                    flex-direction: column;
                    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.3);
                    overflow: hidden;
                }

                .chatbot-header {
                    background-color: #4CAF50;
                    color: white;
                    padding: 15px 20px;
                    display: flex;
                    justify-content: space-between;
                    align-items: center;
                }

                .chatbot-close {
                    background: none;
                    border: none;
                    color: white;
                    font-size: 24px;
                    cursor: pointer;
                    width: 30px;
                    height: 30px;
                    display: flex;
                    align-items: center;
                    justify-content: center;
                    border-radius: 50%;
                }

                .chatbot-close:hover {
                    background-color: rgba(255, 255, 255, 0.2);
                }

                .chatbot-messages {
                    flex: 1;
                    padding: 20px;
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
                    border-bottom-left-radius: 4px;
                }

                .chatbot-input-area {
                    padding: 20px;
                    border-top: 1px solid #eee;
                    background-color: white;
                }

                .chatbot-input {
                    width: 100%;
                    padding: 12px 15px;
                    border: 1px solid #ddd;
                    border-radius: 24px;
                    resize: none;
                    font-size: 14px;
                    font-family: inherit;
                    margin-bottom: 10px;
                }

                .chatbot-send-btn {
                    padding: 10px 20px;
                    background-color: #4CAF50;
                    color: white;
                    border: none;
                    border-radius: 20px;
                    cursor: pointer;
                    font-weight: bold;
                }

                @keyframes float {
                    0% { transform: translateY(0px); }
                    50% { transform: translateY(-5px); }
                    100% { transform: translateY(0px); }
                }
            </style>

            <button class="chatbot-button" id="chatbotToggle">💬</button>

            <div class="chatbot-modal" id="chatbotModal">
                <div class="chatbot-window">
                    <div class="chatbot-header">
                        <h3>Book Assistant</h3>
                        <button class="chatbot-close" id="chatbotClose">×</button>
                    </div>

                    <div class="chatbot-messages" id="chatbotMessages">
                        <div class="chatbot-message bot">
                            Hello! I'm your book assistant. Ask me anything about the content you're reading.
                        </div>
                    </div>

                    <div class="chatbot-input-area">
                        <textarea class="chatbot-input" id="chatbotInput" placeholder="Ask about the book..." rows="2"></textarea>
                        <button class="chatbot-send-btn" id="chatbotSend">Send</button>
                    </div>
                </div>
            </div>
        `;

        document.body.appendChild(chatbotContainer);

        // Get elements
        const toggleButton = document.getElementById('chatbotToggle');
        const modal = document.getElementById('chatbotModal');
        const closeButton = document.getElementById('chatbotClose');
        const messagesContainer = document.getElementById('chatbotMessages');
        const input = document.getElementById('chatbotInput');
        const sendButton = document.getElementById('chatbotSend');

        // Toggle chatbot
        toggleButton.addEventListener('click', function() {
            modal.classList.add('active');
        });

        // Close chatbot
        closeButton.addEventListener('click', function() {
            modal.classList.remove('active');
        });

        // Close when clicking outside
        modal.addEventListener('click', function(e) {
            if (e.target === modal) {
                modal.classList.remove('active');
            }
        });

        // Send message
        sendButton.addEventListener('click', sendMessage);
        input.addEventListener('keydown', function(e) {
            if (e.key === 'Enter' && !e.shiftKey) {
                e.preventDefault();
                sendMessage();
            }
        });

        // API service for backend communication
        // Use environment-based URL - falls back to localhost for development
        const API_BASE_URL = window.CHATBOT_API_URL ||
                            (document.location.hostname === 'localhost' || document.location.hostname === '127.0.0.1' ?
                             'http://localhost:8000/api' :
                             'https://your-deployed-backend-url.com/api'); // Replace with your actual deployed URL
        let sessionId = null;

        // Initialize session
        async function initSession() {
            try {
                const response = await fetch(`${API_BASE_URL}/session/new`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        user_id: null,
                        session_title: 'Book Assistant Chat'
                    }),
                });

                if (response.ok) {
                    const data = await response.json();
                    sessionId = data.session_id;
                } else {
                    // Generate fallback session ID
                    sessionId = `session_${Date.now()}`;
                }
            } catch (error) {
                console.error('Error creating session:', error);
                sessionId = `session_${Date.now()}`;
            }
        }

        // Initialize session when script loads
        initSession();

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

                // Call the API
                const response = await fetch(`${API_BASE_URL}/ask`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({
                        query: message,
                        session_id: sessionId,
                        language: 'en',
                        context_window: 5,
                    }),
                });

                // Remove typing indicator
                typingDiv.remove();

                if (response.ok) {
                    const data = await response.json();
                    addMessage(data.response, 'bot');
                } else {
                    addMessage('Sorry, I encountered an error processing your request. Please try again.', 'bot');
                }
            } catch (error) {
                console.error('Error sending message:', error);
                // Remove typing indicator if it exists
                const typingElements = document.querySelectorAll('.chatbot-message.bot');
                if (typingElements.length > 0) {
                    typingElements[typingElements.length - 1].remove();
                }
                addMessage('Sorry, I encountered an error processing your request. Please try again.', 'bot');
            }
        }

        function addMessage(text, sender) {
            const messageDiv = document.createElement('div');
            messageDiv.className = `chatbot-message ${sender}`;
            messageDiv.textContent = text;
            messagesContainer.appendChild(messageDiv);

            // Scroll to bottom
            messagesContainer.scrollTop = messagesContainer.scrollHeight;
        }
    }
})();