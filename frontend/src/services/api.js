// API service for communicating with the backend RAG system

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api';

class ApiService {
  constructor() {
    this.baseURL = API_BASE_URL;
  }

  // Make a request to the ask endpoint
  async ask(query, sessionId = null, language = 'en', contextWindow = 5) {
    try {
      const response = await fetch(`${this.baseURL}/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          session_id: sessionId,
          language,
          context_window: contextWindow,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error making ask request:', error);
      throw error;
    }
  }

  // Make a request to the selected text endpoint
  async askSelectedText(query, selectedText, sessionId = null, language = 'en', contextWindow = 5) {
    console.log('Making askSelectedText API call with:', { query, selectedText, sessionId });
    try {
      const response = await fetch(`${this.baseURL}/ask/selected-text`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query,
          selected_text: selectedText,
          session_id: sessionId,
          language,
          context_window: contextWindow,
        }),
      });

      console.log('Response status:', response.status);
      if (!response.ok) {
        const errorText = await response.text();
        console.error('API error response:', errorText);
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorText}`);
      }

      const result = await response.json();
      console.log('API response data:', result);
      return result;
    } catch (error) {
      console.error('Error making ask selected text request:', error);
      throw error;
    }
  }

  // Get session history
  async getSessionHistory(sessionId) {
    try {
      const response = await fetch(`${this.baseURL}/session/${sessionId}/history`);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting session history:', error);
      throw error;
    }
  }

  // Create a new session
  async createSession(userId = null, sessionTitle = null) {
    try {
      const response = await fetch(`${this.baseURL}/session/new`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: userId,
          session_title: sessionTitle,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error creating session:', error);
      throw error;
    }
  }
}

const apiService = new ApiService();
export default apiService;