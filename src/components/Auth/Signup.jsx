import React, { useState } from 'react';
import { authService } from '../../services/authService';

const Signup = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    softwareBackground: 'beginner',
    hardwareBackground: 'beginner',
    preferredLanguage: 'English',
    aiSkillLevel: 'Beginner'
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    setLoading(true);

    try {
      // Extract onboarding data
      const { email, password, softwareBackground, hardwareBackground, preferredLanguage, aiSkillLevel } = formData;

      // Prepare onboarding data
      const onboardingData = {
        software_background: softwareBackground,
        hardware_background: hardwareBackground,
        preferred_language: preferredLanguage,
        ai_skill_level: aiSkillLevel
      };

      // Sign up with onboarding data
      await authService.signUp(email, password, onboardingData);

      // Redirect to dashboard or home
      window.location.href = '/';

    } catch (err) {
      setError(err.message || 'An error occurred during sign up');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-container">
      <h2>Sign Up</h2>
      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password:</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="softwareBackground">Software Background:</label>
          <select
            id="softwareBackground"
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="hardwareBackground">Hardware Background:</label>
          <select
            id="hardwareBackground"
            name="hardwareBackground"
            value={formData.hardwareBackground}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="preferredLanguage">Preferred Language:</label>
          <select
            id="preferredLanguage"
            name="preferredLanguage"
            value={formData.preferredLanguage}
            onChange={handleChange}
          >
            <option value="English">English</option>
            <option value="Urdu">Urdu</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="aiSkillLevel">AI Skill Level:</label>
          <select
            id="aiSkillLevel"
            name="aiSkillLevel"
            value={formData.aiSkillLevel}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <button type="submit" disabled={loading}>
          {loading ? 'Signing up...' : 'Sign Up'}
        </button>
      </form>

      <div className="auth-links">
        <p>Already have an account? <a href="/signin">Sign In</a></p>
      </div>
    </div>
  );
};

export default Signup;