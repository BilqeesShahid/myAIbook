import React, { useState } from 'react';

const SignupForm = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [softwareExperience, setSoftwareExperience] = useState('beginner');
  const [hardwareExperience, setHardwareExperience] = useState('beginner');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      // First, sign up the user with Better Auth
      const signUpResponse = await fetch('http://localhost:3001/api/auth/signup/email-password', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          name: email.split('@')[0] // Use part of email as name
        }),
      });

      if (!signUpResponse.ok) {
        const errorData = await signUpResponse.json();
        setError(errorData.error || 'Registration failed');
        setLoading(false);
        return;
      }

      const signUpResult = await signUpResponse.json();

      // After successful signup, update user profile with background info
      // using our custom endpoint
      const token = document.cookie.split('; ').find(row => row.startsWith('better-auth-session='))?.split('=')[1];

      if (token) {
        // Wait a moment for the session to be established
        setTimeout(async () => {
          const backgroundResponse = await fetch('http://localhost:3001/api/user/background', {
            method: 'PUT',
            headers: {
              'Content-Type': 'application/json',
              'Authorization': `Bearer ${token}`,
            },
            body: JSON.stringify({
              software_experience: softwareExperience,
              hardware_experience: hardwareExperience
            }),
          });

          if (!backgroundResponse.ok) {
            console.error('Failed to update user background:', await backgroundResponse.text());
          } else {
            console.log('User background updated successfully');
          }
        }, 1000); // Wait 1 second to ensure session is established
      }

      // If successful, redirect to home
      alert('Registration successful!');
      window.location.href = '/';

    } catch (err) {
      setError(err.message || 'An error occurred during signup');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <h2>Sign Up</h2>
      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            disabled={loading}
          />
        </div>

        <div className="form-group">
          <label htmlFor="softwareExperience">Software Experience Level:</label>
          <select
            id="softwareExperience"
            value={softwareExperience}
            onChange={(e) => setSoftwareExperience(e.target.value)}
            disabled={loading}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="hardwareExperience">Hardware Experience Level:</label>
          <select
            id="hardwareExperience"
            value={hardwareExperience}
            onChange={(e) => setHardwareExperience(e.target.value)}
            disabled={loading}
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
    </div>
  );
};

export default SignupForm;