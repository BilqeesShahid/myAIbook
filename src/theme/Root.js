import React from 'react';

export default function Root({ children }) {
  // Make auth URL available globally
  if (typeof window !== 'undefined') {
    window.ENV = window.ENV || {};
    window.ENV.AUTH_URL = 'http://localhost:3001';
  }

  return (
    <>
      {children}
    </>
  );
}