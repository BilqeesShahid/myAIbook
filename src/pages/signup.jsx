import React from 'react';
import Layout from '@theme/Layout';
import Signup from '../components/Auth/Signup';

function SignupPage() {
  return (
    <Layout title="Sign Up" description="Create your account">
      <div
        style={{
          padding: '2rem',
          display: 'flex',
          justifyContent: 'center',
          minHeight: '80vh',
          backgroundColor: '#f9f9f9',
        }}
      >
        <div
          style={{
            display: 'flex',
            flexWrap: 'wrap',
            width: '100%',
            maxWidth: '1000px',
            gap: '3rem',
            alignItems: 'flex-start', // keep top alignment
          }}
        >
          {/* Image Section */}
          <div
            style={{
              flex: '1 1 400px',
              display: 'flex',
              justifyContent: 'center',
              marginTop: '6rem', // push image down
              borderBottom: '6px solid #28a745',
               borderTop: '6px solid #28a745', // thick green bottom border
              borderRadius: '12px 12px 12px 12px',
            }}
          >
            <img
              src="/img/docusaurus-social-card.jpg"
              alt="Sign Up Illustration"
              style={{
                width: '100%',
                maxWidth: '500px',
                height: 'auto',
                borderRadius: '12px',
                boxShadow: '0 4px 15px rgba(0,0,0,0.1)',
              }}
            />
          </div>

          {/* Form Section */}
          <div
            style={{
              flex: '1 1 400px',
              display: 'flex',
              justifyContent: 'center',
              borderBottom: '6px solid #28a745',
              borderRadius: '0 0 12px 12px',
            }}
          >
            <div
              style={{
                width: '100%',
                maxWidth: '450px',
                padding: '2rem',
                borderRadius: '12px',
                boxShadow: '0 4px 20px rgba(0,0,0,0.05)',
                backgroundColor: '#fff',
              }}
            >
              <Signup />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SignupPage;
