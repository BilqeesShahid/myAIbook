import React from 'react';
import Layout from '@theme/Layout';
import Signin from '../components/Auth/Signin';

function SigninPage() {
  return (
    <Layout title="Sign In" description="Sign in to your account">
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
            alignItems: 'flex-start',
          }}
        >
          {/* Image Section */}
          <div
            style={{
              flex: '1 1 400px',
              display: 'flex',
              justifyContent: 'center',
              marginTop: '4rem', // adjust to align with form
              borderBottom: '6px solid #28a745', // thick green bottom border
               borderTop: '6px solid #28a745', // thick green bottom border
              borderRadius: '12px 12px 12px 12px',
            }}
          >
            <img
              src="/img/undraw_docusaurus_tree.svg"
              alt="Sign In Illustration"
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
              borderBottom: '6px solid #28a745', // thick green bottom border
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
              <Signin />
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SigninPage;
