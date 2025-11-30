import React, { useState, useEffect } from 'react';
import Chatbot from '../components/Chatbot';
import AuthModal from '../components/AuthModal';
import ContentControls from '../components/ContentControls';
import styles from './Root.module.css';

// This wraps the entire Docusaurus site and adds features to every page
export default function Root({children}) {
  const [user, setUser] = useState(null);
  const [showAuthModal, setShowAuthModal] = useState(false);

  useEffect(() => {
    // Check if user is logged in
    const storedUser = localStorage.getItem('user');
    if (storedUser) {
      setUser(JSON.parse(storedUser));
    }
  }, []);

  const handleLogin = (userData) => {
    setUser(userData);
  };

  const handleLogout = () => {
    localStorage.removeItem('authToken');
    localStorage.removeItem('user');
    setUser(null);
  };

  return (
    <>
      {/* Auth Button in top-right corner */}
      <div className={styles.authButton}>
        {user ? (
          <div className={styles.userMenu}>
            <span className={styles.userName}>👋 {user.name}</span>
            <button onClick={handleLogout} className={styles.logoutBtn}>
              Logout
            </button>
          </div>
        ) : (
          <button onClick={() => setShowAuthModal(true)} className={styles.loginBtn}>
            🔐 Login / Sign Up
          </button>
        )}
      </div>

      {/* Content Controls (Personalize & Translate) */}
      <ContentControls />

      {/* Main content */}
      {children}

      {/* Chatbot */}
      <Chatbot user={user} />

      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onLogin={handleLogin}
      />
    </>
  );
}
