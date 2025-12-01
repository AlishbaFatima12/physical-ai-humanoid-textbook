import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import Chatbot from '../components/Chatbot';
import AuthModal from '../components/AuthModal';
import ContentControls from '../components/ContentControls';
import styles from './Root.module.css';

// This wraps the entire Docusaurus site and adds features to every page
export default function Root({children}) {
  const [user, setUser] = useState(null);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const location = useLocation();

  // Check if we're on a docs page (NOT the landing page)
  // Show features on all pages except the root landing page
  const isLandingPage = location.pathname === '/' ||
                        location.pathname === '/physical-ai-humanoid-textbook/' ||
                        location.pathname === '/physical-ai-humanoid-textbook';
  const isDocsPage = !isLandingPage;

  // Debug log
  useEffect(() => {
    console.log('Current pathname:', location.pathname);
    console.log('Is landing page:', isLandingPage);
    console.log('Is docs page (show features):', isDocsPage);
  }, [location.pathname, isLandingPage, isDocsPage]);

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

      {/* Content Controls (Personalize & Translate) - ONLY on chapter pages */}
      {isDocsPage && <ContentControls />}

      {/* Main content */}
      {children}

      {/* Chatbot - ONLY on chapter pages */}
      {isDocsPage && <Chatbot user={user} />}

      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onLogin={handleLogin}
      />
    </>
  );
}
