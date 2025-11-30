import React, { useState } from 'react';
import styles from './styles.module.css';

const BACKEND_URL = process.env.NODE_ENV === 'production'
  ? 'https://physical-ai-humanoid-textbook.onrender.com'
  : 'http://localhost:8000';

export default function AuthModal({ isOpen, onClose, onLogin }) {
  const [isSignup, setIsSignup] = useState(false);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    software_background: 'intermediate',
    hardware_background: 'hobby',
    programming_languages: ['Python'],
    robotics_experience: 'some'
  });

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    const endpoint = isSignup ? '/signup' : '/login';
    const payload = isSignup ? formData : {
      email: formData.email,
      password: formData.password
    };

    try {
      const response = await fetch(`${BACKEND_URL}${endpoint}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(payload)
      });

      const data = await response.json();

      if (response.ok) {
        localStorage.setItem('authToken', data.access_token);
        localStorage.setItem('user', JSON.stringify(data.user));
        onLogin(data.user);
        onClose();
      } else {
        setError(data.detail || 'Authentication failed');
      }
    } catch (err) {
      setError('Connection error. Make sure backend is running.');
    } finally {
      setLoading(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={e => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose}>×</button>

        <h2>{isSignup ? '📝 Sign Up' : '🔐 Log In'}</h2>

        {error && <div className={styles.error}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.form}>
          {isSignup && (
            <>
              <div className={styles.formGroup}>
                <label>Name:</label>
                <input
                  type="text"
                  placeholder="Your full name"
                  value={formData.name}
                  onChange={e => setFormData({...formData, name: e.target.value})}
                  required
                />
              </div>

              <div className={styles.formGroup}>
                <label>Software Background:</label>
                <select
                  value={formData.software_background}
                  onChange={e => setFormData({...formData, software_background: e.target.value})}
                >
                  <option value="beginner">Beginner (Just started programming)</option>
                  <option value="intermediate">Intermediate (1-3 years experience)</option>
                  <option value="advanced">Advanced (3+ years experience)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label>Hardware Experience:</label>
                <select
                  value={formData.hardware_background}
                  onChange={e => setFormData({...formData, hardware_background: e.target.value})}
                >
                  <option value="none">None (No hardware experience)</option>
                  <option value="hobby">Hobby (Arduino, Raspberry Pi)</option>
                  <option value="professional">Professional (Commercial robotics)</option>
                </select>
              </div>

              <div className={styles.formGroup}>
                <label>Robotics Experience:</label>
                <select
                  value={formData.robotics_experience}
                  onChange={e => setFormData({...formData, robotics_experience: e.target.value})}
                >
                  <option value="none">None (Complete beginner)</option>
                  <option value="some">Some (Taken courses, built simple bots)</option>
                  <option value="extensive">Extensive (Professional/research experience)</option>
                </select>
              </div>
            </>
          )}

          <div className={styles.formGroup}>
            <label>Email:</label>
            <input
              type="email"
              placeholder="your@email.com"
              value={formData.email}
              onChange={e => setFormData({...formData, email: e.target.value})}
              required
            />
          </div>

          <div className={styles.formGroup}>
            <label>Password:</label>
            <input
              type="password"
              placeholder="••••••••"
              value={formData.password}
              onChange={e => setFormData({...formData, password: e.target.value})}
              required
              minLength={6}
            />
          </div>

          <button type="submit" className={styles.submitButton} disabled={loading}>
            {loading ? 'Loading...' : (isSignup ? 'Create Account' : 'Log In')}
          </button>
        </form>

        <div className={styles.switchMode}>
          {isSignup ? 'Already have an account?' : "Don't have an account?"}
          {' '}
          <button onClick={() => setIsSignup(!isSignup)} className={styles.linkButton}>
            {isSignup ? 'Log In' : 'Sign Up'}
          </button>
        </div>
      </div>
    </div>
  );
}
