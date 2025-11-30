import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';

const BACKEND_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend-url.up.railway.app'  // UPDATE THIS after deploying backend
  : 'http://localhost:8000';

export default function Chatbot({ user }) {
  const [messages, setMessages] = useState([
    { role: 'assistant', content: user
      ? `Hello ${user.name}! I can answer questions about the Physical AI & Humanoid Robotics textbook, personalized for your ${user.software_background} level. Ask me anything!`
      : 'Hello! I can answer questions about the Physical AI & Humanoid Robotics textbook. Ask me anything!'
    }
  ]);
  const [input, setInput] = useState('');
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Listen for text selection - show options popup
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection().toString().trim();
      if (selection && selection.length > 10) {
        const existingPopup = document.getElementById('text-selection-popup');
        if (existingPopup) existingPopup.remove();

        const popup = document.createElement('div');
        popup.id = 'text-selection-popup';
        popup.style.cssText = `
          position: fixed;
          top: 50%;
          left: 50%;
          transform: translate(-50%, -50%);
          background: white;
          border-radius: 12px;
          box-shadow: 0 8px 32px rgba(0,0,0,0.3);
          padding: 1.5rem;
          z-index: 10001;
          max-width: 400px;
        `;

        popup.innerHTML = `
          <h3 style="margin: 0 0 1rem 0; color: #4f46e5;">💬 Ask about selected text</h3>
          <p style="margin: 0 0 1rem 0; color: #64748b; font-size: 0.9rem;">"${selection.substring(0, 80)}..."</p>
          <div style="display: flex; flex-direction: column; gap: 0.5rem;">
            <button id="option-explain" style="padding: 0.75rem; background: #4f46e5; color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
              📖 Explain in 3-5 points
            </button>
            <button id="option-summarize" style="padding: 0.75rem; background: #7c3aed; color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
              📝 Summarize this
            </button>
            <button id="option-custom" style="padding: 0.75rem; background: #06b6d4; color: white; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
              ❓ Ask custom question
            </button>
            <button id="option-cancel" style="padding: 0.75rem; background: #e5e7eb; color: #374151; border: none; border-radius: 8px; cursor: pointer; font-weight: 600;">
              ✖ Cancel
            </button>
          </div>
        `;

        document.body.appendChild(popup);

        const handleAction = async (prompt) => {
          popup.remove();
          setSelectedText(selection);
          setIsOpen(true);
          setInput(prompt);
          // Auto-send after a short delay
          setTimeout(() => {
            const sendBtn = document.querySelector('[class*="chatbotInput"] button');
            if (sendBtn) sendBtn.click();
          }, 300);
        };

        document.getElementById('option-explain').onclick = () =>
          handleAction(`Explain in 3-5 points: "${selection}"`);

        document.getElementById('option-summarize').onclick = () =>
          handleAction(`Summarize in 3-5 points: "${selection}"`);

        document.getElementById('option-custom').onclick = () => {
          popup.remove();
          setSelectedText(selection);
          setIsOpen(true);
          setInput(`About: "${selection.substring(0, 100)}..." - `);
        };

        document.getElementById('option-cancel').onclick = () => popup.remove();

        setTimeout(() => {
          document.addEventListener('click', (e) => {
            if (!popup.contains(e.target)) popup.remove();
          }, { once: true });
        }, 100);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, [setIsOpen, setInput, setSelectedText]);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    const currentInput = input;
    setInput('');
    setIsLoading(true);

    try {
      const contextText = selectedText || window.getSelection().toString();
      setSelectedText('');

      const authToken = localStorage.getItem('authToken');
      const headers = { 'Content-Type': 'application/json' };
      if (authToken) headers['Authorization'] = `Bearer ${authToken}`;

      const response = await fetch(`${BACKEND_URL}/chat`, {
        method: 'POST',
        headers,
        body: JSON.stringify({
          message: currentInput,
          context: contextText || null,
          conversation_history: messages.slice(-4)
        })
      });

      const data = await response.json();

      setMessages(prev => [...prev, {
        role: 'assistant',
        content: data.response,
        sources: data.sources
      }]);
    } catch (error) {
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Error: Backend server not running'
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Chatbot Toggle Button */}
      <button
        className={styles.chatbotToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Toggle chatbot"
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chatbot Window */}
      {isOpen && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatbotHeader}>
            <h3>🤖 AI Assistant</h3>
            <p>Ask questions about the textbook</p>
          </div>

          <div className={styles.chatbotMessages}>
            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${styles[msg.role]}`}
              >
                <div className={styles.messageContent}>
                  {msg.content}
                </div>
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <small>📚 Sources: {msg.sources.map(s => `Ch ${s.chapter}`).join(', ')}</small>
                  </div>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.loading}>Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className={styles.chatbotInput}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
              placeholder="Ask a question..."
              disabled={isLoading}
            />
            <button onClick={sendMessage} disabled={isLoading || !input.trim()}>
              Send
            </button>
          </div>

          <div className={styles.chatbotHint}>
            <small>💡 Tip: Select text on the page before asking to get context-aware answers!</small>
          </div>
        </div>
      )}
    </>
  );
}
