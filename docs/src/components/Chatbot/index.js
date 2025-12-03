import React, { useState, useRef, useEffect } from 'react';
import styles from './styles.module.css';
import { apiRequest } from '../../utils/api';

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
  const [pendingText, setPendingText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle pending text when it changes
  useEffect(() => {
    if (pendingText) {
      console.log('📝 Setting input with pending text:', pendingText.substring(0, 50) + '...');

      // Ensure chatbot is open
      if (!isOpen) {
        setIsOpen(true);
      }

      // Use a short delay to ensure chatbot is rendered
      setTimeout(() => {
        setInput(pendingText);
        console.log('✅ Input field populated with:', pendingText.substring(0, 50) + '...');
        setPendingText(''); // Clear pending text

        // Focus input field
        setTimeout(() => {
          const inputField = document.querySelector('[data-chatbot-input]');
          if (inputField) {
            inputField.focus();
            console.log('✅ Input field focused');
          }
        }, 50);
      }, 100);
    }
  }, [pendingText, isOpen]);

  // Listen for text selection - show Ask button
  useEffect(() => {

    const handleSelection = () => {
      // Try to get selection from article first (Docusaurus content), then fallback to window
      const articleDoc = document.querySelector('article')?.ownerDocument || document;
      const sel = articleDoc.getSelection() || window.getSelection();
      const selection = sel?.toString().trim() || '';

      console.log('📝 Text selected, length:', selection?.length || 0);
      console.log('📝 Selection source:', articleDoc === document ? 'window' : 'article');

      if (selection && selection.length > 10) {
        console.log('✅ Selection valid, creating Ask button');
        const existingButton = document.getElementById('text-selection-ask-button');
        if (existingButton) {
          console.log('🗑️ Removing existing button');
          existingButton.remove();
        }

        // Get selection position for button placement
        const range = sel?.rangeCount ? sel.getRangeAt(0) : null;
        if (!range) {
          console.log('⚠️ No range found, cannot position button');
          return;
        }
        const rect = range.getBoundingClientRect();

        const button = document.createElement('button');
        button.id = 'text-selection-ask-button';
        button.textContent = '💬 Ask';
        button.style.cssText = `
          position: fixed;
          top: ${rect.top - 45}px;
          left: ${rect.left + (rect.width / 2) - 35}px;
          background: linear-gradient(135deg, #4f46e5, #7c3aed);
          color: white;
          border: none;
          border-radius: 8px;
          padding: 0.5rem 1rem;
          font-weight: 600;
          font-size: 0.9rem;
          cursor: pointer;
          z-index: 10001;
          box-shadow: 0 4px 12px rgba(79, 70, 229, 0.4);
          transition: transform 0.2s;
        `;

        button.onmouseover = () => button.style.transform = 'scale(1.05)';
        button.onmouseout = () => button.style.transform = 'scale(1)';

        button.onclick = (e) => {
          e.stopPropagation();
          console.log('💬 Ask button clicked! Opening chatbot...');
          console.log('📄 Selected text (captured):', selection.substring(0, 50) + '...');
          console.log('📄 Full selection length:', selection.length);

          // Store text - use the captured selection from closure
          const textToAsk = selection.substring(0, 200);
          const formattedText = `Selected text: "${textToAsk}${selection.length > 200 ? '...' : ''}"\n\nWhat do you want to know about it?`;

          console.log('📝 Formatted text:', formattedText);

          // Store full selection for context
          setSelectedText(selection);

          // Open chatbot first
          setIsOpen(true);
          console.log('✅ Chatbot opened');

          // Set input text directly after a delay to ensure chatbot is rendered
          setTimeout(() => {
            console.log('⏰ Timeout fired - setting input now');
            console.log('📝 Setting input to:', formattedText.substring(0, 50) + '...');
            setInput(formattedText);
            console.log('✅ setInput called');

            // Focus input field
            setTimeout(() => {
              const inputField = document.querySelector('[data-chatbot-input]');
              console.log('🔍 Input field found:', !!inputField);
              if (inputField) {
                console.log('📝 Input field value:', inputField.value.substring(0, 50));
                inputField.focus();
                inputField.setSelectionRange(inputField.value.length, inputField.value.length);
                console.log('✅ Input field focused and cursor positioned');
              }
            }, 100);
          }, 300);

          button.remove();
        };

        document.body.appendChild(button);
        console.log('✅ Ask button added to DOM');

        // Remove button after 5 seconds or when clicking elsewhere
        setTimeout(() => {
          if (button.parentNode) {
            console.log('⏱️ Ask button removed (timeout)');
            button.remove();
          }
        }, 5000);

        setTimeout(() => {
          document.addEventListener('click', (e) => {
            if (e.target !== button && button.parentNode) {
              console.log('🗑️ Ask button removed (clicked outside)');
              button.remove();
            }
          }, { once: true });
        }, 100);
      } else if (selection) {
        console.log('⚠️ Selection too short (need > 10 chars)');
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    const currentInput = input;
    setInput('');
    setIsLoading(true);

    try {
      // Try to get selection from article first, then fallback
      const articleDoc = document.querySelector('article')?.ownerDocument || document;
      const sel = articleDoc.getSelection() || window.getSelection();
      const contextText = selectedText || sel?.toString() || '';
      setSelectedText('');

      const data = await apiRequest('/chat', {
        method: 'POST',
        body: JSON.stringify({
          message: currentInput,
          context: contextText || null,
          conversation_history: messages.slice(-4),
          max_tokens: 150,
          temperature: 0.2
        })
      });

      setMessages(prev => [...prev, {
        role: 'assistant',
        content: data.response,
        sources: data.sources
      }]);
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Error: Could not connect to backend. Make sure the backend server is running at http://localhost:8000'
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
              onKeyPress={(e) => e.key === 'Enter' && !isLoading && sendMessage()}
              placeholder={isLoading ? "Sending..." : "Ask a question..."}
              disabled={isLoading}
              data-chatbot-input
            />
            <button onClick={sendMessage} disabled={isLoading || !input.trim()}>
              {isLoading ? '⏳' : '📤'}
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
