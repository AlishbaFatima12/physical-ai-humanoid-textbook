import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';
import { apiRequest, getBackendUrl } from '../../utils/api';

export default function ContentControls() {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [loading, setLoading] = useState(false);
  const [originalContent, setOriginalContent] = useState(null);
  const [chapterPath, setChapterPath] = useState('');

  // Get chapter path and load state
  useEffect(() => {
    const path = window.location.pathname.split('/').filter(Boolean).pop() || 'index';
    setChapterPath(path);
    setTimeout(() => loadSavedChapter(path), 300);
  }, []);

  // Detect URL changes
  useEffect(() => {
    const observer = new MutationObserver(() => {
      const newPath = window.location.pathname.split('/').filter(Boolean).pop() || 'index';
      if (newPath !== chapterPath) {
        setChapterPath(newPath);
        setIsPersonalized(false);
        setIsTranslated(false);
        setOriginalContent(null);
        setTimeout(() => loadSavedChapter(newPath), 500);
      }
    });

    observer.observe(document.body, { childList: true, subtree: true });
    return () => observer.disconnect();
  }, [chapterPath]);

  const getAuthHeaders = () => {
    const token = localStorage.getItem('authToken');
    return token ? { 'Authorization': `Bearer ${token}` } : {};
  };

  const loadSavedChapter = async (path) => {
    const token = localStorage.getItem('authToken');
    if (!token) return;

    try {
      const data = await apiRequest(`/chapters/${path}`, {
        method: 'GET'
      });
      if (data.status === 'success' && data.chapter) {
        let retries = 0;
        const waitForArticle = setInterval(() => {
          const article = document.querySelector('article');
          if (article || retries > 8) {
            clearInterval(waitForArticle);
            if (!article) return;

            if (!originalContent) setOriginalContent(article.innerHTML);

            if (data.chapter.is_personalized && data.chapter.personalized_content) {
              article.innerHTML = data.chapter.personalized_content;
              setIsPersonalized(true);
              setIsTranslated(false);
            } else if (data.chapter.is_translated && data.chapter.translated_content) {
              article.innerHTML = data.chapter.translated_content;
              setIsTranslated(true);
              setIsPersonalized(false);
            }
          }
          retries++;
        }, 100);
      }
    } catch (error) {
      console.log('Load error:', error);
    }
  };

  const saveChapter = async (personalizedContent, translatedContent) => {
    try {
      const content = getPageContent();
      await apiRequest('/chapters/save', {
        method: 'POST',
        body: JSON.stringify({
          chapter_path: chapterPath,
          original_content: originalContent || content,
          personalized_content: personalizedContent,
          translated_content: translatedContent,
          is_personalized: !!personalizedContent,
          is_translated: !!translatedContent
        })
      });
    } catch (error) {
      console.error('Error saving chapter:', error);
    }
  };

  const getPageContent = () => {
    const article = document.querySelector('article');
    return article ? article.innerHTML : '';
  };

  const setPageContent = (html) => {
    const article = document.querySelector('article');
    if (article) {
      article.innerHTML = html;
    }
  };

  const handlePersonalize = async () => {
    const user = JSON.parse(localStorage.getItem('user') || '{}');

    if (!user.software_background) {
      alert('Please log in to personalize content!');
      return;
    }

    if (isPersonalized) {
      // Restore original
      if (originalContent) {
        setPageContent(originalContent);
        setIsPersonalized(false);
        setIsTranslated(false);
        await saveChapter(null, null);
      }
      return;
    }

    setLoading(true);
    const content = getPageContent();
    if (!originalContent) {
      setOriginalContent(content);
    }

    try {
      // FAST PERSONALIZATION - Simple client-side adaptation
      const level = user.software_background || 'intermediate';

      let personalizedHTML = content;
      const article = document.querySelector('article');

      if (level === 'beginner') {
        personalizedHTML = `
          <div class="${styles.personalizedBanner}" style="background: linear-gradient(135deg, #3b82f6, #2563eb);">
            ✨ BEGINNER MODE - Simplified explanations
          </div>
          <div style="background: #eff6ff; padding: 1rem; border-left: 4px solid #3b82f6; margin: 1rem 0; border-radius: 8px;">
            <strong style="color: #1e40af;">💡 Beginner Tips:</strong>
            <ul style="margin: 0.5rem 0 0 0; padding-left: 1.5rem;">
              <li>Take it slow - understand each concept fully</li>
              <li>Try the code examples yourself</li>
              <li>Don't skip the exercises at the end</li>
            </ul>
          </div>
          ${content}
        `;
      } else if (level === 'advanced') {
        personalizedHTML = `
          <div class="${styles.personalizedBanner}" style="background: linear-gradient(135deg, #f59e0b, #d97706);">
            ✨ ADVANCED MODE - Deep technical insights
          </div>
          <div style="background: #fef3c7; padding: 1rem; border-left: 4px solid #f59e0b; margin: 1rem 0; border-radius: 8px;">
            <strong style="color: #92400e;">🚀 Advanced Challenges:</strong>
            <ul style="margin: 0.5rem 0 0 0; padding-left: 1.5rem;">
              <li>Research papers linked at bottom</li>
              <li>Try extending the code examples</li>
              <li>Consider performance optimizations</li>
            </ul>
          </div>
          ${content}
        `;
      } else {
        personalizedHTML = `
          <div class="${styles.personalizedBanner}" style="background: linear-gradient(135deg, #8b5cf6, #7c3aed);">
            ✨ INTERMEDIATE MODE - Standard explanations
          </div>
          ${content}
        `;
      }

      setPageContent(personalizedHTML);
      setIsPersonalized(true);
      setIsTranslated(false);
      await saveChapter(personalizedHTML, null);

    } catch (error) {
      console.error('Personalization error:', error);
      alert('Error personalizing content.');
    } finally {
      setLoading(false);
    }
  };

  const handleTranslate = async () => {
    console.log('🌍 Translate button clicked!', { isTranslated });

    if (isTranslated) {
      console.log('↩️ Restoring original content');
      if (originalContent) {
        setPageContent(originalContent);
        setIsTranslated(false);
        setIsPersonalized(false);
        await saveChapter(null, null);
      }
      return;
    }

    const content = getPageContent();
    console.log('📄 Got page content, length:', content?.length || 0);

    if (!content || content.trim().length === 0) {
      alert('⚠️ No content found to translate. Please wait for page to fully load.');
      return;
    }

    if (!originalContent) setOriginalContent(content);

    const instantHTML = `
      <div class="${styles.translatedBanner}">
        🌍 Translating to Urdu...
      </div>
      ${content}
    `;
    setPageContent(instantHTML);
    setIsTranslated(true);
    setLoading(true);

    try {
      // Try to get cached translation (will fail if not logged in, which is fine)
      try {
        const cachedData = await apiRequest(`/chapters/${chapterPath}`, { method: 'GET' });
        if (cachedData?.status === 'success' && cachedData.chapter?.translated_content) {
          console.log('✅ Found cached translation');
          setPageContent(cachedData.chapter.translated_content);
          setLoading(false);
          return;
        }
      } catch (cacheError) {
        console.log('ℹ️ No cached translation (this is normal)');
      }

      // Extract text content
      const tempDiv = document.createElement('div');
      tempDiv.innerHTML = content;
      const textContent = tempDiv.textContent || tempDiv.innerText;
      const contentToTranslate = textContent.substring(0, 2000).trim();

      console.log('📤 Sending to translate API, length:', contentToTranslate.length);

      const data = await apiRequest('/translate', {
        method: 'POST',
        body: JSON.stringify({
          content: contentToTranslate,
          target_language: 'ur'
        })
      });

      console.log('📥 Translation response:', data);

      if (data.translated_content) {
        const fullHTML = `
          <div class="${styles.translatedBanner}">
            🌍 Translated to Urdu
          </div>
          <div dir="rtl" lang="ur" style="padding: 1rem; line-height: 2; font-size: 1.1rem; background: #f8f9fa; border-radius: 8px;">
            ${data.translated_content.replace(/\\n/g, '<br>')}
          </div>
        `;

        setPageContent(fullHTML);
        console.log('✅ Translation displayed successfully');

        // Try to save (will fail if not logged in, which is fine)
        try {
          await saveChapter(null, fullHTML);
        } catch (saveError) {
          console.log('ℹ️ Could not save translation (login required)');
        }
      } else {
        console.error('❌ No translated_content in response');
        alert('Translation failed: No content received from server');
      }
      setLoading(false);
    } catch (error) {
      console.error('❌ Translation error:', error);
      alert(`Translation failed: ${error.message || 'Unknown error'}`);
      setPageContent(content);
      setIsTranslated(false);
      setLoading(false);
    }
  };

  const handleReset = async () => {
    if (originalContent) {
      setPageContent(originalContent);
      setIsPersonalized(false);
      setIsTranslated(false);

      try {
        await apiRequest(`/chapters/${chapterPath}`, {
          method: 'DELETE'
        });
      } catch (error) {
        console.error('Error resetting chapter:', error);
      }
    }
  };

  return (
    <div className={styles.controls}>
      <button
        onClick={handlePersonalize}
        disabled={loading}
        className={`${styles.button} ${isPersonalized ? styles.active : ''}`}
      >
        {loading ? '⏳ Loading...' : (isPersonalized ? '✓ Personalized' : '👤 Personalize for Me')}
      </button>

      <button
        onClick={handleTranslate}
        disabled={loading}
        className={`${styles.button} ${isTranslated ? styles.active : ''}`}
      >
        {loading ? '⏳ Loading...' : (isTranslated ? '✓ اردو میں' : '🌍 Translate to Urdu')}
      </button>

      {(isPersonalized || isTranslated) && (
        <button
          onClick={handleReset}
          disabled={loading}
          className={`${styles.button} ${styles.resetButton}`}
        >
          🔄 Reset Page
        </button>
      )}
    </div>
  );
}
