# ⚡ QUICK DEPLOYMENT GUIDE - 30 MINUTES

**Everything is ready! Just follow these steps in order.**

---

## 📋 Prerequisites Checklist

Before starting, get these ready:

- [ ] OpenAI API key (from platform.openai.com)
- [ ] Qdrant Cloud account & cluster (free tier at cloud.qdrant.io)
- [ ] Neon PostgreSQL database (free at neon.tech)
- [ ] GitHub account
- [ ] Railway account (railway.app) OR Render account (render.com)

---

## 🚀 3-STEP DEPLOYMENT

### STEP 1: Configure Backend (10 min)

1. **Copy the example file:**
   ```bash
   cd backend
   cp .env.example .env
   ```

2. **Edit `backend/.env` and fill in:**
   ```env
   OPENAI_API_KEY=sk-your-key-from-openai
   DATABASE_URL=postgresql://user:pass@your-neon-db/db?sslmode=require
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-key
   SECRET_KEY=generate-a-random-32-char-string-here
   CORS_ORIGINS=http://localhost:3000,https://alishbafatima12.github.io
   ```

3. **Test locally:**
   ```bash
   pip install -r requirements.txt
   python -m uvicorn app.main:app --reload
   ```

4. **Visit:** http://localhost:8000/docs (should see API docs)

---

### STEP 2: Deploy Backend to Railway (10 min)

1. Go to https://railway.app
2. Click "Start a New Project" → "Deploy from GitHub repo"
3. Select `physical-ai-humanoid-textbook`
4. Set root directory: `backend`
5. Add all environment variables from your `.env` file
6. Deploy and wait 3-5 minutes
7. **Copy your Railway URL** (e.g., `https://your-app.up.railway.app`)

8. **Initialize database & vector store:**
   ```bash
   curl -X POST https://your-app.up.railway.app/ingest
   ```
   (This takes 2-3 minutes - wait for completion)

---

### STEP 3: Deploy Frontend to GitHub Pages (10 min)

1. **Update backend URL in 3 files:**

   Edit `docs/src/components/Chatbot/index.js` line 4-6:
   ```javascript
   const BACKEND_URL = process.env.NODE_ENV === 'production'
     ? 'https://your-app.up.railway.app'  // YOUR RAILWAY URL HERE
     : 'http://localhost:8000';
   ```

   Edit `docs/src/components/AuthModal/index.js` line 4-6:
   ```javascript
   const BACKEND_URL = process.env.NODE_ENV === 'production'
     ? 'https://your-app.up.railway.app'  // YOUR RAILWAY URL HERE
     : 'http://localhost:8000';
   ```

   Edit `docs/src/components/ContentControls/index.js` line 4-6:
   ```javascript
   const BACKEND_URL = process.env.NODE_ENV === 'production'
     ? 'https://your-app.up.railway.app'  // YOUR RAILWAY URL HERE
     : 'http://localhost:8000';
   ```

2. **Commit changes:**
   ```bash
   git add .
   git commit -m "Deploy hackathon project with RAG chatbot"
   git push origin main
   ```

3. **Deploy to GitHub Pages:**
   ```bash
   cd docs
   npm install
   GIT_USER=AlishbaFatima12 npm run deploy
   ```

4. **Visit:** https://alishbafatima12.github.io/physical-ai-humanoid-textbook/

---

## ✅ TEST EVERYTHING

On your live site, test:

1. **Chatbot:** Click bubble → ask "What is Physical AI?"
2. **Login:** Click "Login/Sign Up" → create account
3. **Personalize:** Click "Personalize for Me" button on any chapter
4. **Translate:** Click "Translate to Urdu" button
5. **Reset:** Click "Reset Page" to restore original
6. **Persistence:** Refresh page → personalized content should remain

---

## 🎯 FEATURES CHECKLIST

- [x] RAG Chatbot with OpenAI + Qdrant
- [x] User authentication with JWT
- [x] Background questions (software/hardware level)
- [x] Content personalization per user level
- [x] Urdu translation
- [x] Chapter state persistence (saved to database)
- [x] Reset functionality
- [x] Context-aware chat (select text → ask about it)
- [x] Conversation history
- [x] Source citations
- [x] Responsive UI
- [x] Dark mode support

---

## 📊 HACKATHON POINTS

| Feature | Points | Status |
|---------|--------|--------|
| RAG Chatbot | 100 | ✅ |
| Authentication | +50 | ✅ |
| Personalization | +50 | ✅ |
| Translation | +50 | ✅ |
| **TOTAL** | **250+** | **✅** |

---

## 🐛 Common Issues & Fixes

### "CORS error" on deployed site
**Fix:** Update `CORS_ORIGINS` in Railway to include your GitHub Pages URL:
```
CORS_ORIGINS=https://alishbafatima12.github.io
```

### "Cannot connect to backend"
**Check:**
1. Railway deployment succeeded
2. Backend URL is correct in frontend files
3. Backend is accessible: `curl https://your-backend-url/health`

### "Collection not found"
**Fix:** Run ingestion:
```bash
curl -X POST https://your-backend-url/ingest
```

---

## 🎉 YOU'RE DONE!

Your complete hackathon project is now live with:
- ✅ Full-stack RAG application
- ✅ Production database (Neon PostgreSQL)
- ✅ Vector search (Qdrant Cloud)
- ✅ AI-powered features (OpenAI)
- ✅ Professional UI/UX
- ✅ Deployed and accessible online

**Project URL:** https://alishbafatima12.github.io/physical-ai-humanoid-textbook/

---

## 📸 Screenshots for Submission

Take these screenshots:
1. Homepage with chatbot bubble
2. Chatbot conversation
3. Login/signup modal
4. Personalized content
5. Urdu translation
6. Backend API docs: `https://your-backend-url/docs`

---

**Good luck with your hackathon! 🚀**
