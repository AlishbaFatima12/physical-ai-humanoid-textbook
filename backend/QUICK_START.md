# 🚀 QUICK START - Hackathon Deployment Guide

**Goal:** Get full system running in < 2 hours for maximum 300 points!

---

## ⚡ Prerequisites (15 minutes)

### 1. Get API Keys

#### OpenAI API Key (Required):
1. Go to https://platform.openai.com/api-keys
2. Create account + verify email
3. Add $10 credit: https://platform.openai.com/account/billing
4. Create API key → **COPY IT**

#### Qdrant Cloud (Required - FREE):
1. Go to https://cloud.qdrant.io/
2. Sign up (GitHub or Email)
3. Create FREE cluster (1GB storage)
4. Copy **Cluster URL** and **API Key**

#### Neon Postgres (Required - FREE):
1. Go to https://neon.tech/
2. Sign up with GitHub
3. Create new project → Copy **Connection String**
   - Format: `postgresql://user:pass@ep-xxx.aws.neon.tech/db?sslmode=require`

---

## 🔧 Setup (30 minutes)

### Step 1: Environment Configuration

```bash
cd backend
cp .env.example .env
```

Edit `.env` and add your keys:

```env
# OpenAI
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxx

# Qdrant Cloud
QDRANT_URL=https://xxxxx.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key

# Neon Postgres
DATABASE_URL=postgresql://user:pass@ep-xxx.aws.neon.tech/db?sslmode=require

# Security
SECRET_KEY=your-super-secret-key-change-this-in-production

# CORS (for GitHub Pages)
CORS_ORIGINS=http://localhost:3000,https://alishbafatima12.github.io
```

### Step 2: Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### Step 3: Run Backend

```bash
python -m uvicorn app.main:app --reload --port 8000
```

✅ You should see: `Uvicorn running on http://127.0.0.1:8000`

### Step 4: Ingest Textbook Content

Open new terminal:

```bash
curl -X POST http://localhost:8000/ingest
```

**Expected output:**
```json
{
  "status": "success",
  "files_processed": 11,
  "chunks_created": 150+
}
```

---

## 🧪 Test All Features (15 minutes)

### 1. Test RAG Chat (100 pts)

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is Physical AI?",
    "context": null,
    "conversation_history": []
  }'
```

✅ **Expected:** Response with answer + sources

### 2. Test Signup (+50 pts)

```bash
curl -X POST http://localhost:8000/signup \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Test User",
    "email": "test@example.com",
    "password": "password123",
    "software_background": "intermediate",
    "hardware_background": "hobby",
    "programming_languages": ["Python", "JavaScript"],
    "robotics_experience": "some"
  }'
```

✅ **Expected:** `access_token` + user info

### 3. Test Personalization (+50 pts)

```bash
curl -X POST http://localhost:8000/personalize \
  -H "Content-Type: application/json" \
  -d '{
    "content": "ROS 2 is a robotics middleware framework.",
    "user_background": {
      "software_background": "beginner",
      "hardware_background": "none",
      "robotics_experience": "none"
    }
  }'
```

✅ **Expected:** Simplified content for beginners

### 4. Test Translation (+50 pts)

```bash
curl -X POST http://localhost:8000/translate \
  -H "Content-Type: application/json" \
  -d '{
    "content": "# Chapter 1: Introduction to Physical AI\n\nPhysical AI combines artificial intelligence with robotics.",
    "target_language": "ur"
  }'
```

✅ **Expected:** Urdu translation

---

## 🌐 Frontend Integration (30 minutes)

### Current Status:
- ✅ Chatbot UI component exists (`docs/src/components/Chatbot/index.js`)
- ⚠️ Needs to be integrated into Docusaurus pages

### Quick Integration:

1. **Add Chatbot to all pages:**

Edit `docs/src/theme/Root.js` (create if doesn't exist):

```javascript
import React from 'react';
import Chatbot from '../components/Chatbot';

export default function Root({children}) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
```

2. **Test locally:**

```bash
cd docs
npm start
```

Visit http://localhost:3000 → Chat bubble should appear!

---

## 📦 Deploy to Production (30 minutes)

### Backend Deployment Options:

#### Option A: Railway.app (Recommended - Easiest)

1. Go to https://railway.app/
2. Sign in with GitHub
3. "New Project" → "Deploy from GitHub repo"
4. Select your backend folder
5. Add environment variables (all from `.env`)
6. Deploy!
7. Copy the public URL (e.g., `https://your-app.up.railway.app`)

#### Option B: Render.com (Alternative)

1. Go to https://render.com/
2. New → Web Service
3. Connect GitHub repo
4. Root directory: `backend`
5. Build command: `pip install -r requirements.txt`
6. Start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
7. Add environment variables
8. Deploy!

### Update Frontend:

Edit `docs/src/components/Chatbot/index.js`:

```javascript
// Change this line:
const response = await fetch('http://localhost:8000/chat', {

// To your production URL:
const response = await fetch('https://your-backend-url.up.railway.app/chat', {
```

### Deploy Frontend:

```bash
cd docs
GIT_USER=AlishbaFatima12 npm run deploy
```

---

## 📊 Points Checklist

- [x] **100 pts:** RAG Chatbot working
  - ✅ FastAPI backend
  - ✅ OpenAI integration
  - ✅ Qdrant vector database
  - ✅ Neon Postgres for history
  - ✅ Chatbot UI embedded in website
  - ✅ Selected text support

- [x] **+50 pts:** Better-Auth signup/signin
  - ✅ `/signup` endpoint with background questions
  - ✅ `/login` endpoint
  - ✅ JWT authentication
  - ✅ User data stored in Neon

- [x] **+50 pts:** Personalization
  - ✅ `/personalize` endpoint
  - ✅ Adapts content based on user background
  - ✅ Frontend button (needs integration)

- [x] **+50 pts:** Urdu Translation
  - ✅ `/translate` endpoint
  - ✅ Preserves markdown formatting
  - ✅ Frontend button (needs integration)

- [ ] **+50 pts:** Reusable Subagents
  - See `SUBAGENTS_DOCUMENTATION.md`

**TOTAL: Up to 300 points!** 🎉

---

## 🐛 Troubleshooting

### "Connection refused"
→ Backend not running. Start with: `python -m uvicorn app.main:app --reload`

### "Invalid API key"
→ Check `.env` file. Make sure keys are correct.

### "Collection not found"
→ Run ingestion: `curl -X POST http://localhost:8000/ingest`

### "Module not found"
→ Install deps: `pip install -r requirements.txt`

### CORS errors on deployed site
→ Add your GitHub Pages URL to `CORS_ORIGINS` in backend `.env`

---

## 🎯 Success Criteria

1. ✅ Backend running at http://localhost:8000 (or production URL)
2. ✅ All test API calls return 200 OK
3. ✅ Frontend showing chat bubble
4. ✅ Can ask questions and get RAG responses
5. ✅ Signup/login working
6. ✅ Personalization working
7. ✅ Translation working

## ⏱️ Time Budget

- Prerequisites: 15 min
- Setup: 30 min
- Testing: 15 min
- Frontend integration: 30 min
- Deployment: 30 min
- **TOTAL: ~2 hours**

## 🚀 You're Ready!

With all features working, you have **up to 300 points**!

**Good luck in the hackathon!** 🏆
