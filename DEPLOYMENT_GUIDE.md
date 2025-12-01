# 🚀 Physical AI Textbook - Complete Setup Guide

## ⚙️ Quick Start

### Terminal 1: Backend
```bash
cd backend
python -m uvicorn app.main:app --reload --port 8000
```

### Terminal 2: Frontend  
```bash
cd docs
npm start
```

### Open Browser:
```
http://localhost:3000/physical-ai-humanoid-textbook/
```

## ✅ What Works:
- **Smart Landing Page** - Robotic theme
- **RAG Chatbot** - 150 token responses
- **Text Popup** - Explain, Summarize, Translate (Urdu)
- **Personalize** - Adapt to skill level
- **Smart Backend** - Auto-detects local/production

## 🔧 Backend Already Has:
- ✅ CORS configured
- ✅ max_tokens & temperature params
- ✅ All endpoints working

## 🧪 Test Features:
1. Click "Start Reading →"
2. Select text → popup appears
3. Click chatbot (💬)
4. Click Personalize/Translate buttons

## 📦 Deploy:
```bash
cd docs
npm run build
GIT_USER=AlishbaFatima12 npm run deploy
```
