# 🚀 Render Deployment Guide

## Prerequisites
- ✅ Render account (free): https://render.com/
- ✅ GitHub repository pushed
- ✅ All API keys ready (OpenAI, Qdrant, Neon PostgreSQL)

---

## 📋 Quick Deployment Steps

### Option A: One-Click Deploy (Recommended)

1. **Push to GitHub** (if not already done):
   ```bash
   git add backend/render.yaml
   git commit -m "Add Render deployment config"
   git push origin main
   ```

2. **Deploy to Render**:
   - Go to: https://render.com/
   - Click **"New +"** → **"Blueprint"**
   - Connect your GitHub repository: `AlishbaFatima12/physical-ai-humanoid-textbook`
   - Render will detect `render.yaml` automatically
   - Click **"Apply"**

3. **Set Environment Variables** (in Render Dashboard):
   Navigate to your service → Environment → Add:

   ```
   OPENAI_API_KEY=sk-proj-TpDr... (your actual key)
   DATABASE_URL=postgresql://neondb_owner:npg_... (your Neon URL)
   QDRANT_URL=https://5c7c3b7f-586a-4bab... (your Qdrant URL)
   QDRANT_API_KEY=eyJhbGci... (your Qdrant key)
   CORS_ORIGINS=http://localhost:3000,https://alishbafatima12.github.io
   ```

4. **Wait for Build** (~5-10 minutes)
   - Watch the logs for "Build successful"
   - Your backend will be live at: `https://physical-ai-humanoid-textbook-backend.onrender.com`

---

### Option B: Manual Deployment

1. **Create New Web Service**:
   - Go to https://dashboard.render.com/
   - Click **"New +"** → **"Web Service"**
   - Connect GitHub repository

2. **Configure Service**:
   ```
   Name: physical-ai-humanoid-textbook-backend
   Region: Oregon (US West)
   Branch: main
   Root Directory: backend
   Runtime: Python 3
   Build Command: pip install -r requirements.txt
   Start Command: uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```

3. **Set Environment Variables** (same as Option A step 3)

4. **Deploy** → Click "Create Web Service"

---

## ✅ Verification

After deployment, test your backend:

```bash
# Health check
curl https://physical-ai-humanoid-textbook-backend.onrender.com/health

# Should return: {"status":"healthy"}
```

---

## 🔧 Post-Deployment

### 1. Run Data Ingestion (Important!)

After first deployment, you need to ingest the textbook content:

```bash
curl -X POST https://physical-ai-humanoid-textbook-backend.onrender.com/ingest
```

This will:
- Read all textbook chapters from your repo
- Create embeddings using OpenAI
- Store in Qdrant vector database
- Takes ~2-3 minutes

### 2. Update Frontend

The frontend already checks for production backend automatically via `docs/src/utils/api.js`:
- Local: tries `http://localhost:8000` first
- Production: falls back to Render URL

No changes needed! Just redeploy frontend:

```bash
cd docs
npm run build
GIT_USER=AlishbaFatima12 npm run deploy
```

---

## 🐛 Troubleshooting

### Build Fails
- **Error**: `ModuleNotFoundError`
  - **Fix**: Check `requirements.txt` has all dependencies
  - Missing package? Add it to `requirements.txt`

### 502 Bad Gateway
- **Cause**: App crashed or port binding issue
- **Fix**: Check Render logs for errors
- Make sure start command uses `$PORT` (not hardcoded 8000)

### CORS Errors on Frontend
- **Cause**: `CORS_ORIGINS` not set correctly
- **Fix**: Add both localhost AND GitHub Pages URLs:
  ```
  CORS_ORIGINS=http://localhost:3000,https://alishbafatima12.github.io
  ```

### Database Connection Issues
- **Cause**: Wrong `DATABASE_URL` format
- **Fix**: Neon URL should start with `postgresql://`
- Get fresh connection string from Neon dashboard

### Qdrant Connection Failed
- **Cause**: Wrong API key or cluster URL
- **Fix**: Verify credentials in Qdrant Cloud dashboard
- URL format: `https://xxxxx.qdrant.io`

---

## 💰 Render Free Tier Limits

- ✅ 750 hours/month (enough for 24/7)
- ✅ Automatically sleeps after 15 min inactivity
- ✅ Wakes up on first request (~30 sec cold start)
- ❌ Monthly build minutes limit (500 min)

**Tip**: To avoid cold starts, use a service like UptimeRobot to ping your backend every 10 minutes.

---

## 📊 Monitoring

View logs in Render Dashboard:
1. Go to your service
2. Click "Logs" tab
3. See real-time output

---

## 🔐 Security Checklist

Before deploying:
- ✅ Never commit `.env` file
- ✅ Use Render's environment variables (encrypted)
- ✅ Restrict CORS to your domains only
- ✅ Keep API keys secret

---

## 🎯 Next Steps

After successful deployment:

1. ✅ Test health endpoint
2. ✅ Run `/ingest` to load textbook content
3. ✅ Test chat endpoint with sample query
4. ✅ Redeploy frontend to use production backend
5. ✅ Test all features on live site

---

## 📞 Need Help?

- Render Docs: https://render.com/docs
- Render Community: https://community.render.com/
- Check logs for specific errors

---

**Last Updated**: December 1, 2025
