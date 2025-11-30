# RAG Chatbot Setup Guide

## Prerequisites

Before running the chatbot, you need to set up accounts for:
1. OpenAI (for GPT-4)
2. Qdrant Cloud (for vector database)

---

## Step 1: OpenAI Setup

### Create OpenAI Account & Get API Key

1. Go to [https://platform.openai.com/signup](https://platform.openai.com/signup)
2. Sign up with your email
3. Verify your email address
4. Once logged in, go to [https://platform.openai.com/api-keys](https://platform.openai.com/api-keys)
5. Click "Create new secret key"
6. **Copy the API key** (you won't be able to see it again!)
7. Save it somewhere safe

**Cost:** OpenAI charges per token used. For testing, $5-10 credit should be sufficient.

---

## Step 2: Qdrant Cloud Setup

### Create Qdrant Cloud Account

1. Go to [https://cloud.qdrant.io/](https://cloud.qdrant.io/)
2. Click "Get Started" or "Sign Up"
3. Sign up with your email or GitHub account
4. Verify your email

### Create a Free Cluster

1. Once logged in, click "Create Cluster"
2. Choose the **Free Tier** (1GB storage, perfect for this project)
3. Select a region (choose closest to you)
4. Give it a name (e.g., "textbook-rag")
5. Click "Create"

### Get API Key and URL

1. Once the cluster is created, click on it
2. Go to "API Keys" tab
3. Click "Create API Key"
4. **Copy the API key**
5. Also copy the **Cluster URL** (looks like: `https://xxx-yyy.aws.cloud.qdrant.io:6333`)

**Cost:** Free tier is sufficient for this project!

---

## Step 3: Environment Configuration

1. In the `backend` directory, create a `.env` file:

```bash
cd backend
cp .env.example .env
```

2. Edit `.env` and add your credentials:

```env
# OpenAI API
OPENAI_API_KEY=sk-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Qdrant Cloud
QDRANT_URL=https://your-cluster.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key_here

# App Settings
ENVIRONMENT=development
CORS_ORIGINS=http://localhost:3000,https://alishbafatima12.github.io
```

---

## Step 4: Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

---

## Step 5: Run the Backend

```bash
# From the backend directory
python -m uvicorn app.main:app --reload --port 8000
```

You should see:
```
INFO:     Uvicorn running on http://127.0.0.1:8000
```

---

## Step 6: Ingest Textbook Content

Open a new terminal and run:

```bash
curl -X POST http://localhost:8000/ingest
```

This will load all your textbook chapters into Qdrant.

---

## Step 7: Test the Chatbot

### Test via API:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

### Test via Website:

1. Make sure the Docusaurus dev server is running:
   ```bash
   cd docs
   npm start
   ```

2. Open [http://localhost:3000](http://localhost:3000)

3. Look for the chat bubble in the bottom-right corner

4. Click it and ask questions!

---

## Troubleshooting

### Error: "Connection refused" or "Cannot connect to backend"

**Solution:** Make sure the FastAPI backend is running on port 8000

```bash
cd backend
python -m uvicorn app.main:app --reload --port 8000
```

### Error: "Invalid API key"

**Solution:** Check your `.env` file and make sure the OpenAI and Qdrant API keys are correct

### Error: "Collection not found"

**Solution:** Run the ingestion step:

```bash
curl -X POST http://localhost:8000/ingest
```

### Error: "Module not found"

**Solution:** Install dependencies:

```bash
cd backend
pip install -r requirements.txt
```

---

## Cost Estimate

### OpenAI Costs:
- GPT-4 Turbo: ~$0.01 per query
- For 100 test queries: ~$1
- Recommended starting credit: $10

### Qdrant Costs:
- Free tier: $0
- 1GB storage is enough for the entire textbook

### Total estimated cost for hackathon: **$5-15**

---

## Next Steps

Once everything is running:

1. ✅ Backend running on http://localhost:8000
2. ✅ Textbook content ingested into Qdrant
3. ✅ Frontend running on http://localhost:3000
4. ✅ Chat bubble appears on website
5. ✅ Can ask questions and get RAG responses

**You're ready to submit! 🎉**

---

## Optional: Deploy to Production

### Deploy Backend (Vercel/Railway/Render):
1. Push backend code to GitHub
2. Connect to Vercel/Railway/Render
3. Add environment variables
4. Deploy!

### Update Frontend:
Edit `docs/src/components/Chatbot/index.js`:
```javascript
const response = await fetch('https://your-backend-url.com/chat', {
  // ... rest of code
});
```

Then redeploy to GitHub Pages.
