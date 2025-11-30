"""
RAG Service using OpenAI and Qdrant
"""

import os
from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.http import models
from sentence_transformers import SentenceTransformer

class RAGService:
    def __init__(self):
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.collection_name = "textbook_content"
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')

    async def get_response(self, query: str, context: str = None, history: list = None, user_background: dict = None):
        """
        Get RAG response for user query with optional personalization
        """
        # 1. Get query embedding
        query_vector = self.embedding_model.encode(query).tolist()

        # 2. Search similar content in Qdrant
        search_results = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_vector,
            limit=5
        ).points

        # 3. Build context from retrieved documents
        retrieved_context = "\n\n".join([
            result.payload.get("content", "")
            for result in search_results
        ])

        # 4. Add user-selected text if provided
        if context:
            retrieved_context = f"User Selected Text:\n{context}\n\nRelated Content:\n{retrieved_context}"

        # 5. Build messages for OpenAI with personalization
        system_content = f"""Answer in EXACTLY 3-5 bullet points. Use context: {retrieved_context}

RULES:
- ALWAYS return 3-5 points ONLY
- NO paragraphs
- Each point = 1 short sentence
- Be direct and clear"""

        if user_background:
            system_content += f""" Adapt for: {user_background.get('software_background', 'intermediate')} level."""

        messages = [{"role": "system", "content": system_content}]

        # Add last 2 messages from history only
        if history and len(history) > 0:
            messages.extend([{"role": msg.role, "content": msg.content} for msg in history[-2:]])

        messages.append({"role": "user", "content": query})

        # 6. Get response from OpenAI - FAST MODEL
        response = self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=messages,
            temperature=0.3,
            max_tokens=100
        )

        # 7. Format sources
        sources = [
            {
                "chapter": result.payload.get("chapter", "Unknown"),
                "title": result.payload.get("title", "Untitled"),
                "score": result.score
            }
            for result in search_results
        ]

        return {
            "response": response.choices[0].message.content,
            "sources": sources,
            "tokens_used": response.usage.total_tokens
        }
