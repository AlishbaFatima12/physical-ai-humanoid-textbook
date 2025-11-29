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
        system_content = f"""You are an expert AI assistant for the Physical AI & Humanoid Robotics textbook by Syeda Alishba Fatima.

Use the following context from the textbook to answer the user's question:

{retrieved_context}

Provide clear, accurate answers based on the textbook content."""

        if user_background:
            system_content += f"""

Adapt your response for a user with this background:
- Software skill level: {user_background.get('software_background', 'intermediate')}
- Hardware experience: {user_background.get('hardware_background', 'hobby')}
- Robotics experience: {user_background.get('robotics_experience', 'some')}

Adjust technical depth and explanations accordingly."""

        messages = [{"role": "system", "content": system_content}]

        # Add conversation history
        if history:
            messages.extend([{"role": msg.role, "content": msg.content} for msg in history])

        # Add current query
        messages.append({"role": "user", "content": query})

        # 6. Get response from OpenAI
        response = self.openai_client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=messages,
            temperature=0.7,
            max_tokens=1000
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
