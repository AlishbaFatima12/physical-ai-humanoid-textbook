"""
Ingestion Service to load textbook content into Qdrant
"""

import os
import glob
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
import re

class IngestionService:
    def __init__(self):
        self.qdrant_client = QdrantClient(
            url=os.getenv("QDRANT_URL"),
            api_key=os.getenv("QDRANT_API_KEY")
        )
        self.openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.collection_name = "textbook_content"

    async def ingest_textbook_content(self):
        """
        Ingest all textbook markdown files into Qdrant
        """
        # Get path to docs directory
        docs_path = Path(__file__).parent.parent.parent.parent / "docs" / "docs"

        # Find all markdown files
        md_files = list(docs_path.glob("*.md"))

        # Create/recreate collection
        try:
            self.qdrant_client.delete_collection(self.collection_name)
        except:
            pass

        self.qdrant_client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=1536,  # text-embedding-3-small embedding size
                distance=models.Distance.COSINE
            )
        )

        # Process each file
        points = []
        point_id = 0

        for md_file in md_files:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract chapter info
            filename = md_file.name
            chapter_match = re.search(r'(\d+)-chapter-(\d+)', filename)
            chapter_num = chapter_match.group(2) if chapter_match else "intro"

            # Split into chunks (simple by paragraphs)
            chunks = self._split_into_chunks(content)

            # Filter out very small chunks
            valid_chunks = [chunk for chunk in chunks if len(chunk.strip()) >= 50]

            # Batch create embeddings using OpenAI API (memory efficient)
            if valid_chunks:
                embeddings_response = self.openai_client.embeddings.create(
                    model="text-embedding-3-small",
                    input=valid_chunks
                )

                for i, chunk in enumerate(valid_chunks):
                    embedding = embeddings_response.data[i].embedding

                    points.append(
                        models.PointStruct(
                            id=point_id,
                            vector=embedding,
                            payload={
                                "content": chunk,
                                "chapter": chapter_num,
                                "title": self._extract_title(chunk),
                                "file": filename
                            }
                        )
                    )
                    point_id += 1

        # Upload to Qdrant
        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        return {
            "files_processed": len(md_files),
            "chunks_created": len(points)
        }

    def _split_into_chunks(self, content: str, chunk_size: int = 1000) -> list:
        """Split content into manageable chunks"""
        # Split by double newlines (paragraphs)
        paragraphs = content.split('\n\n')

        chunks = []
        current_chunk = ""

        for para in paragraphs:
            if len(current_chunk) + len(para) < chunk_size:
                current_chunk += para + "\n\n"
            else:
                if current_chunk:
                    chunks.append(current_chunk.strip())
                current_chunk = para + "\n\n"

        if current_chunk:
            chunks.append(current_chunk.strip())

        return chunks

    def _extract_title(self, chunk: str) -> str:
        """Extract title from chunk"""
        lines = chunk.split('\n')
        for line in lines:
            if line.startswith('#'):
                return line.replace('#', '').strip()
        return "Untitled Section"
