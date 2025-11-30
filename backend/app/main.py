"""
FastAPI RAG Chatbot for Physical AI Textbook
Integrates OpenAI, Qdrant, Neon Postgres, Better-Auth, Personalization, and Translation
"""

from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, EmailStr
from typing import List, Optional
from sqlalchemy.orm import Session
import os
import uuid
from dotenv import load_dotenv

from . import models, auth
from .database import get_db, init_db

# Load environment variables
load_dotenv()

app = FastAPI(
    title="Physical AI Textbook RAG Chatbot",
    description="Full-featured chatbot with RAG, Auth, Personalization & Translation",
    version="2.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("CORS_ORIGINS", "http://localhost:3000,https://alishbafatima12.github.io").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize database on startup
@app.on_event("startup")
async def startup_event():
    init_db()

# ========================================
# REQUEST/RESPONSE MODELS
# ========================================

class ChatMessage(BaseModel):
    role: str
    content: str

class ChatRequest(BaseModel):
    message: str
    context: Optional[str] = None
    conversation_history: Optional[List[ChatMessage]] = []
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]
    tokens_used: int

class SignupRequest(BaseModel):
    name: str
    email: EmailStr
    password: str
    # Background questions for personalization
    software_background: str  # beginner, intermediate, advanced
    hardware_background: str  # none, hobby, professional
    programming_languages: List[str]
    robotics_experience: str  # none, some, extensive

class LoginRequest(BaseModel):
    email: EmailStr
    password: str

class AuthResponse(BaseModel):
    access_token: str
    token_type: str
    user: dict

class PersonalizeRequest(BaseModel):
    content: str
    user_background: dict

class TranslateRequest(BaseModel):
    content: str
    target_language: str = "ur"  # Urdu

class SaveChapterRequest(BaseModel):
    chapter_path: str
    original_content: str
    personalized_content: Optional[str] = None
    translated_content: Optional[str] = None
    is_personalized: bool = False
    is_translated: bool = False

# ========================================
# CORE ENDPOINTS
# ========================================

@app.get("/")
async def root():
    return {
        "message": "Physical AI Textbook RAG Chatbot API",
        "version": "2.0.0",
        "features": ["RAG", "Auth", "Personalization", "Translation"],
        "endpoints": {
            "/chat": "POST - Chat with RAG",
            "/signup": "POST - User signup with background questions",
            "/login": "POST - User login",
            "/personalize": "POST - Personalize content",
            "/translate": "POST - Translate to Urdu",
            "/ingest": "POST - Ingest textbook content"
        }
    }

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "rag-chatbot-full"}

# ========================================
# RAG CHAT ENDPOINT
# ========================================

@app.post("/chat", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    current_user: Optional[models.User] = Depends(auth.get_current_user),
    db: Session = Depends(get_db)
):
    """Main chat endpoint with RAG capabilities"""
    try:
        from .services.rag_service import RAGService

        rag_service = RAGService()

        # Get user background for personalization
        user_background = None
        if current_user:
            user_background = {
                "software_background": current_user.software_background,
                "hardware_background": current_user.hardware_background,
                "robotics_experience": current_user.robotics_experience
            }

        # Get response using RAG
        response = await rag_service.get_response(
            query=request.message,
            context=request.context,
            history=request.conversation_history,
            user_background=user_background
        )

        # Save conversation to database
        session_id = request.session_id or str(uuid.uuid4())
        conversation = models.Conversation(
            user_id=current_user.id if current_user else None,
            session_id=session_id,
            message=request.message,
            response=response["response"],
            context=request.context,
            sources=response["sources"],
            tokens_used=response["tokens_used"]
        )
        db.add(conversation)
        db.commit()

        return response

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ========================================
# AUTH ENDPOINTS (BONUS +50 pts)
# ========================================

@app.post("/signup", response_model=AuthResponse)
async def signup(request: SignupRequest, db: Session = Depends(get_db)):
    """Signup with background questions for personalization"""
    # Check if user already exists
    existing_user = db.query(models.User).filter(models.User.email == request.email).first()
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")

    # Create new user
    hashed_password = auth.get_password_hash(request.password)
    new_user = models.User(
        name=request.name,
        email=request.email,
        hashed_password=hashed_password,
        software_background=request.software_background,
        hardware_background=request.hardware_background,
        programming_languages=request.programming_languages,
        robotics_experience=request.robotics_experience
    )

    db.add(new_user)
    db.commit()
    db.refresh(new_user)

    # Create default preferences
    preferences = models.UserPreference(
        user_id=new_user.id,
        content_complexity="standard",
        show_code_examples=True,
        show_math_details=True,
        preferred_language="en"
    )
    db.add(preferences)
    db.commit()

    # Generate token
    access_token = auth.create_access_token(data={"sub": new_user.id})

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": {
            "id": new_user.id,
            "name": new_user.name,
            "email": new_user.email,
            "software_background": new_user.software_background,
            "hardware_background": new_user.hardware_background,
            "robotics_experience": new_user.robotics_experience
        }
    }

@app.post("/login", response_model=AuthResponse)
async def login(request: LoginRequest, db: Session = Depends(get_db)):
    """User login"""
    user = db.query(models.User).filter(models.User.email == request.email).first()

    if not user or not auth.verify_password(request.password, user.hashed_password):
        raise HTTPException(status_code=401, detail="Invalid email or password")

    # Generate token
    access_token = auth.create_access_token(data={"sub": user.id})

    return {
        "access_token": access_token,
        "token_type": "bearer",
        "user": {
            "id": user.id,
            "name": user.name,
            "email": user.email,
            "software_background": user.software_background,
            "hardware_background": user.hardware_background,
            "robotics_experience": user.robotics_experience
        }
    }

@app.get("/me")
async def get_current_user_info(
    current_user: models.User = Depends(auth.get_current_active_user)
):
    """Get current user info"""
    return {
        "id": current_user.id,
        "name": current_user.name,
        "email": current_user.email,
        "software_background": current_user.software_background,
        "hardware_background": current_user.hardware_background,
        "robotics_experience": current_user.robotics_experience
    }

# ========================================
# PERSONALIZATION ENDPOINT (BONUS +50 pts)
# ========================================

@app.post("/personalize")
async def personalize_content(
    request: PersonalizeRequest,
    current_user: Optional[models.User] = Depends(auth.get_current_user)
):
    """Personalize content based on user background"""
    try:
        from openai import OpenAI

        client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Build personalization prompt
        user_level = request.user_background.get("software_background", "intermediate")
        hardware_exp = request.user_background.get("hardware_background", "hobby")
        robotics_exp = request.user_background.get("robotics_experience", "some")

        system_prompt = f"""You are an expert content adapter for a Physical AI & Humanoid Robotics textbook.

Adapt the following content for a user with this background:
- Software skill level: {user_level}
- Hardware experience: {hardware_exp}
- Robotics experience: {robotics_exp}

Adjustments to make:
1. If beginner: Simplify technical terms, add more explanations, include analogies
2. If advanced: Add more technical depth, reference research papers, include advanced topics
3. Adjust code complexity and detail level accordingly
4. Maintain the core teaching objectives

Return ONLY the adapted content, preserving markdown formatting."""

        response = client.chat.completions.create(
            model="gpt-4-turbo-preview",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.content}
            ],
            temperature=0.7,
            max_tokens=2000
        )

        return {
            "personalized_content": response.choices[0].message.content,
            "user_level": user_level
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ========================================
# TRANSLATION ENDPOINT (BONUS +50 pts)
# ========================================

@app.post("/translate")
async def translate_content(request: TranslateRequest):
    """Translate content to Urdu - INSTANT with gpt-4o-mini"""
    try:
        from openai import OpenAI

        client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        system_prompt = """Translate to Urdu in bullet points (3-5 points max). Keep technical terms in English."""

        response = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.content[:2000]}
            ],
            temperature=0.3,
            max_tokens=150,
            stream=False
        )

        return {
            "translated_content": response.choices[0].message.content,
            "target_language": request.target_language
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ========================================
# CONTENT INGESTION ENDPOINT
# ========================================

@app.post("/ingest")
async def ingest_content():
    """Ingest textbook content into vector database"""
    try:
        from .services.ingestion_service import IngestionService

        ingestion_service = IngestionService()
        result = await ingestion_service.ingest_textbook_content()

        return {
            "status": "success",
            "message": "Content ingested successfully",
            **result
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# ========================================
# CHAPTER PERSISTENCE ENDPOINTS
# ========================================

@app.post("/chapters/save")
async def save_chapter(
    request: SaveChapterRequest,
    current_user: models.User = Depends(auth.get_current_active_user),
    db: Session = Depends(get_db)
):
    """Save personalized/translated chapter for user"""
    # Check if chapter already exists for user
    existing = db.query(models.PersonalizedChapter).filter(
        models.PersonalizedChapter.user_id == current_user.id,
        models.PersonalizedChapter.chapter_path == request.chapter_path
    ).first()

    if existing:
        # Update existing
        existing.original_content = request.original_content
        existing.personalized_content = request.personalized_content
        existing.translated_content = request.translated_content
        existing.is_personalized = request.is_personalized
        existing.is_translated = request.is_translated
    else:
        # Create new
        chapter = models.PersonalizedChapter(
            user_id=current_user.id,
            chapter_path=request.chapter_path,
            original_content=request.original_content,
            personalized_content=request.personalized_content,
            translated_content=request.translated_content,
            is_personalized=request.is_personalized,
            is_translated=request.is_translated
        )
        db.add(chapter)

    db.commit()

    return {"status": "success", "message": "Chapter saved"}

@app.get("/chapters/{chapter_path}")
async def get_chapter(
    chapter_path: str,
    current_user: models.User = Depends(auth.get_current_active_user),
    db: Session = Depends(get_db)
):
    """Get saved chapter for user"""
    chapter = db.query(models.PersonalizedChapter).filter(
        models.PersonalizedChapter.user_id == current_user.id,
        models.PersonalizedChapter.chapter_path == chapter_path
    ).first()

    if not chapter:
        return {"status": "not_found", "chapter": None}

    return {
        "status": "success",
        "chapter": {
            "original_content": chapter.original_content,
            "personalized_content": chapter.personalized_content,
            "translated_content": chapter.translated_content,
            "is_personalized": chapter.is_personalized,
            "is_translated": chapter.is_translated
        }
    }

@app.delete("/chapters/{chapter_path}")
async def reset_chapter(
    chapter_path: str,
    current_user: models.User = Depends(auth.get_current_active_user),
    db: Session = Depends(get_db)
):
    """Reset chapter to original (delete personalization)"""
    chapter = db.query(models.PersonalizedChapter).filter(
        models.PersonalizedChapter.user_id == current_user.id,
        models.PersonalizedChapter.chapter_path == chapter_path
    ).first()

    if chapter:
        db.delete(chapter)
        db.commit()

    return {"status": "success", "message": "Chapter reset to original"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True
    )
