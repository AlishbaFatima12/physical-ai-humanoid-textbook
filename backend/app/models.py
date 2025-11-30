"""
Database Models for Neon Postgres
"""

from sqlalchemy import Column, Integer, String, Text, DateTime, JSON, Boolean
from sqlalchemy.sql import func
from .database import Base

class User(Base):
    """User model for Better-Auth integration"""
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String, unique=True, index=True, nullable=False)
    name = Column(String, nullable=False)
    hashed_password = Column(String, nullable=False)

    # Background information for personalization
    software_background = Column(String)  # beginner, intermediate, advanced
    hardware_background = Column(String)  # none, hobby, professional
    programming_languages = Column(JSON)  # List of languages
    robotics_experience = Column(String)  # none, some, extensive

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

class Conversation(Base):
    """Conversation history model"""
    __tablename__ = "conversations"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, nullable=True)  # Null for anonymous users
    session_id = Column(String, index=True, nullable=False)

    message = Column(Text, nullable=False)
    response = Column(Text, nullable=False)
    context = Column(Text)  # User-selected text
    sources = Column(JSON)  # Retrieved sources from RAG
    tokens_used = Column(Integer)

    created_at = Column(DateTime(timezone=True), server_default=func.now())

class UserPreference(Base):
    """User preferences for personalization"""
    __tablename__ = "user_preferences"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, unique=True, nullable=False)

    # Personalization settings
    content_complexity = Column(String, default="standard")  # simplified, standard, advanced
    show_code_examples = Column(Boolean, default=True)
    show_math_details = Column(Boolean, default=True)
    preferred_language = Column(String, default="en")  # en, ur

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

class PersonalizedChapter(Base):
    """Store personalized chapter content for users"""
    __tablename__ = "personalized_chapters"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, nullable=False)
    chapter_path = Column(String, nullable=False)  # e.g., "chapter-1-introduction"

    # Content states
    original_content = Column(Text, nullable=False)
    personalized_content = Column(Text, nullable=True)
    translated_content = Column(Text, nullable=True)

    # Flags
    is_personalized = Column(Boolean, default=False)
    is_translated = Column(Boolean, default=False)

    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())
