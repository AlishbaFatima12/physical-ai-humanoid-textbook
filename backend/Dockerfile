# Use Python 3.11 slim image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Copy requirements from backend directory
COPY ./requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy entire backend application
COPY . .

# Expose port (Railway will use $PORT env var)
EXPOSE 8000

# Start command - Railway will inject PORT env variable
CMD uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8000}
