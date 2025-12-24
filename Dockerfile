# Use Python 3.11 slim image
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies including uv
RUN apt-get update && apt-get install -y \
    curl \
    && rm -rf /var/lib/apt/lists/* \
    && curl -LsSf https://astral.sh/uv/install.sh | sh

# Add uv to PATH (it installs to /root/.local/bin)
ENV PATH="/root/.local/bin:$PATH"

# Copy backend files
COPY backend/ /app/backend/

# Install Python dependencies using uv
WORKDIR /app/backend
RUN uv pip install --system -e .

# Expose port 7860 (HF Spaces default)
EXPOSE 7860

# Run the FastAPI app
CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "7860"]
