"""FastAPI application entry point.

Run the RAG Agent API server with:
    python server.py

Or with uvicorn directly:
    uvicorn api:app --reload --port 8000
"""

import os

import uvicorn
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


def main():
    """Start the FastAPI server."""
    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "8000"))
    reload = os.environ.get("RELOAD", "false").lower() == "true"

    print(f"Starting RAG Agent API server on {host}:{port}")
    print(f"API Docs: http://{host}:{port}/docs")
    print(f"Health: http://{host}:{port}/health")

    uvicorn.run(
        "api:app",
        host=host,
        port=port,
        reload=reload,
    )


if __name__ == "__main__":
    main()
