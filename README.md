---
title: RAG Chatbot - Physical AI & Robotics
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
---

# RAG Chatbot for Physical AI & Humanoid Robotics

A Retrieval-Augmented Generation (RAG) chatbot that answers questions about Physical AI and Humanoid Robotics using documentation from a Docusaurus book.

## Features

- 🔍 Semantic search with Qdrant vector database
- 🤖 Powered by Gemini 2.5 Flash via OpenAI Agents SDK
- 📚 Retrieves 10 relevant chunks per query for comprehensive answers
- 📝 Provides source citations with clickable links
- 💬 Supports both streaming and non-streaming responses
- 🎯 Selected text mode for answering based on user-selected content
- 💾 Chat history persistence with Neon PostgreSQL

## Tech Stack

- **Backend**: FastAPI (Python 3.11)
- **LLM**: Gemini 2.5 Flash
- **Vector DB**: Qdrant Cloud
- **Embeddings**: Cohere embed-english-v3.0
- **Database**: Neon PostgreSQL
- **Deployment**: Docker

## Environment Variables Required

Set these in your Hugging Face Space settings:

```bash
GEMINI_API_KEY=your_gemini_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_CONNECTION_STRING=your_neon_connection_string
```

## API Endpoints

- `POST /chat` - Non-streaming chat endpoint
- `POST /chat/stream` - Server-Sent Events streaming endpoint
- `GET /health` - Health check endpoint
- `GET /conversations/{session_id}` - Get chat history for a session

## Recent Updates

- ✅ Fixed RAG retrieval pipeline (chunks now pre-retrieved and injected into prompt)
- ✅ Increased retrieval from 5 to 10 chunks for richer context
- ✅ Improved system prompts for better citation behavior
- ✅ Full source citations with URLs and relevance scores

## Usage

Send a POST request to `/chat/stream`:

```json
{
  "query": "What is ROS 2?",
  "mode": "general",
  "session_id": "your-session-uuid"
}
```

Response includes:
- Streaming text chunks with inline citations
- Source array with full metadata
- Query metadata (chunks retrieved, query time, model used)
