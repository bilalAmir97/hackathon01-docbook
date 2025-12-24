---
id: 0001
title: Test ROS 2 Query RAG Pipeline
stage: general
date: 2025-12-24
surface: agent
model: claude-opus-4-5
feature: none
branch: 001-futuristic-homepage-ui
user: Bilal
command: test RAG pipeline
labels: ["testing", "rag", "retrieval", "debugging"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/api.py
  - backend/agent.py
  - backend/retrieval.py
  - test_rag.json
  - test_rag_nonstream.json
tests:
  - Direct Qdrant retrieval test (passed)
  - Streaming endpoint test (passed)
  - Non-streaming endpoint test (passed)
  - Tool calling verification (failed)
---

## Prompt

test a query what is ros 2and make sure rag retrieval pipelines works correctly

## Response snapshot

Tested RAG pipeline with query "What is ROS 2?" via both streaming and non-streaming endpoints.

### ✅ Working Components:
1. Backend server starts successfully on port 8000
2. Qdrant retrieval working correctly - retrieves 3 relevant chunks
3. Response streaming via SSE functional
4. Answer quality is good with detailed ROS 2 information

### ❌ Critical Issue:
Agent NOT calling `search_documentation` tool, resulting in:
- chunks_retrieved: 0
- sources: [] (empty)
- Agent answers from internal knowledge instead of retrieved docs

### Evidence:
- Direct Qdrant test: Successfully retrieves chunks with title "Introduction to ROS 2 Concepts"
- Both API endpoints return empty sources array
- Tool is properly registered with @function_tool decorator
- System prompt instructs "ONLY use information from search_documentation tool"

### Root Cause:
Gemini model (gemini/gemini-2.5-flash) bypassing tool calls despite explicit instructions, preferring to answer from internal knowledge.

## Outcome

- ✅ Impact: Identified critical bug - RAG pipeline infrastructure works but agent doesn't use retrieval tool
- 🧪 Tests: Verified Qdrant retrieval, API endpoints, streaming functionality
- 📁 Files: Tested backend/api.py, backend/agent.py, backend/retrieval.py
- 🔁 Next prompts: Fix agent tool-calling behavior, strengthen system prompts, consider tool_choice parameter
- 🧠 Reflection: Backend infrastructure is solid; issue is purely in agent tool-calling compliance

## Evaluation notes (flywheel)

- Failure modes observed: LLM bypassing tools when confident about topic from training data
- Graders run and results (PASS/FAIL): Qdrant retrieval PASS, Tool calling FAIL, API structure PASS
- Prompt variant (if applicable): GENERAL_SYSTEM_PROMPT with "CRITICAL RULES" for tool usage
- Next experiment (smallest change to try): Add explicit tool_choice='required' or pre-call validation
