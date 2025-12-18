---
id: "0004"
title: "Spec-4 Pydantic Fix and Database Lifespan"
stage: green
date: 2025-12-17
surface: agent
model: claude-opus-4-5
feature: 007-spec04-chatkit-frontend
branch: 007-spec04-chatkit-frontend
user: system
command: /sp.implement
labels: ["pydantic", "fix", "database", "lifespan", "litellm", "gemini", "spec-4"]
links:
  spec: specs/007-spec04-chatkit-frontend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - backend/pydantic_patch.py
  - backend/api.py
  - backend/agent.py
tests:
  - Health endpoint: PASS
  - CORS verification: PASS
  - Pydantic validation: PASS (no more InputTokensDetails errors)
  - Chat endpoint: BLOCKED (Gemini rate limit)
---

## Prompt

/sp.implement use spec4 verifier agent. I have just changed the api key in env variable and model from gemini-2.5-flash to gemini-2.0-flash. Restart the fast api server and try again

## Response snapshot

Fixed the Pydantic validation error that was blocking chat functionality:

**Problem**: LiteLLM returns `None` for `cached_tokens` when using Gemini, but OpenAI SDK's `InputTokensDetails` Pydantic model requires an integer, causing validation errors.

**Solution**: Created a dedicated `pydantic_patch.py` module that modifies the Pydantic model fields BEFORE any OpenAI SDK imports:
1. Created `backend/pydantic_patch.py` with model field modifications and `model_rebuild(force=True)`
2. Updated `backend/api.py` to import the patch first
3. Updated `backend/agent.py` to import the patch first
4. Added FastAPI lifespan event for database pool initialization

**Verification Results** (via spec4-e2e-verifier agent):
- Health endpoint: PASS - All services healthy
- CORS configuration: PASS - Proper headers returned
- Pydantic validation: PASS - No more `InputTokensDetails.cached_tokens` errors
- Selected-text-only enforcement: PASS (code review)
- Chat endpoint: BLOCKED - Gemini API rate limit (free tier quota exceeded)
- Database initialization: Fixed - Now called via lifespan event

## Outcome

- ✅ Impact: Pydantic validation error fixed, database lifespan event added
- 🧪 Tests: Health PASS, CORS PASS, Chat BLOCKED (rate limit - not a code issue)
- 📁 Files: 3 files modified/created (pydantic_patch.py, api.py, agent.py)
- 🔁 Next prompts: Wait for Gemini rate limit reset or upgrade API tier to fully verify chat
- 🧠 Reflection: Python import order was critical - patch must happen before any OpenAI SDK imports

## Evaluation notes (flywheel)

- Failure modes observed: Initial patch in api.py didn't work due to import order; needed separate module imported first
- Graders run and results (PASS/FAIL): Health PASS, CORS PASS, Pydantic PASS, Chat BLOCKED
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test chat endpoint after Gemini rate limit resets
