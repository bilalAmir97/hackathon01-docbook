---
id: "0002"
title: "Clarify Spec-3 Gemini and Postgres Integration"
stage: spec
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 006-rag-agent-api
branch: 006-rag-agent-api
user: developer
command: /sp.clarify
labels: ["clarification", "gemini", "neon-postgres", "openai-agents-sdk", "spec"]
links:
  spec: specs/006-rag-agent-api/spec.md
  ticket: null
  adr: history/adr/0006-spec2-spec3-integration-pattern.md
  pr: null
files:
  - specs/006-rag-agent-api/spec.md
tests:
  - N/A (specification phase)
---

## Prompt

Use context7 mcp server to fetch the OpenAI Agents SDK documentation. Analyze the @OpenAIAgents_SampleCode.md and @specify_prompt.md files in order to take the context from them. Make sure to just make one file for agent named agent.py for agent persisting retrieval function in order to respone user queries.
Also, Make sure to use OpenAI Agents SDK (with Gemini API via OpenAI-compatible endpoint),

## Response snapshot

Ran clarification workflow with 5 questions based on analysis of:
- OpenAI Agents SDK documentation (via Context7 MCP)
- User's `specify_prompt.md` requirements
- User's `OpenAIAgents_SampleCode.md` code pattern
- Existing `backend/retrieval.py` from Spec-2

### Clarifications Resolved:

| # | Question | Answer |
|---|----------|--------|
| 1 | LLM Provider | Gemini API via OpenAI-compatible endpoint (`https://generativelanguage.googleapis.com/v1beta/openai/`) |
| 2 | Conversation History | Yes, store in Neon Postgres with session ID, enable multi-turn chat |
| 3 | File Structure | Hybrid - `agent.py` imports from existing `retrieval.py`, separate `api.py` for FastAPI |
| 4 | Response Time (p95) | 3 seconds (updated from 5 seconds per specify_prompt.md) |
| 5 | SDK Tracing | Disabled (`tracing_disabled=True`) since using Gemini backend |

### Spec Updates Made:
- Changed `OPENAI_API_KEY` to `GEMINI_API_KEY` in environment variables
- Added `GEMINI_MODEL`, `GEMINI_BASE_URL`, `DATABASE_URL` env vars
- Added FR-013, FR-014, FR-015 for conversation history
- Added `session_id` to ChatRequest schema
- Added Conversation Storage Schema (Neon Postgres)
- Added Implementation File Structure section
- Added Tracing Configuration section
- Updated SC-001 from 5s to 3s, SC-002 from 1.5s to 1s
- Added Neon Postgres to Dependencies

## Outcome

- ✅ Impact: High - Aligned spec with user's actual requirements (Gemini, Postgres, file structure)
- 🧪 Tests: N/A (specification phase)
- 📁 Files: 1 file updated (spec.md with 5 clarifications integrated)
- 🔁 Next prompts: /sp.plan to create architecture, /sp.tasks to generate implementation tasks
- 🧠 Reflection: User's input files contained critical requirements not in original spec; clarification caught 5 significant gaps

## Evaluation notes (flywheel)

- Failure modes observed: None - all questions answered clearly
- Graders run and results (PASS/FAIL): All clarifications integrated successfully - PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider auto-detecting discrepancies between spec and reference docs earlier
