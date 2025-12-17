---
id: 003
title: Plan ChatKit Frontend Integration
stage: plan
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 007-spec04-chatkit-frontend
branch: 007-spec04-chatkit-frontend
user: architect
command: /sp.plan
labels: ["planning", "architecture", "chatkit", "docusaurus", "fastapi", "cors", "sse"]
links:
  spec: specs/007-spec04-chatkit-frontend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/007-spec04-chatkit-frontend/plan.md
  - specs/007-spec04-chatkit-frontend/research.md
  - specs/007-spec04-chatkit-frontend/data-model.md
  - specs/007-spec04-chatkit-frontend/quickstart.md
  - specs/007-spec04-chatkit-frontend/contracts/api-spec.yaml
tests:
  - N/A (planning phase - testing checklist defined in plan.md)
---

## Prompt

/sp.plan

- Create: end-to-end architecture sketch for (Docusaurus book + embedded ChatKit UI ↔ FastAPI agent API ↔ Qdrant retrieval ↔ Neon Postgres via database.py), including selected-text-only request flow.
- Contract: define API endpoints + payload schema (message, session_id, selected_text, stream flag) and response schema (answer + citations + errors), plus env-based API base URL config for dev/prod.
- Decisions to document: widget placement (global vs per-page), streaming vs non-streaming, session_id storage (cookie vs localStorage), and CORS policy (strict allowlist for localhost + deployed book domain).
- Quality validation: enforce "selected-text-only" gating + safe fallbacks when retrieval is empty; define minimal logging (request id, session id) without leaking secrets.
- Testing strategy: E2E smoke checklist—UI renders, no CORS errors, chat completes, citations render, history persists/reloads, selected-text-only is enforced.

## Response snapshot

Created comprehensive technical plan for Spec-4: Docusaurus ChatKit Frontend + Integration.

**Key deliverables:**
1. **plan.md** - Full implementation plan with:
   - ASCII architecture diagrams (system overview, data flows)
   - Technical context and constitution check
   - Project structure (8 React components, 3 hooks, 2 backend modifications)
   - API contracts (existing + new endpoints)
   - Backend modifications (CORS hardening, /conversations endpoint)
   - 3 architectural decisions documented (SSE, localStorage, swizzled Root)
   - 18-item E2E testing checklist

2. **research.md** - Resolved decisions with rationale:
   - @chatui/core library selection
   - Swizzled Root integration method
   - localStorage session strategy
   - Context-aware citation navigation
   - React Error Boundary for errors

3. **data-model.md** - Frontend and backend entity definitions:
   - Message, Citation, ChatSession, ChatError interfaces
   - State transitions and lifecycle
   - API request/response models
   - Entity relationship diagram

4. **contracts/api-spec.yaml** - OpenAPI 3.0 specification:
   - POST /chat, POST /chat/stream (existing)
   - GET /conversations/{session_id} (new)
   - Complete schemas with examples

5. **quickstart.md** - Developer setup guide:
   - Prerequisites and quick setup steps
   - Development workflow
   - File structure after setup
   - Troubleshooting guide

## Outcome

- ✅ Impact: Complete technical blueprint for Spec-4 implementation
- 🧪 Tests: 18-item E2E smoke test checklist defined (P1-P3 prioritized)
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, quickstart.md, api-spec.yaml)
- 🔁 Next prompts: `/sp.tasks` to generate implementation task breakdown
- 🧠 Reflection: Existing Spec-3 backend has most infrastructure; only 2 modifications needed (CORS hardening + /conversations endpoint)

## Evaluation notes (flywheel)

- Failure modes observed: None - comprehensive analysis with spec4-integration-planner agent
- Graders run and results (PASS/FAIL): Constitution check - PASS (all principles satisfied)
- Prompt variant (if applicable): Used spec4-integration-planner agent for domain expertise
- Next experiment (smallest change to try): Consider adding WCAG accessibility target to plan
