---
id: "0001"
title: "Create RAG Agent API Specification"
stage: spec
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 006-rag-agent-api
branch: 006-rag-agent-api
user: developer
command: /sp.specify
labels: ["rag", "openai-agents-sdk", "fastapi", "qdrant", "spec"]
links:
  spec: specs/006-rag-agent-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-rag-agent-api/spec.md
  - specs/006-rag-agent-api/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

Spec 3: RAG Agent API (OpenAI Agents SDK + FastAPI + Qdrant)

Goal:
Build a FastAPI backend that hosts an OpenAI Agents SDK agent which answers questions about the book by retrieving relevant chunks from Qdrant and grounding responses in those sources.

Target audience:
Backend developers implementing an agentic RAG layer on top of the existing Qdrant retrieval pipeline.

Focus:
- Implement an agent with a retrieval tool that queries Qdrant (top-k + optional metadata filters) and returns cited sources.
- Expose API endpoints for chat (and optionally streaming), returning answer + sources (url/section/chunk refs).
- Support a "selected-text only" mode by accepting user-selected text (or a selection scope) and restricting grounding strictly to that selection.

Success criteria:
- For general questions, the agent retrieves from Qdrant and answers with source metadata for citations.
- For selected-text mode, responses are grounded only in the provided selection (or explicitly say "not found in selection")

## Response snapshot

Created feature branch `006-rag-agent-api` and comprehensive specification including:

1. **4 User Stories** with priorities (P1-P3):
   - P1: General Q&A with citations
   - P1: Selected-text-only mode
   - P2: Streaming chat API
   - P3: Configurable retrieval parameters

2. **12 Functional Requirements** covering:
   - `/chat` and `/chat/stream` endpoints
   - OpenAI Agents SDK integration
   - Qdrant retrieval tool with Spec-2 interface
   - Source citations (source_url, page_title, section_heading)
   - Selected-text mode with strict grounding
   - Health endpoint and error handling

3. **API Contracts** defined:
   - ChatRequest, ChatResponse, SourceCitation
   - StreamingResponse (SSE events)
   - ErrorResponse, HealthResponse

4. **10 Success Criteria** (measurable):
   - Response time < 5 seconds (p95)
   - First chunk latency < 1.5 seconds (p95)
   - 100% citation accuracy and grounding compliance

5. **Error Taxonomy** with HTTP status codes
6. **Constraints Catalog** aligned with Spec-1/Spec-2
7. **Environment Variables** documented

## Outcome

- ✅ Impact: High - enables conversational Q&A over book content with citations
- 🧪 Tests: N/A (specification phase - tests defined in tasks phase)
- 📁 Files: 2 files created (spec.md, requirements.md checklist)
- 🔁 Next prompts: /sp.plan to create architecture, /sp.tasks to generate implementation tasks
- 🧠 Reflection: Specification builds cleanly on Spec-1 (ingestion) and Spec-2 (retrieval) with clear dependency chain

## Evaluation notes (flywheel)

- Failure modes observed: None - all checklist items passed
- Graders run and results (PASS/FAIL): Quality checklist PASS (all items green)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider adding conversation memory as future spec if needed
