---
id: "0003"
title: "Spec-3 RAG Agent API Technical Plan"
stage: plan
date: 2025-12-16
surface: agent
model: claude-opus-4-5-20251101
feature: 006-rag-agent-api
branch: 006-rag-agent-api
user: user
command: /sp.plan
labels: ["rag", "fastapi", "openai-agents-sdk", "qdrant", "gemini"]
links:
  spec: specs/006-rag-agent-api/spec.md
  ticket: null
  adr: history/adr/0006-spec2-spec3-integration-pattern.md
  pr: null
files:
  - specs/006-rag-agent-api/plan.md
  - specs/006-rag-agent-api/research.md
  - specs/006-rag-agent-api/data-model.md
  - specs/006-rag-agent-api/quickstart.md
  - specs/006-rag-agent-api/contracts/api-contracts.md
tests:
  - null
---

## Prompt

use the spec-3 planner agent

* Create: architecture sketch for (FastAPI service → OpenAI Agents SDK agent → Qdrant retrieval tool → response with citations + selected-text-only mode); define minimal endpoint(s) + response schema.
* Research approach: research-concurrent; if Agents SDK details aren't in context, use Context7 MCP to fetch official OpenAI Agents SDK docs before finalizing tool wiring.
* Decisions needing documentation: streaming vs non-streaming; tool interface (inputs/outputs); citation payload shape; selected-text-only enforcement strategy; error/fallback behavior.
* Quality validation: logging/tracing plan + "no hallucinated citations" rule; handle empty/low-confidence retrieval safely.
* Testing strategy: API smoke tests for chat + selected-text; Qdrant connectivity check; deterministic test prompts with expected cited sources.
* - you can create agent.py file in backend folder in this file you can get embedding from qdrant you have qdrant url and qdran api and you have also qdrant collection name than retir

## Response snapshot

Created comprehensive technical plan for Spec-3: RAG Agent API including:

**Architecture**: FastAPI service → OpenAI Agents SDK agent → Qdrant retrieval tool → response with citations. Two modes: general (Qdrant RAG) and selected-text-only (no external retrieval).

**Key Decisions Documented**:
1. Streaming vs Non-Streaming: Both endpoints (`/chat` and `/chat/stream`)
2. Tool Interface: Single `@function_tool search_documentation` wrapping Spec-2 retrieve()
3. Citation Payload: Structured with source_url, page_title, section_heading, chunk_text, relevance_score
4. Selected-Text Enforcement: Agent configuration swap (remove tools entirely)
5. Error/Fallback: Typed error taxonomy with retry strategies

**Generated Artifacts**:
- plan.md: Complete technical architecture with diagrams, API contracts, testing strategy
- research.md: OpenAI Agents SDK patterns, citation extraction, enforcement strategies
- data-model.md: Pydantic v2 schemas, database schema, type mappings
- quickstart.md: Step-by-step implementation guide with code examples
- contracts/api-contracts.md: OpenAPI specification, cURL/Python examples

## Outcome

- ✅ Impact: Complete technical plan ready for implementation
- 🧪 Tests: Test strategy defined (49 tests across 7 categories)
- 📁 Files: 5 planning artifacts created
- 🔁 Next prompts: /sp.tasks to generate atomic implementation tasks
- 🧠 Reflection: Used spec3-agent-api-planner agent successfully; Context7 MCP provided OpenAI Agents SDK documentation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (planning phase)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement agent.py and validate tool wiring
