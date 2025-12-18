---
id: 004
title: ADR SSE Streaming Decision
stage: plan
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: 007-spec04-chatkit-frontend
branch: 007-spec04-chatkit-frontend
user: architect
command: /sp.adr
labels: ["adr", "sse", "streaming", "websocket", "architecture"]
links:
  spec: specs/007-spec04-chatkit-frontend/spec.md
  ticket: null
  adr: history/adr/0007-frontend-backend-real-time-communication-strategy.md
  pr: null
files:
  - history/adr/0007-frontend-backend-real-time-communication-strategy.md
tests:
  - N/A (architecture decision record)
---

## Prompt

/sp.adr sse-streaming-decision

## Response snapshot

Created ADR-0007: Frontend-Backend Real-Time Communication Strategy.

**Decision**: Use Server-Sent Events (SSE) for real-time streaming from backend to frontend.

**Clustered Components**:
- Transport Protocol: SSE via EventSource API
- Data Format: Newline-delimited JSON
- Event Types: chunk, sources, done, error
- Error Recovery: Browser-native auto-reconnect
- Backend: FastAPI StreamingResponse

**Alternatives Analyzed**:
1. WebSocket - Rejected (overkill, complexity, proxy issues)
2. Long Polling - Rejected (latency, resource intensive)
3. HTTP/2 Server Push - Rejected (wrong use case)

**Significance Checklist**:
- ✅ Impact: Affects real-time UX and infrastructure
- ✅ Alternatives: 3 alternatives with detailed tradeoffs
- ✅ Scope: Cross-cutting (frontend, backend, deployment)

## Outcome

- ✅ Impact: Documented architectural decision for team alignment
- 🧪 Tests: N/A (decision record)
- 📁 Files: 1 ADR created (0007-frontend-backend-real-time-communication-strategy.md)
- 🔁 Next prompts: `/sp.tasks` to generate implementation task breakdown
- 🧠 Reflection: SSE is the right choice for unidirectional streaming; leverages existing Spec-3 implementation

## Evaluation notes (flywheel)

- Failure modes observed: None - decision clearly passes significance test
- Graders run and results (PASS/FAIL):
  - Clustered decision (not atomic): PASS
  - Explicit alternatives with rationale: PASS
  - Clear pros/cons: PASS
  - Concise but detailed: PASS
- Prompt variant (if applicable): Standard /sp.adr with decision title
- Next experiment (smallest change to try): Consider documenting localStorage vs cookie decision as separate ADR if questioned
