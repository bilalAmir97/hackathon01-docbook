---
id: "0002"
title: "ADR for Idempotency Content Hash Strategy"
stage: plan
date: 2025-12-14
surface: agent
model: claude-opus-4-5-20251101
feature: 004-Spec01-embedding-pipeline
branch: 004-Spec01-embedding-pipeline
user: developer
command: /sp.adr
labels: ["adr", "idempotency", "hashing", "architecture"]
links:
  spec: ./spec.md
  ticket: null
  adr: ../../history/adr/0005-idempotency-strategy-via-content-hashing.md
  pr: null
files:
  - history/adr/0005-idempotency-strategy-via-content-hashing.md
tests:
  - null
---

## Prompt

/sp.adr idempotency-content-hash-strategy

Document the architectural decision for implementing idempotency in the embedding pipeline via deterministic point IDs and content hashing.

## Response snapshot

Created ADR-0005: Idempotency Strategy via Content Hashing

Decision documents a two-layer strategy:
1. Deterministic Point IDs: SHA-256(url + "|" + chunk_index)[:32]
2. Content Hash Change Detection: SHA-256(normalized_text) stored in Qdrant payload

Four alternatives were considered and rejected:
- Blind Upsert: Wastes API calls on unchanged content
- Last-Modified Header: Not reliably available from Vercel/Docusaurus
- File-Based Checkpoint: Can desync from Qdrant state
- MD5 Hash: Weaker collision resistance, no performance benefit

Consequences documented include API cost reduction (positive), query latency overhead (negative), and orphan detection gap (future work).

No conflicts with existing ADRs (0001-0004 are unrelated to data pipelines).

## Outcome

- Impact: Medium - Documents critical design decision for pipeline re-runs
- Tests: N/A (architecture documentation)
- Files: 1 ADR created
- Next prompts: /sp.tasks to generate implementation tasks
- Reflection: ADR properly clusters deterministic IDs + content hash as integrated strategy

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Significance test PASS (impacts API cost, has alternatives, cross-cutting)
- Prompt variant (if applicable): Standard /sp.adr with user-specified decision title
- Next experiment (smallest change to try): Implement hash comparison in pipeline main loop
