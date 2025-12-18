# Implementation Plan: Docusaurus ChatKit Frontend + Integration

**Branch**: `007-spec04-chatkit-frontend` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-spec04-chatkit-frontend/spec.md`

---

## Summary

Build a RAG chatbot frontend embedded in the Docusaurus book using `@chatui/core`, integrated with the existing FastAPI backend (Spec-3). The implementation includes:
- Global chat widget via swizzled Root theme component
- SSE streaming for real-time responses with citations
- Selected-text capture for focused Q&A mode
- Session persistence via localStorage + Neon Postgres
- Environment-based CORS hardening for security

---

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), Python 3.11 (backend modifications)
**Primary Dependencies**:
- Frontend: React 18, `@chatui/core`, Docusaurus 3.x
- Backend: FastAPI (existing), asyncpg (existing)
**Storage**: localStorage (session_id), Neon Postgres (conversation history)
**Testing**: Manual E2E smoke tests, browser dev tools validation
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge - last 2 versions)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <5s for streaming start, <100KB chat widget bundle (gzipped)
**Constraints**: Mobile-responsive (375px+), no CORS errors, graceful error handling
**Scale/Scope**: Single chat widget, ~8 React components, 2 backend modifications

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| IV. Clarity & Structure | ✅ PASS | Modular component design, clear separation |
| V. Accessibility & Readability | ✅ PASS | Mobile-responsive, keyboard accessible |
| Platform: Docusaurus | ✅ PASS | Uses standard Docusaurus patterns (swizzling) |
| No broken links/hallucinations | ✅ PASS | All citations link to actual book pages |

---

## Architecture

### System Overview (ASCII)

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           DOCUSAURUS BOOK (Frontend)                         │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                     src/theme/Root.tsx (Swizzled)                       ││
│  │  ┌─────────────────────────────────────────────────────────────────┐   ││
│  │  │                      <ChatWidget />                              │   ││
│  │  │  ┌──────────────┐  ┌───────────────┐  ┌────────────────────┐   │   ││
│  │  │  │ ChatToggle   │  │  ChatPanel    │  │ SelectedTextCtx    │   │   ││
│  │  │  │ (FAB Button) │  │               │  │ (Provider)         │   │   ││
│  │  │  └──────────────┘  │ ┌───────────┐ │  └────────────────────┘   │   ││
│  │  │                    │ │MessageList│ │                           │   ││
│  │  │                    │ └───────────┘ │                           │   ││
│  │  │                    │ ┌───────────┐ │                           │   ││
│  │  │                    │ │MessageIn  │ │                           │   ││
│  │  │                    │ └───────────┘ │                           │   ││
│  │  │                    └───────────────┘                           │   ││
│  │  └─────────────────────────────────────────────────────────────────┘   ││
│  └─────────────────────────────────────────────────────────────────────────┘│
│                                     │                                        │
│                                     │ fetch() + ReadableStream (SSE-over-POST)│
│                                     ▼                                        │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                    ┌─────────────────┼─────────────────┐
                    │                 │                 │
                    │           CORS Check              │
                    │   (ALLOWED_ORIGINS allowlist)     │
                    │                 │                 │
                    └─────────────────┼─────────────────┘
                                      │
┌─────────────────────────────────────▼───────────────────────────────────────┐
│                          FASTAPI BACKEND (Spec-3)                            │
│                                                                              │
│  ┌────────────────────┐  ┌────────────────────┐  ┌──────────────────────┐  │
│  │ POST /chat         │  │ POST /chat/stream  │  │ GET /conversations/  │  │
│  │ (sync response)    │  │ (SSE streaming)    │  │ {session_id}         │  │
│  └─────────┬──────────┘  └─────────┬──────────┘  └──────────┬───────────┘  │
│            │                       │                         │              │
│            └───────────┬───────────┴─────────────────────────┘              │
│                        │                                                     │
│                        ▼                                                     │
│  ┌─────────────────────────────────────────────────────────────────────────┐│
│  │                         agent.py (run_agent)                            ││
│  │  ┌─────────────────────┐    ┌────────────────────────────────────────┐ ││
│  │  │ Mode: "general"     │    │ Mode: "selected_text"                  │ ││
│  │  │ → Qdrant retrieval  │    │ → Use provided text only               │ ││
│  │  │ → Cohere embedding  │    │ → NO Qdrant calls                      │ ││
│  │  └─────────────────────┘    └────────────────────────────────────────┘ ││
│  └─────────────────────────────────────────────────────────────────────────┘│
│                        │                                                     │
│            ┌───────────┴───────────┐                                        │
│            ▼                       ▼                                        │
│  ┌─────────────────────┐  ┌─────────────────────┐                          │
│  │   Qdrant Cloud      │  │   Neon Postgres     │                          │
│  │   (rag_embedding)   │  │   (conversations)   │                          │
│  │   - 1024-dim vectors│  │   - save_convo()    │                          │
│  │   - Cohere embed    │  │   - get_history()   │                          │
│  └─────────────────────┘  └─────────────────────┘                          │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Data Flow: General Query

```
User types question → MessageInput
         │
         ▼
ChatWidget.handleSend(query)
         │
         ├── Get session_id from localStorage
         ├── Check for selected_text context (null for general mode)
         │
         ▼
APIClient.streamChat({
  query: "What is ROS 2?",
  session_id: "uuid-here",
  mode: "general",
  selected_text: null,
  stream: true
})
         │
         ▼
fetch() + ReadableStream → POST /chat/stream
(Manual SSE parsing - see ADR-001 implementation)
         │
         ▼
FastAPI receives request
         │
         ├── CORS check (ALLOWED_ORIGINS)
         ├── Generate trace_id
         │
         ▼
run_agent_streamed(query, mode="general", ...)
         │
         ├── embed_query(query) via Cohere
         ├── retrieve() from Qdrant
         ├── Gemini generates answer with @function_tool
         │
         ▼
Yield SSE events:
  {"type":"chunk","content":"ROS 2 is..."}
  {"type":"chunk","content":" a robotics..."}
  {"type":"sources","sources":[{source_url, page_title, ...}]}
  {"type":"done","metadata":{query_time_ms, chunks_retrieved}}
         │
         ▼
Frontend processes events:
  - Append chunks to message
  - Store sources for citation rendering
  - Update loading state
         │
         ▼
save_conversation(session_id, query, response, sources, ...)
         │
         ▼
MessageList renders:
  - User message bubble
  - Assistant message with streaming text
  - CitationList with clickable links
```

### Data Flow: Selected-Text Mode

```
User selects text on book page
         │
         ▼
document.addEventListener('mouseup', captureSelection)
         │
         ├── window.getSelection().toString()
         ├── Validate length < 10,000 chars
         ├── Get selection bounding rect for positioning
         │
         ▼
SelectedTextContext.setSelectedText(text)
         │
         ▼
Floating tooltip appears near selection:
┌─────────────────┐
│ 💬 Ask about this │  ← Positioned above/below selection
└─────────────────┘
         │
         ▼
User clicks tooltip button
         │
         ▼
ChatWidget opens with selected text visible
         │
         ▼
User types question about the selection
         │
         ▼
APIClient.streamChat({
  query: "Explain this passage",
  session_id: "uuid-here",
  mode: "selected_text",          // ← Different mode
  selected_text: "The captured text from the book...",
  stream: true
})
         │
         ▼
FastAPI /chat/stream
         │
         ▼
run_agent_streamed(query, mode="selected_text", selected_text="...")
         │
         ├── NO Qdrant retrieval (mode check)
         ├── Gemini answers using ONLY selected_text
         │
         ▼
If answer found in selection:
  → Return answer with source: "From your selection"

If answer NOT in selection:
  → "The provided selection does not contain information about [topic].
     Would you like me to search the full documentation?"
```

---

## Project Structure

### Documentation (this feature)

```text
specs/007-spec04-chatkit-frontend/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── api-spec.yaml    # OpenAPI specification
└── tasks.md             # Phase 2 output (/sp.tasks)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
src/
├── theme/
│   └── Root.tsx                 # NEW: Swizzled Root component
├── components/
│   └── Chat/
│       ├── index.ts             # NEW: Barrel export
│       ├── ChatWidget.tsx       # NEW: Main widget container
│       ├── ChatToggle.tsx       # NEW: FAB toggle button
│       ├── ChatPanel.tsx        # NEW: Expandable chat panel
│       ├── MessageList.tsx      # NEW: Message display
│       ├── MessageInput.tsx     # NEW: Input with send button
│       ├── CitationList.tsx     # NEW: Citation rendering
│       ├── SelectedTextBadge.tsx# NEW: Selected text indicator in chat
│       ├── SelectionTooltip.tsx # NEW: Floating "Ask about this" button
│       └── ErrorFallback.tsx    # NEW: Error boundary fallback
├── hooks/
│   ├── useChat.ts               # NEW: Chat state management
│   ├── useSelectedText.ts       # NEW: Text selection capture
│   └── useSession.ts            # NEW: Session ID management
├── services/
│   └── api.ts                   # NEW: Backend API client
├── contexts/
│   └── ChatContext.tsx          # NEW: Chat state context
└── css/
    └── chat.module.css          # NEW: Chat-specific styles

# Backend (existing, with modifications)
backend/
├── api.py                       # MODIFY: CORS + /conversations endpoint
├── database.py                  # EXISTING: Already has get_conversation_history()
├── api_models.py                # MODIFY: Add ConversationsResponse model
└── ...                          # Other existing files unchanged

# Configuration
docusaurus.config.ts             # MODIFY: Add customFields.apiUrl
.env.example                     # MODIFY: Add ALLOWED_ORIGINS, API_URL
```

**Structure Decision**: Web application pattern with Docusaurus frontend extending existing backend. Frontend components use Docusaurus conventions (src/theme for swizzling, src/components for reusable UI).

---

## API Contracts

### Existing Endpoints (Spec-3)

#### POST /chat
Synchronous chat with full response.

```yaml
Request:
  Content-Type: application/json
  Body:
    query: string           # Required, 1-2000 chars
    session_id: string|null # UUID v4 or null
    mode: "general"|"selected_text"  # Default: "general"
    selected_text: string|null       # Required if mode="selected_text", max 10000 chars
    top_k: integer          # Default: 5, range 1-20
    score_threshold: float|null      # Range 0.0-1.0
    filters: object|null    # Metadata filters

Response (200):
  answer: string
  sources: SourceCitation[]
  mode: string
  metadata:
    query_time_ms: float
    chunks_retrieved: integer
    model: string

SourceCitation:
  source_url: string
  page_title: string
  section_heading: string
  chunk_text: string        # Truncated to 500 chars
  relevance_score: float    # 0.0-1.0
```

#### POST /chat/stream
SSE streaming response.

```yaml
Request: Same as /chat

Response: text/event-stream
Events:
  - type: "chunk"
    content: string         # Partial answer text

  - type: "sources"
    sources: SourceCitation[]

  - type: "done"
    metadata:
      query_time_ms: float
      chunks_retrieved: integer
      model: string

  - type: "error"
    error_code: string
    message: string
```

### New Endpoint (Spec-4)

#### GET /conversations/{session_id}
Retrieve conversation history for a session.

```yaml
Path Parameters:
  session_id: string        # UUID v4

Query Parameters:
  limit: integer            # Default: 20, max: 20

Response (200):
  session_id: string
  messages: ConversationMessage[]

ConversationMessage:
  id: string                # UUID
  role: "user"|"assistant"
  content: string
  sources: SourceCitation[]|null
  mode: string
  created_at: string        # ISO 8601

Response (404):
  error_code: "session_not_found"
  message: "No conversations found for session"
```

---

## Backend Modifications

### 1. CORS Hardening (api.py:78-84)

**Current (INSECURE):**
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # ← Security risk
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Required (SECURE):**
```python
import os

ALLOWED_ORIGINS = os.getenv(
    "ALLOWED_ORIGINS",
    "http://localhost:3000,http://localhost:3001"
).split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
    max_age=600,  # Preflight cache: 10 minutes
)
```

### 2. New Endpoint (api.py)

```python
from database import get_conversation_history

@app.get("/conversations/{session_id}")
async def get_conversations(
    session_id: str,
    limit: int = 20,
) -> dict:
    """Retrieve conversation history for a session."""
    # Validate session_id format (UUID v4)
    try:
        uuid.UUID(session_id, version=4)
    except ValueError:
        raise HTTPException(
            status_code=400,
            detail={"error_code": "invalid_session_id", "message": "Invalid UUID format"}
        )

    # Enforce limit
    limit = min(limit, 20)

    # Get history from database
    history = await get_conversation_history(session_id, limit=limit)

    # Transform to API format
    messages = []
    for conv in history:
        # User message
        messages.append({
            "id": str(uuid.uuid4()),
            "role": "user",
            "content": conv["query"],
            "sources": None,
            "mode": conv["mode"],
            "created_at": conv["created_at"],
        })
        # Assistant message
        messages.append({
            "id": str(uuid.uuid4()),
            "role": "assistant",
            "content": conv["response"],
            "sources": conv["sources"],
            "mode": conv["mode"],
            "created_at": conv["created_at"],
        })

    return {"session_id": session_id, "messages": messages}
```

---

## Frontend Components

### Component Hierarchy

```
<Root>                              # Swizzled theme component
└── <ChatContext.Provider>          # Global chat state
    └── <SelectedTextContext.Provider>  # Selected text state
        ├── {children}              # Docusaurus page content
        ├── <SelectionTooltip />    # Floating "Ask about this" (appears on text select)
        └── <ChatWidget>            # Fixed position widget
            ├── <ChatToggle />      # FAB button (bottom-right)
            └── <ChatPanel>         # Expandable panel
                ├── <ChatHeader>    # Title + New Conversation button
                ├── <SelectedTextBadge />  # Shows captured text in chat
                ├── <MessageList>   # Scrollable messages
                │   └── <Message>   # Individual message bubble
                │       └── <CitationList>  # Clickable citations
                ├── <MessageInput>  # Text input + send
                └── <ErrorFallback> # Error boundary fallback
```

### Key Implementation Notes

1. **Root.tsx**: Wrap entire app with providers
2. **useChat hook**: Manages messages state, streaming, history loading
3. **useSelectedText hook**: Uses `mouseup` event to capture Selection API + bounding rect
4. **useSession hook**: `crypto.randomUUID()` + localStorage persistence
5. **api.ts**: `fetch()` + `ReadableStream` for SSE-over-POST (see ADR-001 implementation)
6. **SelectionTooltip.tsx**: Floating button positioned via `getBoundingClientRect()`:
   - Appears above selection (or below if near top of viewport)
   - Uses `position: fixed` with calculated top/left
   - Hides on click outside, scroll, or Escape key
   - Clicking opens chat with selected text pre-loaded

---

## Environment Configuration

### Frontend (.env / docusaurus.config.ts)

```typescript
// docusaurus.config.ts
const config: Config = {
  // ... existing config
  customFields: {
    apiUrl: process.env.DOCUSAURUS_API_URL || 'http://localhost:8000',
  },
};
```

```bash
# .env (frontend)
DOCUSAURUS_API_URL=http://localhost:8000  # Dev
# DOCUSAURUS_API_URL=https://api.production.com  # Prod
```

### Backend (.env)

```bash
# Existing (Spec-3)
DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
COHERE_API_KEY=...
GEMINI_API_KEY=...

# New (Spec-4)
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001,https://your-book-domain.com
```

---

## Architectural Decisions

### ADR-001: SSE over WebSocket for Streaming

**Decision**: Use Server-Sent Events (SSE) instead of WebSocket.

**Rationale**:
- SSE is simpler for unidirectional server→client streaming
- Built-in browser EventSource API with auto-reconnect
- Works through HTTP/2 multiplexing
- FastAPI already implements SSE in Spec-3

**Alternatives Rejected**:
- WebSocket: Overkill for unidirectional flow, more complex error handling
- Long-polling: Higher latency, more requests

**Implementation Note - SSE-over-POST Pattern**:

Native `EventSource` API only supports GET requests, but our `/chat/stream` endpoint requires POST (to send query payload). Solution: Use `fetch()` with `ReadableStream` to manually parse SSE events.

```typescript
// services/api.ts - SSE-over-POST Implementation

interface StreamCallbacks {
  onChunk: (content: string) => void;
  onSources: (sources: Citation[]) => void;
  onDone: (metadata: ResponseMetadata) => void;
  onError: (error: ChatError) => void;
}

export async function streamChat(
  request: ChatRequest,
  callbacks: StreamCallbacks,
  signal?: AbortSignal
): Promise<void> {
  const response = await fetch(`${API_BASE_URL}/chat/stream`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
    signal,
  });

  if (!response.ok) {
    const error = await response.json();
    callbacks.onError(error);
    return;
  }

  const reader = response.body?.getReader();
  if (!reader) {
    callbacks.onError({ error_code: 'stream_error', message: 'No response body' });
    return;
  }

  const decoder = new TextDecoder();
  let buffer = '';

  try {
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n\n');
      buffer = lines.pop() || ''; // Keep incomplete chunk in buffer

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          const jsonStr = line.slice(6); // Remove 'data: ' prefix
          try {
            const event = JSON.parse(jsonStr);

            switch (event.type) {
              case 'chunk':
                callbacks.onChunk(event.content);
                break;
              case 'sources':
                callbacks.onSources(event.sources);
                break;
              case 'done':
                callbacks.onDone(event.metadata);
                break;
              case 'error':
                callbacks.onError(event);
                break;
            }
          } catch (parseError) {
            console.error('Failed to parse SSE event:', parseError);
          }
        }
      }
    }
  } finally {
    reader.releaseLock();
  }
}
```

**Usage in useChat hook**:

```typescript
// hooks/useChat.ts

const sendMessage = async (query: string) => {
  const abortController = new AbortController();
  setIsLoading(true);

  // Add user message immediately
  addMessage({ role: 'user', content: query });

  // Add placeholder for assistant response
  const assistantMsgId = addMessage({ role: 'assistant', content: '' });

  await streamChat(
    {
      query,
      session_id: sessionId,
      mode: selectedText ? 'selected_text' : 'general',
      selected_text: selectedText,
      stream: true,
    },
    {
      onChunk: (content) => {
        // Append to existing assistant message
        updateMessage(assistantMsgId, (prev) => prev + content);
      },
      onSources: (sources) => {
        setMessageSources(assistantMsgId, sources);
      },
      onDone: (metadata) => {
        setIsLoading(false);
        setQueryMetadata(metadata);
      },
      onError: (error) => {
        setIsLoading(false);
        setError(error);
      },
    },
    abortController.signal
  );
};
```

**Reconnection Strategy**:

Since we're using `fetch()` instead of native `EventSource`, we need manual reconnection:

```typescript
const MAX_RETRIES = 5;
const INITIAL_DELAY_MS = 1000;
const MAX_DELAY_MS = 30000;
const BACKOFF_MULTIPLIER = 2;

async function streamChatWithRetry(
  request: ChatRequest,
  callbacks: StreamCallbacks,
  retryCount = 0
): Promise<void> {
  try {
    await streamChat(request, callbacks);
  } catch (error) {
    if (retryCount < MAX_RETRIES && isRetryableError(error)) {
      const delay = Math.min(
        INITIAL_DELAY_MS * Math.pow(BACKOFF_MULTIPLIER, retryCount),
        MAX_DELAY_MS
      );
      callbacks.onRetrying?.(delay, retryCount + 1);
      await sleep(delay);
      return streamChatWithRetry(request, callbacks, retryCount + 1);
    }
    throw error;
  }
}

function isRetryableError(error: unknown): boolean {
  // Retry on network errors, 503, 502, but NOT on 400/401/429
  if (error instanceof TypeError) return true; // Network error
  if (error && typeof error === 'object' && 'status' in error) {
    const status = (error as { status: number }).status;
    return status === 502 || status === 503 || status === 504;
  }
  return false;
}
```

### ADR-002: localStorage over Cookie for Session ID

**Decision**: Store session_id in localStorage, not cookies.

**Rationale**:
- No need for server-side session reading
- Simpler implementation (no cookie parsing)
- Works without HTTPS in development
- User can clear via "New Conversation" button

**Alternatives Rejected**:
- Cookies: Adds complexity for cross-origin requests, CSRF concerns
- sessionStorage: Session lost on tab close (bad UX)

### ADR-003: Swizzled Root Component for Global Widget

**Decision**: Use Docusaurus theme swizzling for Root component.

**Rationale**:
- Widget persists across all page navigations
- Chat state preserved during SPA transitions
- Standard Docusaurus pattern for global components
- No need for per-page MDX embedding

**Alternatives Rejected**:
- Custom plugin: Higher complexity, same result
- MDX embed: Manual per-page, state lost on navigation

---

## Testing Checklist

### E2E Smoke Tests (Priority Order)

| # | Test | Expected | Priority |
|---|------|----------|----------|
| 1 | Chat widget FAB visible on homepage | Floating button in bottom-right | P1 |
| 2 | Click FAB opens chat panel | Panel expands with input field | P1 |
| 3 | Type question and submit | Message appears, loading indicator shows | P1 |
| 4 | Response streams progressively | Text appears word-by-word | P1 |
| 5 | Citations render after response | Clickable links below answer | P1 |
| 6 | Click internal citation | Navigates to book page, chat stays open | P1 |
| 7 | No CORS errors in console | DevTools shows no red CORS messages | P1 |
| 8 | Select text → "Ask about this" | Chat opens with selection context | P1 |
| 9 | Selected-text mode enforced | Response says "from your selection" | P1 |
| 10 | Reload page → history loads | Previous messages restored | P2 |
| 11 | "New Conversation" clears state | New session_id, empty history | P2 |
| 12 | Navigate to other page | Chat state persists (open/messages) | P2 |
| 13 | Backend 503 → user-friendly error | "Service unavailable" not raw error | P2 |
| 14 | Backend 429 → rate limit message | Shows wait time | P2 |
| 15 | Mobile viewport (375px) | Chat panel fullscreen, usable | P3 |
| 16 | Empty input → send disabled | Submit button grayed out | P3 |
| 17 | Long response (>2000 words) | Renders completely, scrollable | P3 |
| 18 | Private browsing → warning | Ephemeral session message | P3 |

---

## Complexity Tracking

> No constitution violations. Implementation follows standard patterns.

| Item | Complexity | Justification |
|------|------------|---------------|
| 8 React components | Low | Single responsibility, reusable |
| 3 custom hooks | Low | Separation of concerns |
| 2 backend changes | Low | Minimal modification to existing code |
| SSE streaming | Medium | Browser-native EventSource API |

---

## Next Steps

1. **Run `/sp.tasks`** to generate implementation task breakdown
2. **Consider ADR** for SSE streaming decision (📋 Architectural decision detected)
3. **Validate** `@chatui/core` compatibility with Docusaurus React version
