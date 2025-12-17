# Data Model: Docusaurus ChatKit Frontend + Integration

**Feature**: 007-spec04-chatkit-frontend
**Date**: 2025-12-17

---

## Frontend Entities

### Message

Represents a single message in the chat conversation.

```typescript
interface Message {
  id: string;           // UUID v4, generated client-side for optimistic updates
  role: 'user' | 'assistant';
  content: string;      // Message text
  sources?: Citation[]; // Only for assistant messages
  mode: 'general' | 'selected_text';
  status: 'pending' | 'streaming' | 'complete' | 'error';
  createdAt: Date;
}
```

**State Transitions**:
```
[user sends message]
    → pending (optimistic)
    → complete (user messages complete immediately)

[assistant response]
    → pending (waiting for first chunk)
    → streaming (receiving SSE chunks)
    → complete (received "done" event)
    → error (received "error" event or network failure)
```

---

### Citation

Represents a source citation from the assistant's response.

```typescript
interface Citation {
  sourceUrl: string;      // URL of the book page
  pageTitle: string;      // Human-readable page title
  sectionHeading: string; // Section within the page
  chunkText: string;      // Relevant text snippet (max 500 chars)
  relevanceScore: number; // 0.0 - 1.0
}
```

**Special Cases**:
- Selected-text mode: `sourceUrl = "selected_text"`, `pageTitle = "From your selection"`

---

### ChatSession

Represents the current chat session state.

```typescript
interface ChatSession {
  sessionId: string;     // UUID v4, persisted in localStorage
  messages: Message[];   // Conversation history
  isOpen: boolean;       // Chat panel visibility
  isLoading: boolean;    // Request in progress
  selectedText: string | null;  // Captured text for selected_text mode
  error: ChatError | null;      // Current error state
}
```

**Lifecycle**:
```
[first visit]
    → Generate new sessionId via crypto.randomUUID()
    → Store in localStorage under "chat_session_id"

[subsequent visits]
    → Read sessionId from localStorage
    → Load history from GET /conversations/{sessionId}

[New Conversation button]
    → Generate new sessionId
    → Clear messages
    → Update localStorage
```

---

### ChatError

Represents error states in the chat.

```typescript
interface ChatError {
  code: 'network' | 'cors' | 'rate_limited' | 'server_error' | 'validation';
  message: string;       // User-friendly message
  retryAfter?: number;   // Seconds to wait (for rate limiting)
  canRetry: boolean;     // Whether retry button should show
}
```

---

## Backend Entities (Existing)

### Conversation (Neon Postgres)

From `backend/database.py` - schema already implemented.

```sql
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) NOT NULL,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    sources JSONB DEFAULT '[]'::jsonb,
    mode VARCHAR(50) NOT NULL DEFAULT 'general',
    selected_text TEXT,
    chunks_retrieved INTEGER DEFAULT 0,
    latency_ms FLOAT DEFAULT 0.0,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Indexes
CREATE INDEX idx_conversations_session_id ON conversations(session_id);
CREATE INDEX idx_conversations_session_created ON conversations(session_id, created_at DESC);
```

**Constraints**:
- `session_id`: UUID v4 format validated
- `mode`: Either "general" or "selected_text"
- `sources`: JSONB array of SourceCitation objects
- Retention: 30 days (handled by backend cleanup)

---

## API Request/Response Models

### ChatRequest

```typescript
interface ChatRequest {
  query: string;                    // 1-2000 chars
  session_id?: string;              // UUID v4 or null
  mode?: 'general' | 'selected_text'; // Default: 'general'
  selected_text?: string;           // Required if mode='selected_text', max 10000 chars
  stream?: boolean;                 // Default: true for frontend
  top_k?: number;                   // Default: 5, range 1-20
  score_threshold?: number;         // Range 0.0-1.0
  filters?: Record<string, unknown>;
}
```

### ChatResponse (Sync)

```typescript
interface ChatResponse {
  answer: string;
  sources: Citation[];
  mode: 'general' | 'selected_text';
  metadata: {
    query_time_ms: number;
    chunks_retrieved: number;
    model: string;
  };
}
```

### StreamEvent (SSE)

```typescript
type StreamEvent =
  | { type: 'chunk'; content: string }
  | { type: 'sources'; sources: Citation[] }
  | { type: 'done'; metadata: ResponseMetadata }
  | { type: 'error'; error_code: string; message: string };
```

### ConversationsResponse (New)

```typescript
interface ConversationsResponse {
  session_id: string;
  messages: Array<{
    id: string;
    role: 'user' | 'assistant';
    content: string;
    sources?: Citation[];
    mode: string;
    created_at: string;  // ISO 8601
  }>;
}
```

---

## Storage

### localStorage Keys

| Key | Type | Description |
|-----|------|-------------|
| `chat_session_id` | string (UUID) | Current session identifier |
| `chat_is_open` | boolean | Widget open/closed state |

### State Management

Frontend uses React Context for global chat state:

```typescript
interface ChatContextValue {
  session: ChatSession;
  // Actions
  sendMessage: (query: string) => Promise<void>;
  loadHistory: () => Promise<void>;
  newConversation: () => void;
  setSelectedText: (text: string | null) => void;
  clearError: () => void;
  toggleOpen: () => void;
}
```

---

## Validation Rules

### Query Validation
- Min length: 1 character
- Max length: 2000 characters
- Cannot be empty or whitespace-only

### Selected Text Validation
- Max length: 10000 characters
- Required when mode = "selected_text"

### Session ID Validation
- Must be valid UUID v4 format
- Backend validates: `uuid.UUID(session_id, version=4)`

---

## Entity Relationships

```
┌─────────────────┐     1:N     ┌─────────────────┐
│   ChatSession   │────────────▶│     Message     │
│                 │             │                 │
│ - sessionId     │             │ - id            │
│ - messages[]    │             │ - role          │
│ - isOpen        │             │ - content       │
│ - selectedText  │             │ - sources[]     │
└─────────────────┘             └────────┬────────┘
                                         │
                                         │ 1:N (assistant only)
                                         ▼
                                ┌─────────────────┐
                                │    Citation     │
                                │                 │
                                │ - sourceUrl     │
                                │ - pageTitle     │
                                │ - chunkText     │
                                └─────────────────┘
```
