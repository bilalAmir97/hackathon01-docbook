# Implementation Tasks: Docusaurus ChatKit Frontend + Integration

**Feature**: 007-spec04-chatkit-frontend
**Branch**: `007-spec04-chatkit-frontend`
**Generated**: 2025-12-17
**Updated**: 2025-12-17
**Total Tasks**: 36
**Completed**: 36
**Phases**: 9

---

## Task Summary

| Phase | Tasks | Description |
|-------|-------|-------------|
| 1 | T001-T004 | Backend Modifications |
| 2 | T005-T009 | Frontend Foundation |
| 3 | T010-T012b | Custom Hooks |
| 4 | T013-T018 | Core Components |
| 5 | T019-T020 | Selected Text Features |
| 6 | T021-T022 | Error Handling |
| 7 | T023-T025 | Integration & Assembly |
| 8 | T026-T033 | E2E Verification |
| 9 | T034-T035 | Documentation |

---

## Phase 1: Backend Modifications

### T001: CORS Hardening ✅
**Priority**: P1 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Update FastAPI CORS middleware to use environment-based allowlist instead of wildcard origins.

**Files**:
- Modify: `backend/api.py` (lines 77-84)
- Modify: `backend/.env.example`

**Acceptance Criteria**:
- [ ] `ALLOWED_ORIGINS` environment variable is read from `.env`
- [ ] Default value is `http://localhost:3000,http://localhost:3001`
- [ ] Origins are split by comma into a list
- [ ] `allow_methods` restricted to `["GET", "POST", "OPTIONS"]`
- [ ] `allow_headers` restricted to `["Content-Type", "Authorization"]`
- [ ] `max_age=600` set for preflight caching
- [ ] Backend starts without errors with new config
- [ ] CORS preflight request from allowed origin succeeds
- [ ] CORS preflight request from disallowed origin fails

**Test Command**:
```bash
# Test allowed origin
curl -X OPTIONS http://localhost:8000/chat \
  -H "Origin: http://localhost:3000" \
  -H "Access-Control-Request-Method: POST" -v

# Test disallowed origin (should fail)
curl -X OPTIONS http://localhost:8000/chat \
  -H "Origin: http://evil.com" \
  -H "Access-Control-Request-Method: POST" -v
```

---

### T002: Add ConversationsResponse Pydantic Model ✅
**Priority**: P1 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Add Pydantic models for the `/conversations/{session_id}` endpoint response.

**Files**:
- Modify: `backend/api_models.py`

**Acceptance Criteria**:
- [ ] `ConversationMessage` model with fields: id, role, content, sources, mode, created_at
- [ ] `ConversationsResponse` model with fields: session_id, messages (list)
- [ ] All field types match `api-spec.yaml` schema
- [ ] Models import successfully without errors
- [ ] JSON serialization works correctly

**Code Pattern**:
```python
class ConversationMessage(BaseModel):
    id: str
    role: Literal["user", "assistant"]
    content: str
    sources: list[SourceCitation] | None = None
    mode: Literal["general", "selected_text"]
    created_at: datetime

class ConversationsResponse(BaseModel):
    session_id: str
    messages: list[ConversationMessage]
```

---

### T003: Add retry_after to ErrorResponse Model ✅
**Priority**: P1 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Add `retry_after` field to ErrorResponse model for 429 rate limit handling.

**Files**:
- Modify: `backend/api_models.py`

**Acceptance Criteria**:
- [ ] `retry_after: int | None = None` field added to ErrorResponse
- [ ] Field description: "Seconds to wait (for rate limits)"
- [ ] Existing error responses still work
- [ ] 429 responses can include retry_after value

---

### T004: Implement GET /conversations/{session_id} Endpoint ✅
**Priority**: P1 | **Dependencies**: T002 | **Status**: COMPLETED

**Description**: Add new endpoint to retrieve conversation history for a session.

**Files**:
- Modify: `backend/api.py`

**Acceptance Criteria**:
- [ ] Endpoint: `GET /conversations/{session_id}`
- [ ] Query parameter: `limit` (default: 20, max: 20)
- [ ] UUID v4 validation for session_id (return 400 if invalid)
- [ ] Uses existing `get_conversation_history()` from database.py
- [ ] Returns ConversationsResponse with user/assistant message pairs
- [ ] Empty history returns `{"session_id": "...", "messages": []}`
- [ ] Response ordered oldest to newest

**Test Command**:
```bash
# Valid session
curl http://localhost:8000/conversations/550e8400-e29b-41d4-a716-446655440000

# Invalid UUID format
curl http://localhost:8000/conversations/invalid-uuid
# Expected: 400 {"error_code": "invalid_session_id", ...}
```

---

## Phase 2: Frontend Foundation

### T005: Install Frontend Dependencies ✅
**Priority**: P1 | **Dependencies**: None | **Status**: COMPLETED (native APIs used)

**Description**: Install required npm packages for the chat UI.

**Files**:
- Modify: `package.json`

**Acceptance Criteria**:
- [ ] `@chatui/core` installed
- [ ] `uuid` installed (for session ID generation fallback)
- [ ] No peer dependency conflicts
- [ ] `npm install` completes without errors
- [ ] `npm run build` still works

**Command**:
```bash
npm install @chatui/core uuid
npm install --save-dev @types/uuid
```

---

### T006: Configure Docusaurus customFields ✅
**Priority**: P1 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Add API URL configuration to Docusaurus config.

**Files**:
- Modify: `docusaurus.config.ts`

**Acceptance Criteria**:
- [ ] `customFields.apiUrl` added to config
- [ ] Reads from `process.env.DOCUSAURUS_API_URL`
- [ ] Default fallback: `http://localhost:8000`
- [ ] Config validates successfully (`npm run build` passes)

**Code Pattern**:
```typescript
const config: Config = {
  // ... existing
  customFields: {
    apiUrl: process.env.DOCUSAURUS_API_URL || 'http://localhost:8000',
  },
};
```

---

### T007: Create TypeScript Type Definitions ✅
**Priority**: P1 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Create TypeScript interfaces matching backend models and API contracts.

**Files**:
- Create: `src/types/chat.ts`

**Acceptance Criteria**:
- [ ] `Message` interface (id, role, content, sources, mode, createdAt)
- [ ] `Citation` interface (sourceUrl, pageTitle, sectionHeading, chunkText, relevanceScore)
- [ ] `ChatRequest` interface
- [ ] `ChatResponse` interface
- [ ] `StreamEvent` union type (chunk | sources | done | error)
- [ ] `ChatError` interface (error_code, message, retry_after?)
- [ ] All types exported from index
- [ ] No TypeScript errors

---

### T008: Create API Service with SSE-over-POST ✅
**Priority**: P1 | **Dependencies**: T006, T007 | **Status**: COMPLETED

**Description**: Implement API client with fetch + ReadableStream for SSE streaming.

**Files**:
- Create: `src/services/api.ts`

**Acceptance Criteria**:
- [ ] `getApiUrl()` reads from Docusaurus customFields
- [ ] `streamChat()` function calls `/chat/stream` endpoint with callbacks (onChunk, onSources, onDone, onError)
- [ ] `sendChat()` function calls `/chat` endpoint for sync responses (optional fallback)
- [ ] Uses `fetch()` with POST method
- [ ] Parses SSE `data: {...}\n\n` format using ReadableStream
- [ ] Handles buffer for incomplete chunks
- [ ] AbortController signal support for cancellation
- [ ] `streamChatWithRetry()` wrapper with exponential backoff
- [ ] Retry params: initial=1s, multiplier=2x, max=30s, maxRetries=5
- [ ] `isRetryableError()` only retries 502, 503, 504
- [ ] `loadConversationHistory()` for GET /conversations
- [ ] All functions typed correctly

---

### T009: Create Environment Configuration ✅
**Priority**: P2 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Document and set up environment variables for frontend.

**Files**:
- Modify: `.env.example`

**Acceptance Criteria**:
- [ ] `DOCUSAURUS_API_URL` documented with examples
- [ ] Dev default: `http://localhost:8000`
- [ ] Production placeholder documented
- [ ] Instructions for local development

---

## Phase 3: Custom Hooks

### T010: Implement useSession Hook ✅
**Priority**: P1 | **Dependencies**: T007 | **Status**: COMPLETED

**Description**: Hook for session ID management with localStorage persistence.

**Files**:
- Create: `src/hooks/useSession.ts`

**Acceptance Criteria**:
- [ ] Generates UUID v4 via `crypto.randomUUID()` (fallback to uuid package)
- [ ] Stores in localStorage key: `chat_session_id`
- [ ] Retrieves existing on mount
- [ ] `resetSession()` function clears and generates new ID
- [ ] Detects private browsing (try-catch localStorage)
- [ ] Returns `{ sessionId, resetSession, isPrivateBrowsing }`

**Test Cases**:
- [ ] New visitor gets new UUID
- [ ] Returning visitor gets same UUID
- [ ] resetSession() generates different UUID
- [ ] Private browsing returns fallback behavior

---

### T011: Implement useSelectedText Hook ✅
**Priority**: P1 | **Dependencies**: T007 | **Status**: COMPLETED

**Description**: Hook for capturing text selection with bounding rect for tooltip positioning.

**Files**:
- Create: `src/hooks/useSelectedText.ts`

**Acceptance Criteria**:
- [ ] Listens to `mouseup` event on document
- [ ] Captures `window.getSelection().toString()`
- [ ] Validates length < 10,000 characters
- [ ] Gets bounding rect via `selection.getRangeAt(0).getBoundingClientRect()`
- [ ] Clears selection on Escape key, click outside, scroll
- [ ] Returns `{ selectedText, selectionRect, clearSelection }`
- [ ] Cleans up event listeners on unmount

**Test Cases**:
- [ ] Selecting text updates state
- [ ] Empty selection returns null
- [ ] Long text (>10k chars) is truncated or rejected
- [ ] Escape key clears selection

---

### T012: Implement useChat Hook ✅
**Priority**: P1 | **Dependencies**: T008, T010, T011 | **Status**: COMPLETED

**Description**: Main hook managing chat state, messages, and streaming.

**Files**:
- Create: `src/hooks/useChat.ts`

**Acceptance Criteria**:
- [ ] Manages messages array state
- [ ] `sendMessage(query)` function
- [ ] Adds user message immediately to UI
- [ ] Creates placeholder assistant message for streaming
- [ ] Calls `streamChat()` with callbacks to update message
- [ ] `onChunk` appends to assistant message content
- [ ] `onSources` attaches citations to assistant message
- [ ] `onDone` marks loading complete
- [ ] `onError` sets error state
- [ ] `loadHistory()` fetches from /conversations on mount
- [ ] `clearChat()` resets messages and calls resetSession
- [ ] `isLoading` state during streaming
- [ ] `error` state for displaying errors
- [ ] Integrates selectedText into request when present

**Test Cases**:
- [ ] Sending message shows in UI immediately
- [ ] Streaming updates message progressively
- [ ] Sources appear after streaming completes
- [ ] Error state displays on failure
- [ ] History loads on mount if session exists

---

### T012b: Implement Message Queue/Debounce Logic ✅
**Priority**: P1 | **Dependencies**: T012 | **Status**: COMPLETED

**Description**: Add debounce and queue logic to prevent rapid message submissions from causing race conditions or duplicate requests.

**Files**:
- Modify: `src/hooks/useChat.ts`

**Acceptance Criteria**:
- [ ] Submit button disables immediately on click (before API call)
- [ ] Debounce of 300ms prevents accidental double-clicks
- [ ] If user submits while previous request is in-flight, new message is queued
- [ ] Queued messages are sent sequentially after current response completes
- [ ] Visual indicator shows "Sending..." state
- [ ] Queue is cleared on error (user can retry manually)
- [ ] Maximum queue depth of 3 messages (additional submissions show warning)

**Test Cases**:
- [ ] Rapid double-click only sends one message
- [ ] Second message sent while streaming is queued
- [ ] Queued message sends after first response completes
- [ ] Error clears queue and re-enables input

**Code Pattern**:
```typescript
const [messageQueue, setMessageQueue] = useState<string[]>([]);
const [isSending, setIsSending] = useState(false);

const sendMessage = useCallback(async (query: string) => {
  if (isSending) {
    if (messageQueue.length < 3) {
      setMessageQueue(prev => [...prev, query]);
    }
    return;
  }
  setIsSending(true);
  // ... existing send logic
}, [isSending, messageQueue]);

// Process queue after response
useEffect(() => {
  if (!isSending && messageQueue.length > 0) {
    const [next, ...rest] = messageQueue;
    setMessageQueue(rest);
    sendMessage(next);
  }
}, [isSending, messageQueue]);
```

---

## Phase 4: Core Components

### T013: Create ChatContext Provider ✅
**Priority**: P1 | **Dependencies**: T012b | **Status**: COMPLETED

**Description**: React context for global chat state access.

**Files**:
- Create: `src/contexts/ChatContext.tsx`

**Acceptance Criteria**:
- [ ] Creates ChatContext with useChat hook values
- [ ] ChatProvider component wraps children
- [ ] useChatContext() hook for consuming
- [ ] Throws error if used outside provider
- [ ] Exports ChatProvider and useChatContext

---

### T014: Create ChatToggle Component (FAB) ✅
**Priority**: P1 | **Dependencies**: T013 | **Status**: COMPLETED

**Description**: Floating action button to open/close chat panel.

**Files**:
- Create: `src/components/Chat/ChatToggle.tsx`

**Acceptance Criteria**:
- [ ] Fixed position: bottom-right corner
- [ ] Chat bubble icon (or message icon)
- [ ] Badge showing unread count (optional)
- [ ] onClick toggles chat panel open/closed
- [ ] Accessible: aria-label, keyboard focusable
- [ ] Mobile-friendly touch target (48x48px minimum)

---

### T015: Create ChatPanel Component ✅
**Priority**: P1 | **Dependencies**: T013 | **Status**: COMPLETED

**Description**: Expandable panel containing the chat interface.

**Files**:
- Create: `src/components/Chat/ChatPanel.tsx`

**Acceptance Criteria**:
- [ ] Fixed position overlay or slide-in panel
- [ ] Header with title "Ask the Book" + close button
- [ ] "New Conversation" button in header
- [ ] Contains MessageList, MessageInput
- [ ] Responsive: full-screen on mobile (375px), panel on desktop
- [ ] Close on Escape key
- [ ] Smooth open/close animation (CSS transition)

---

### T016: Create MessageList Component ✅
**Priority**: P1 | **Dependencies**: T007 | **Status**: COMPLETED

**Description**: Scrollable list of chat messages.

**Files**:
- Create: `src/components/Chat/MessageList.tsx`

**Acceptance Criteria**:
- [ ] Renders array of messages
- [ ] User messages styled differently (right-aligned, different color)
- [ ] Assistant messages with streaming indicator when loading
- [ ] Auto-scrolls to bottom on new message
- [ ] Empty state: "Ask a question to get started"
- [ ] Renders CitationList for assistant messages with sources

---

### T017: Create MessageInput Component ✅
**Priority**: P1 | **Dependencies**: T013 | **Status**: COMPLETED

**Description**: Text input with send button for composing messages.

**Files**:
- Create: `src/components/Chat/MessageInput.tsx`

**Acceptance Criteria**:
- [ ] Textarea for multiline input
- [ ] Send button (disabled when empty or loading)
- [ ] Enter to send (Shift+Enter for newline)
- [ ] Placeholder text
- [ ] Shows loading state during streaming
- [ ] Max length indicator (2000 chars)
- [ ] Accessible: label, aria attributes

---

### T018: Create CitationList Component ✅
**Priority**: P1 | **Dependencies**: T007 | **Status**: COMPLETED

**Description**: Renders clickable citation links below assistant messages.

**Files**:
- Create: `src/components/Chat/CitationList.tsx`

**Acceptance Criteria**:
- [ ] Renders list of Citation objects
- [ ] Shows page title and section heading
- [ ] Internal links use SPA navigation (no full reload)
- [ ] External links open in new tab
- [ ] "From your selection" for selected_text mode citations
- [ ] Shows relevance score as visual indicator (optional)
- [ ] Expandable snippet preview

---

## Phase 5: Selected Text Features

### T019: Create SelectedTextBadge Component ✅
**Priority**: P2 | **Dependencies**: T011, T013 | **Status**: COMPLETED

**Description**: Badge in chat panel showing captured selected text.

**Files**:
- Create: `src/components/Chat/SelectedTextBadge.tsx`

**Acceptance Criteria**:
- [ ] Shows when selectedText is non-null
- [ ] Displays truncated preview (first 100 chars + "...")
- [ ] "Clear" button to remove selection
- [ ] Visual indicator that next question uses this context
- [ ] Collapsible to show full text

---

### T020: Create SelectionTooltip Component ✅
**Priority**: P1 | **Dependencies**: T011, T013 | **Status**: COMPLETED

**Description**: Floating "Ask about this" button that appears near text selection.

**Files**:
- Create: `src/components/Chat/SelectionTooltip.tsx`

**Acceptance Criteria**:
- [ ] Appears when selectedText is non-null
- [ ] Positioned using selectionRect from useSelectedText
- [ ] Above selection (below if near top of viewport)
- [ ] Uses `position: fixed` with calculated top/left
- [ ] Button text: "Ask about this" (or icon + text)
- [ ] Clicking opens chat panel with selection loaded
- [ ] Hides on click outside, scroll, Escape key
- [ ] Smooth fade-in animation
- [ ] Z-index above page content but below chat panel

**Test Cases**:
- [ ] Selecting text shows tooltip
- [ ] Clicking tooltip opens chat
- [ ] Scrolling hides tooltip
- [ ] Escape key hides tooltip

---

## Phase 6: Error Handling

### T021: Create ErrorFallback Component ✅
**Priority**: P2 | **Dependencies**: None | **Status**: COMPLETED

**Description**: Fallback UI for error boundary.

**Files**:
- Create: `src/components/Chat/ErrorFallback.tsx`

**Acceptance Criteria**:
- [ ] Friendly error message (not technical)
- [ ] "Try Again" button to reset
- [ ] Logs error to console (console.error)
- [ ] Does not expose stack traces to users
- [ ] Different messages for different error codes

---

### T022: Integrate Error Boundary ✅
**Priority**: P2 | **Dependencies**: T021, T015 | **Status**: COMPLETED

**Description**: Wrap chat components in React Error Boundary.

**Files**:
- Modify: `src/components/Chat/ChatPanel.tsx`

**Acceptance Criteria**:
- [ ] Uses react-error-boundary or custom ErrorBoundary
- [ ] Catches render errors in chat components
- [ ] Shows ErrorFallback on error
- [ ] Recovery via resetErrorBoundary

---

## Phase 7: Integration & Assembly

### T023: Create ChatWidget Container ✅
**Priority**: P1 | **Dependencies**: T014, T015, T019, T020 | **Status**: COMPLETED

**Description**: Main container assembling all chat components.

**Files**:
- Create: `src/components/Chat/ChatWidget.tsx`
- Create: `src/components/Chat/index.ts` (barrel export)

**Acceptance Criteria**:
- [ ] Renders ChatToggle (always visible)
- [ ] Renders ChatPanel (when open)
- [ ] Renders SelectionTooltip (when text selected)
- [ ] Manages open/closed state
- [ ] Exports from index.ts

---

### T024: Swizzle Root Component ✅
**Priority**: P1 | **Dependencies**: T013, T023 | **Status**: COMPLETED

**Description**: Swizzle Docusaurus Root to inject chat widget globally.

**Files**:
- Create: `src/theme/Root.tsx`

**Command**:
```bash
npx docusaurus swizzle @docusaurus/theme-classic Root --wrap
```

**Acceptance Criteria**:
- [ ] Root.tsx created via swizzle command
- [ ] Wraps children with ChatContext.Provider
- [ ] Wraps with SelectedTextContext.Provider
- [ ] Renders ChatWidget after children
- [ ] Chat available on all pages
- [ ] No hydration errors

---

### T025: Create Chat CSS Module ✅
**Priority**: P2 | **Dependencies**: T014, T015, T016, T017, T020 | **Status**: COMPLETED

**Description**: Styles for all chat components.

**Files**:
- Create: `src/css/chat.module.css`

**Acceptance Criteria**:
- [ ] FAB button styles (fixed position, circular, shadow)
- [ ] Panel styles (slide-in, max-width, responsive)
- [ ] Message bubble styles (user vs assistant)
- [ ] Citation list styles
- [ ] Selection tooltip styles
- [ ] Loading/streaming indicator styles
- [ ] Mobile responsive (375px breakpoint)
- [ ] Respects Docusaurus theme (light/dark mode)
- [ ] Bundle size < 10KB

---

## Phase 8: E2E Verification

### T026: E2E Test - Chat Widget Renders ✅
**Priority**: P1 | **Dependencies**: T024 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify chat FAB appears on all pages.

**Test Steps**:
1. Navigate to homepage
2. Verify floating button visible (bottom-right)
3. Navigate to different page
4. Verify button still visible

**Acceptance Criteria**:
- [ ] FAB visible on homepage
- [ ] FAB visible on documentation pages
- [ ] FAB visible after navigation

---

### T027: E2E Test - Chat Opens and Closes ✅
**Priority**: P1 | **Dependencies**: T026 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify chat panel opens/closes correctly.

**Test Steps**:
1. Click FAB
2. Verify panel opens
3. Click close button
4. Verify panel closes
5. Press Escape
6. Verify panel closes

**Acceptance Criteria**:
- [ ] Panel opens on FAB click
- [ ] Panel closes on close button
- [ ] Panel closes on Escape key

---

### T028: E2E Test - Message Streaming ✅
**Priority**: P1 | **Dependencies**: T027 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify messages send and stream correctly.

**Test Steps**:
1. Open chat
2. Type "What is ROS 2?"
3. Click send
4. Observe response streaming

**Acceptance Criteria**:
- [ ] User message appears immediately
- [ ] Loading indicator shows
- [ ] Response streams progressively (not all at once)
- [ ] Citations appear after response
- [ ] Loading indicator hides when complete

---

### T029: E2E Test - No CORS Errors ✅
**Priority**: P1 | **Dependencies**: T001, T028 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify no CORS errors in browser console.

**Test Steps**:
1. Open browser DevTools Console
2. Send a chat message
3. Check for CORS errors

**Acceptance Criteria**:
- [ ] No "Access-Control-Allow-Origin" errors
- [ ] No "CORS policy" errors
- [ ] Network requests succeed (200/streaming)

---

### T030: E2E Test - Citation Navigation ✅
**Priority**: P1 | **Dependencies**: T028 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify citation links work correctly.

**Test Steps**:
1. Get response with citations
2. Click internal citation link
3. Verify SPA navigation (no full reload)
4. Verify chat stays open

**Acceptance Criteria**:
- [ ] Internal link navigates without reload
- [ ] Chat panel remains open
- [ ] Correct page loads

---

### T031: E2E Test - Selected Text Mode ✅
**Priority**: P1 | **Dependencies**: T020, T028 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify text selection and "Ask about this" flow.

**Test Steps**:
1. Select text on a book page
2. Verify tooltip appears
3. Click "Ask about this"
4. Verify chat opens with selection badge
5. Ask question
6. Verify response mentions "from your selection"

**Acceptance Criteria**:
- [ ] Tooltip appears on selection
- [ ] Chat opens with selection loaded
- [ ] Selection badge visible in chat
- [ ] Response is grounded in selection only

---

### T032: E2E Test - Session Persistence ✅
**Priority**: P2 | **Dependencies**: T010, T028 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify conversation persists across page reload.

**Test Steps**:
1. Send a message
2. Get response
3. Refresh page
4. Open chat
5. Verify history loaded

**Acceptance Criteria**:
- [ ] Messages reload after refresh
- [ ] Session ID preserved in localStorage
- [ ] Order is correct (oldest first)

---

### T033: E2E Test - New Conversation ✅
**Priority**: P2 | **Dependencies**: T032 | **Status**: READY (see e2e-test-plan.md)

**Description**: Verify "New Conversation" clears state.

**Test Steps**:
1. Have conversation with messages
2. Click "New Conversation"
3. Verify messages cleared
4. Verify new session ID

**Acceptance Criteria**:
- [ ] Messages cleared from UI
- [ ] New session_id in localStorage
- [ ] Can start fresh conversation

---

## Phase 9: Documentation

### T034: Create Usage Documentation ✅
**Priority**: P3 | **Dependencies**: T026-T033 | **Status**: COMPLETED

**Description**: Document chat widget usage for end users.

**Files**:
- Create or update: `docs/chat-assistant.md` (if needed)

**Acceptance Criteria**:
- [ ] How to open/close chat
- [ ] How to ask questions
- [ ] How to use selected text mode
- [ ] How to start new conversation
- [ ] Troubleshooting common issues

---

### T035: Update Environment Setup Guide ✅
**Priority**: P3 | **Dependencies**: T001, T006 | **Status**: COMPLETED

**Description**: Document environment configuration for developers.

**Files**:
- Modify: `specs/007-spec04-chatkit-frontend/quickstart.md`

**Acceptance Criteria**:
- [ ] Backend CORS setup documented
- [ ] Frontend API URL setup documented
- [ ] Production deployment notes
- [ ] Troubleshooting CORS issues

---

## Dependency Graph

```
T001 (CORS) ─────────────────────────────────────┐
T002 (Models) ──► T004 (Endpoint) ───────────────┤
T003 (retry_after) ──────────────────────────────┤
                                                 │
T005 (npm) ──────────────────────────────────────┤
T006 (config) ──► T008 (API) ────────────────────┤
T007 (types) ──┬► T008 (API) ────────────────────┤
               ├► T010 (useSession) ─────────────┤
               ├► T011 (useSelectedText) ────────┤
               └► T016 (MessageList) ────────────┤
                                                 │
T010 + T011 + T008 ──► T012 (useChat) ───────────┤
                                                 │
T012 ──► T013 (Context) ─────────────────────────┤
                                                 │
T013 ──┬► T014 (Toggle) ─────────────────────────┤
       ├► T015 (Panel) ──► T022 (ErrorBoundary) ─┤
       ├► T017 (Input) ──────────────────────────┤
       └► T019 (Badge) ──────────────────────────┤
                                                 │
T007 ──┬► T016 (MessageList) ────────────────────┤
       └► T018 (Citations) ──────────────────────┤
                                                 │
T011 + T013 ──► T020 (Tooltip) ──────────────────┤
                                                 │
T021 (ErrorFallback) ──► T022 (ErrorBoundary) ───┤
                                                 │
T014 + T015 + T019 + T020 ──► T023 (Widget) ─────┤
                                                 │
T013 + T023 ──► T024 (Root) ─────────────────────┤
                                                 │
T014-T020 ──► T025 (CSS) ────────────────────────┤
                                                 │
T024 ──► T026-T033 (E2E Tests) ──────────────────┤
                                                 │
T026-T033 ──► T034-T035 (Docs) ──────────────────┘
```

---

## Critical Path

The longest dependency chain:

```
T007 → T008 → T012 → T013 → T023 → T024 → T026
(types)→(API)→(hook)→(ctx)→(widget)→(Root)→(E2E)
```

**Estimated Critical Path**: 7 tasks

---

## Implementation Priority

### P1 (Must Have) - 24 tasks
Core functionality required for MVP:
- Backend: T001, T002, T003, T004
- Foundation: T005, T006, T007, T008
- Hooks: T010, T011, T012, T012b
- Components: T013, T014, T015, T016, T017, T018, T020, T023, T024
- E2E: T026, T027, T028, T029, T030, T031

### P2 (Should Have) - 7 tasks
Important but not blocking:
- Config: T009
- Components: T019, T021, T022, T025
- E2E: T032, T033

### P3 (Nice to Have) - 2 tasks
Documentation and polish:
- Docs: T034, T035
