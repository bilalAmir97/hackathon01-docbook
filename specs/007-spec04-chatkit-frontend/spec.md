# Feature Specification: Docusaurus ChatKit Frontend + Integration

**Feature Branch**: `007-spec04-chatkit-frontend`
**Created**: 2025-12-17
**Status**: Draft
**Input**: User description: "Build the RAG chatbot frontend inside the Docusaurus book using ChatKit, then integrate it with the existing FastAPI + OpenAI Agents backend, including Neon Postgres chat history (database.py) and browser-safe CORS."

---

## Overview

This feature builds the frontend chatbot UI for the RAG-powered book assistant, embedded within the Docusaurus documentation site. It connects to the existing FastAPI + OpenAI Agents backend (Spec-3) and enables readers to ask questions about the book content directly from any page.

**Target Users**:
- Readers of the published book who want to ask questions about the content
- Developers maintaining the full-stack RAG experience

**Core Value**: Enable readers to interactively explore book content through natural language Q&A, with the ability to highlight specific text for focused questions, while maintaining conversation history across sessions.

---

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask a General Question About Book Content (Priority: P1)

A reader is browsing the book and wants to ask a general question about topics covered. They open the chat widget, type their question, and receive an answer with clickable citations pointing to relevant sections of the book.

**Why this priority**: This is the core value proposition - enabling Q&A over book content with verifiable sources.

**Independent Test**: Can be fully tested by opening the chat widget, asking a question like "What is ROS 2?", and verifying the response appears with source citations.

**Acceptance Scenarios**:

1. **Given** a reader is on any page of the book, **When** they click the chat widget icon, **Then** a chat interface opens allowing them to type a question.
2. **Given** a reader submits a question, **When** the backend processes it, **Then** the response renders progressively (streaming) with typing indicator during generation.
3. **Given** the response includes citations, **When** rendered, **Then** each citation displays source title and clicking it navigates to that page/section.
4. **Given** the backend is unavailable, **When** the reader submits a question, **Then** an error message appears with retry option (not a browser error).

---

### User Story 2 - Ask About Selected Text (Priority: P1)

A reader highlights a specific passage in the book and wants to ask a question about just that selected text. The chat interface captures the selection and sends it along with the question, restricting the answer to only the highlighted content.

**Why this priority**: Critical for focused Q&A where users want explanations scoped to specific content they're reading.

**Independent Test**: Can be tested by selecting text on a page, clicking a contextual button, and verifying the question is scoped to that selection.

**Acceptance Scenarios**:

1. **Given** a reader selects text on the book page, **When** they click the "Ask about this" button (or similar trigger), **Then** the chat opens with the selected text displayed as context.
2. **Given** selected text is provided, **When** the reader submits a question, **Then** the request includes the selected text and mode is set to "selected_text".
3. **Given** selected-text mode is active, **When** the answer cannot be found in the selection, **Then** the response clearly indicates this and offers to search the full documentation.
4. **Given** no text is selected but user triggers "Ask about selection", **When** the action is triggered, **Then** a helpful tooltip or message prompts them to select text first.

---

### User Story 3 - Continue a Conversation Across Sessions (Priority: P2)

A reader returns to the book later and wants to continue their previous conversation. The chat interface retrieves their conversation history from the backend using a persistent session ID.

**Why this priority**: Improves user experience by enabling multi-turn conversations and persistence across visits.

**Independent Test**: Can be tested by having a conversation, closing the browser, reopening, and verifying previous messages are restored.

**Acceptance Scenarios**:

1. **Given** a reader has previous conversations, **When** they open the chat widget, **Then** recent conversation history is loaded and displayed.
2. **Given** session_id is stored locally, **When** the reader sends a new message, **Then** it includes the session_id so backend groups conversations correctly.
3. **Given** conversation history exceeds display limit, **When** rendered, **Then** older messages are scrollable and newest are visible by default.
4. **Given** no previous session exists, **When** the reader first opens chat, **Then** a new session_id is generated and stored locally.

---

### User Story 4 - View and Navigate Citations (Priority: P2)

A reader receives an answer with citations and wants to verify the information by visiting the source pages. They click on citation links which navigate them to the referenced book sections.

**Why this priority**: Citations are essential for trust and verification, enabling readers to confirm information accuracy.

**Independent Test**: Can be tested by receiving a response with citations and clicking each one to verify navigation works.

**Acceptance Scenarios**:

1. **Given** a response contains citations, **When** rendered, **Then** each citation shows page title and is visually distinct (e.g., numbered footnotes or inline chips).
2. **Given** a reader clicks a citation, **When** the target page is in the current book, **Then** the browser navigates to that page and scrolls to the relevant section if anchor is available.
3. **Given** a citation includes a text snippet, **When** displayed, **Then** the snippet is shown in a tooltip or expandable preview.
4. **Given** multiple citations reference the same page, **When** rendered, **Then** they are visually grouped or deduplicated appropriately.

---

### User Story 5 - Mobile-Responsive Chat Experience (Priority: P3)

A reader accesses the book on a mobile device and uses the chat feature. The chat interface adapts to smaller screens with appropriate layout and touch interactions.

**Why this priority**: Mobile accessibility ensures broader reach but is secondary to core functionality.

**Independent Test**: Can be tested by accessing the chat on mobile viewport sizes and verifying usability.

**Acceptance Scenarios**:

1. **Given** a reader is on a mobile device, **When** they open the chat widget, **Then** it expands to a fullscreen or appropriately sized modal.
2. **Given** mobile viewport, **When** the keyboard opens for typing, **Then** the chat input remains visible and accessible.
3. **Given** mobile viewport, **When** viewing citations, **Then** they are tappable with adequate touch targets.

---

### Edge Cases

| Scenario | Expected Behavior |
|----------|-------------------|
| Network disconnected during request | Show "Connection lost" message with retry button |
| Backend returns 503 (service unavailable) | Show user-friendly error: "Service temporarily unavailable, please try again" |
| Backend returns 429 (rate limited) | Show "Please wait X seconds before sending another message" |
| Very long response (>2000 words) | Response renders progressively; no truncation |
| Selected text exceeds max length (>10000 chars) | Show warning asking user to select shorter passage |
| Empty message submitted | Submit button disabled when input is empty |
| Chat widget open while navigating pages | Chat state persists during navigation |
| Multiple rapid message submissions | Queue messages; disable submit until previous completes |
| Session storage unavailable (private mode) | Generate ephemeral session; warn about no persistence |
| CORS error from backend | Show user-friendly connection error, not browser CORS message |

---

## Requirements *(mandatory)*

### Functional Requirements

**Chat Widget Core**
- **FR-001**: System MUST display a chat widget icon/button accessible from all book pages.
- **FR-002**: System MUST render a chat interface when the widget is activated, including message history, input field, and send button.
- **FR-003**: System MUST send chat requests to the backend `/chat` or `/chat/stream` endpoint.
- **FR-004**: System MUST render responses progressively as streaming data arrives (SSE).
- **FR-005**: System MUST display a typing/loading indicator while awaiting response.

**Selected Text Capture**
- **FR-006**: System MUST capture user-selected text from the book page when a designated action is triggered (button, keyboard shortcut, or context menu).
- **FR-007**: System MUST display captured selected text in the chat interface as visible context.
- **FR-008**: System MUST include selected text in the request payload with `mode: "selected_text"`.
- **FR-009**: System MUST allow users to clear the selected text context and switch back to general mode.

**Session Management**
- **FR-010**: System MUST generate a unique session_id (UUID v4) on first use and persist it in localStorage (no automatic expiration).
- **FR-011**: System MUST include session_id in all chat requests to enable conversation grouping.
- **FR-012**: System MUST load and display conversation history from the backend on chat widget open.
- **FR-013**: System MUST support a "New Conversation" action that generates a new session_id and clears the previous one from localStorage.

**Citation Display**
- **FR-014**: System MUST render source citations included in backend responses.
- **FR-015**: System MUST make citations clickable; internal book links use Docusaurus SPA navigation (scroll to section, chat stays open), external links open in new tab.
- **FR-016**: System MUST display citation metadata (page title, section heading) visually.
- **FR-017**: System MUST handle selected-text citations differently (display "From your selection" instead of URL).

**Error Handling**
- **FR-018**: System MUST display user-friendly error messages for all backend errors (no raw HTTP errors or CORS messages).
- **FR-019**: System MUST provide retry capability for transient errors (503, network failures).
- **FR-020**: System MUST handle rate limiting (429) by displaying wait time and disabling input temporarily.

**Backend Integration (CORS & API)**
- **FR-021**: Backend MUST configure CORS middleware to allow requests from frontend origins (local dev + production URLs).
- **FR-022**: Backend MUST accept `Origin` headers from configured allowlist without errors.
- **FR-023**: System MUST work correctly in both local development (e.g., `http://localhost:3000`) and deployed environments.

**History Retrieval**
- **FR-024**: Backend MUST expose a `/conversations/{session_id}` or similar endpoint to retrieve conversation history.
- **FR-025**: System MUST call history endpoint on chat widget initialization to populate previous messages.

### API Contracts (Frontend → Backend)

#### Request to `/chat` (sync) or `/chat/stream` (SSE)

```
POST /chat
POST /chat/stream
Content-Type: application/json

{
  "query": string,                    // User's question (required)
  "session_id": string | null,        // UUID for conversation grouping
  "mode": "general" | "selected_text", // Query mode (default: "general")
  "selected_text": string | null      // Required if mode="selected_text"
}
```

**Note**: Use `/chat` for synchronous full response, `/chat/stream` for Server-Sent Events streaming. The endpoint determines response format; no `stream` field needed in request body.

#### Request to `/conversations/{session_id}`

```
GET /conversations/{session_id}?limit=20

Response:
{
  "session_id": string,
  "messages": [
    {
      "id": string,
      "role": "user" | "assistant",
      "content": string,
      "sources": SourceCitation[] | null,
      "created_at": string (ISO 8601)
    }
  ]
}
```

#### CORS Configuration (Backend)

```
Allowed Origins (environment-aware):
- Local development: http://localhost:3000, http://localhost:3001, http://127.0.0.1:3000
- Production: Deployed Docusaurus URL (e.g., https://yourdomain.com)
- Configurable via ALLOWED_ORIGINS environment variable

Allowed Methods: GET, POST, OPTIONS
Allowed Headers: Content-Type, Authorization
```

### Key Entities

- **ChatWidget**: The UI component embedded in Docusaurus that provides the chat interface.
- **Message**: A single exchange in the conversation (user question or assistant response).
- **SelectedTextContext**: The captured text from the book page provided as context for the question.
- **SessionId**: A UUID identifying the conversation session, stored in browser local storage.
- **Citation**: A reference to a book section included in the assistant's response.

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

| ID | Criterion | Threshold | Measurement Method |
|----|-----------|-----------|-------------------|
| SC-001 | Chat widget loads on all book pages | 100% of pages | Automated test visiting each page |
| SC-002 | User can submit question and receive response | Within 5 seconds (including streaming start) | Manual testing + timing |
| SC-003 | Selected text capture works | 100% success on Chrome, Firefox, Safari | Cross-browser manual testing |
| SC-004 | Citations render and are clickable | 100% of responses with citations | Manual verification |
| SC-005 | Conversation history persists across sessions | Correctly restored on page reload | Manual testing with local storage check |
| SC-006 | No CORS errors in browser console | 0 CORS errors | Browser dev tools inspection |
| SC-007 | Error messages are user-friendly | No raw HTTP errors displayed | Manual error injection testing |
| SC-008 | Mobile responsive layout | Usable on 375px viewport | Visual testing on mobile viewport |
| SC-009 | Streaming response renders progressively | Text appears in chunks, not all at once | Visual observation during testing |
| SC-010 | Chat state persists during page navigation | Chat open state and messages maintained | Navigate between pages with chat open |

---

## Constraints Catalog

### Technical Constraints

| Constraint | Value | Source |
|------------|-------|--------|
| Backend API | FastAPI (Spec-3) | Spec-3 |
| Backend streaming | SSE (Server-Sent Events) | Spec-3 |
| Session ID format | UUID v4 | Spec-3 FR-015 |
| Selected text max length | 10,000 characters | Spec-3 |
| Query max length | 2,000 characters | Spec-3 |
| Conversation history endpoint | New requirement for Spec-4 | This spec |

### Frontend Constraints

| Constraint | Value |
|------------|-------|
| Framework | React (Docusaurus uses React) |
| Chat library | `@chatui/core` (Alibaba ChatUI) |
| Styling | Compatible with Docusaurus theme (CSS modules or styled-components) |
| Browser support | Modern browsers (Chrome, Firefox, Safari, Edge - last 2 versions) |
| Storage | localStorage for session_id persistence |

### Backend Constraints (CORS additions to Spec-3)

| Constraint | Value |
|------------|-------|
| CORS middleware | FastAPI CORSMiddleware |
| Origin allowlist | Environment-configurable |
| Credentials | Allowed (for potential future auth) |
| Preflight caching | Max 600 seconds |

---

## Assumptions

- Spec-3 (FastAPI + OpenAI Agents backend) is deployed and accessible.
- Spec-3 backend can be extended to add CORS configuration and conversation history endpoint.
- Docusaurus site is built with React 17+ or 18.
- `@chatui/core` (Alibaba ChatUI) provides necessary UI primitives for chat interface.
- Backend will be hosted at a known URL (configured via environment variable in frontend).
- Users have modern browsers with JavaScript enabled.
- Local storage is available (fallback handled for private browsing).

---

## Out of Scope

- User authentication and login (chat is anonymous with session-based history)
- Admin dashboard for viewing all conversations
- Analytics and usage tracking on frontend
- Offline support / service workers
- Voice input/output
- Rich media in responses (images, videos)
- Custom chat themes beyond Docusaurus compatibility
- Rate limiting UI beyond displaying backend 429 errors

---

## Dependencies

| Dependency | Type | Description |
|------------|------|-------------|
| Spec-3 (006-rag-agent-api) | Feature | FastAPI backend with /chat, /chat/stream endpoints |
| Docusaurus | Platform | Documentation site framework where chat is embedded |
| `@chatui/core` | Library | Alibaba ChatUI - React components for chat UI |
| React | Library | UI framework (bundled with Docusaurus) |
| Spec-3 Backend Modifications | Feature | CORS config + /conversations endpoint |

---

## Environment Variables (Frontend)

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `REACT_APP_API_URL` or `DOCUSAURUS_API_URL` | Yes | Backend API base URL | `https://api.example.com` |

---

## Environment Variables (Backend - CORS additions)

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `ALLOWED_ORIGINS` | Yes | Comma-separated list of allowed origins | `http://localhost:3000,https://yourdomain.com` |
| `CORS_ALLOW_CREDENTIALS` | No | Allow credentials (default: true) | `true` |

---

## Implementation Scope

### Frontend (Docusaurus/React)

**Integration Method**: Swizzled Root theme component (`src/theme/Root.tsx`) for global chat widget injection

1. **ChatWidget Component**: Floating button + expandable chat panel (using `@chatui/core`)
2. **MessageList Component**: Renders conversation history with user/assistant bubbles
3. **MessageInput Component**: Text input with send button, disabled states
4. **CitationDisplay Component**: Renders clickable source citations
5. **SelectedTextCapture**: Hook/utility to capture text selection from page
6. **SessionManager**: Handles session_id generation, storage, retrieval
7. **APIClient**: Handles backend communication including SSE streaming
8. **ErrorBoundary**: React Error Boundary that catches component errors, displays user-friendly fallback, and logs to console.error for debugging

### Backend Additions (Spec-3 modifications)

1. **CORS Middleware Configuration**: Add to FastAPI app with environment-aware origin allowlist
2. **Conversations Endpoint**: `GET /conversations/{session_id}` to retrieve chat history
3. **Request Schema Update**: Ensure `session_id` and `selected_text` fields are properly handled

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| CORS misconfiguration | Frontend cannot communicate with backend | Thorough testing in both dev and production environments; environment-specific allowlists |
| ChatKit incompatibility | UI library may not integrate smoothly with Docusaurus | Evaluate alternatives early; fallback to custom React components if needed |
| SSE streaming complexity | Browser EventSource API may have limitations | Test cross-browser; implement fallback to polling if needed |
| Session ID collisions | Unlikely but possible | Use crypto.randomUUID() which is collision-resistant |
| Large conversation history | Performance issues loading many messages | Implement pagination on history endpoint; lazy load older messages |
| Selected text not captured correctly | Cross-browser Selection API differences | Test thoroughly on all target browsers; use established polyfills if needed |

---

## Clarifications

### Session 2025-12-17

- Q: Which React chat UI library should be used for the ChatWidget implementation? → A: `@chatui/core` (Alibaba ChatUI) - lightweight, customizable, actively maintained
- Q: How should the chat widget be integrated into Docusaurus? → A: Swizzled theme component (Root) - global injection, persists across SPA navigation
- Q: When should a chat session expire and require a new session_id? → A: On explicit "New Conversation" action only (user-controlled); backend handles 30-day retention cleanup
- Q: What should happen when a user clicks a citation link? → A: Internal book links use Docusaurus SPA navigation (scroll to section, chat stays open); external links open in new tab
- Q: How should frontend errors be logged? → A: React Error Boundary for graceful degradation + console.error for developer debugging; no external error tracking service

---
