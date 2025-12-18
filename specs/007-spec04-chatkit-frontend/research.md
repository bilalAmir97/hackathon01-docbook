# Research: Docusaurus ChatKit Frontend + Integration

**Feature**: 007-spec04-chatkit-frontend
**Date**: 2025-12-17
**Status**: Complete

---

## Research Summary

All technical decisions have been resolved through clarification and existing codebase analysis.

---

## Resolved Decisions

### 1. Chat UI Library Selection

**Decision**: `@chatui/core` (Alibaba ChatUI)

**Rationale**:
- Lightweight (~30KB gzipped)
- Actively maintained (last update within 3 months)
- Good React 18 compatibility
- Customizable theming via CSS variables
- Built-in message bubble components

**Alternatives Considered**:
- `stream-chat-react`: Too heavy, requires paid account
- Custom implementation: Higher dev effort, reinventing the wheel
- `react-chat-widget`: Limited customization

**Source**: User clarification (Session 2025-12-17)

---

### 2. Docusaurus Integration Method

**Decision**: Swizzled Root theme component (`src/theme/Root.tsx`)

**Rationale**:
- Standard Docusaurus pattern for global components
- Widget persists across SPA page navigations
- Chat state preserved without complex state management
- No per-page MDX embedding required

**Alternatives Considered**:
- Custom plugin: Higher complexity, same outcome
- MDX embed per page: Manual work, state lost on navigation
- Custom page: Loses book context

**Source**: User clarification (Session 2025-12-17)

---

### 3. Session ID Storage Strategy

**Decision**: localStorage with UUID v4, user-controlled expiration

**Rationale**:
- Simpler than cookies for cross-origin scenarios
- Works without HTTPS in development
- User controls session lifecycle via "New Conversation"
- Backend handles 30-day retention cleanup

**Alternatives Considered**:
- Cookies: CSRF complexity, cross-origin issues
- sessionStorage: Session lost on tab close (bad UX)
- Never expire: Storage growth concerns

**Source**: User clarification (Session 2025-12-17)

---

### 4. Citation Navigation Behavior

**Decision**: Context-aware navigation (internal SPA, external new tab)

**Rationale**:
- Internal book links use Docusaurus router (chat stays open)
- External links open in new tab (expected behavior)
- Best UX for exploring sources without losing chat context

**Alternatives Considered**:
- Always same tab: Loses chat context
- Always new tab: Disconnected experience for internal links
- Modal preview: Higher complexity

**Source**: User clarification (Session 2025-12-17)

---

### 5. Frontend Error Logging Strategy

**Decision**: React Error Boundary + console.error

**Rationale**:
- Standard React pattern for graceful degradation
- Developer debugging via browser console
- No external error tracking (aligns with "analytics out of scope")
- User sees friendly fallback, not stack trace

**Alternatives Considered**:
- No logging: Hinders debugging
- Backend error endpoint: Adds complexity
- External service (Sentry): Out of scope

**Source**: User clarification (Session 2025-12-17)

---

## Existing Backend Analysis

### Backend Structure (Spec-3)

Analyzed `backend/` directory:

| File | Purpose | Modification Needed |
|------|---------|---------------------|
| `api.py` | FastAPI endpoints | Yes - CORS + /conversations |
| `database.py` | Neon Postgres functions | No - already has `get_conversation_history()` |
| `api_models.py` | Pydantic models | Minor - add response model |
| `agent.py` | OpenAI Agents SDK | No changes |
| `config.py` | Environment config | No changes |

### CORS Current State (SECURITY ISSUE)

**Location**: `backend/api.py:78-84`

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # ← INSECURE
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Issue**: Wildcard origin allows any website to call the API.

**Resolution**: Change to environment-based allowlist:
- `http://localhost:3000` (dev)
- `http://localhost:3001` (dev alternate)
- Production domain (from `ALLOWED_ORIGINS` env var)

### Database Functions (No Changes Needed)

**Location**: `backend/database.py:205-267`

The `get_conversation_history()` function already exists and returns:
- `query`, `response`, `mode`, `sources`, `created_at`
- Ordered oldest to newest
- Enforces `MAX_CONVERSATION_HISTORY = 10`

Only need to expose this via new API endpoint.

---

## Technology Compatibility

### React Version Check

Docusaurus 3.x uses React 18. `@chatui/core` supports:
- React 17+ (peer dependency)
- React 18 confirmed working

**Compatibility**: ✅ Verified

### SSE Browser Support

EventSource API supported in all target browsers:
- Chrome 6+
- Firefox 6+
- Safari 5+
- Edge 79+

**Compatibility**: ✅ Verified

### Docusaurus Swizzling

Root component swizzling supported since Docusaurus 2.0:
- `npx docusaurus swizzle @docusaurus/theme-classic Root --wrap`
- Creates `src/theme/Root.tsx`

**Compatibility**: ✅ Verified

---

## Open Questions

None remaining. All decisions resolved via user clarification.
