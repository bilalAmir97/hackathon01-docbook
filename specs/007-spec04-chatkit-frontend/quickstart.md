# Quickstart: Docusaurus ChatKit Frontend + Integration

**Feature**: 007-spec04-chatkit-frontend
**Date**: 2025-12-17

---

## Prerequisites

- Node.js 18+ (for Docusaurus)
- Python 3.11 (for backend)
- Running backend (Spec-3) at `http://localhost:8000`
- Neon Postgres connection configured

---

## Quick Setup

### 1. Backend CORS Configuration

Update `backend/.env`:

```bash
# Add this line
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:3001
```

Restart backend:
```bash
cd backend
uvicorn api:app --reload --port 8000
```

### 2. Frontend Setup

The frontend uses native browser APIs (no additional chat libraries required). The implementation uses:
- `crypto.randomUUID()` for session IDs (with fallback for older browsers)
- Native `fetch()` with `ReadableStream` for SSE streaming
- CSS Modules for styling

Swizzle Root component (if not already done):
```bash
npx docusaurus swizzle @docusaurus/theme-classic Root --wrap
```

This creates `src/theme/Root.tsx`.

### 3. Configure API URL

Update `docusaurus.config.ts`:

```typescript
const config: Config = {
  // ... existing config
  customFields: {
    apiUrl: process.env.DOCUSAURUS_API_URL || 'http://localhost:8000',
  },
};
```

### 4. Start Development

```bash
npm start
```

Visit `http://localhost:3000` - chat widget should appear in bottom-right corner.

---

## Development Workflow

### Test Chat Flow

1. Click chat FAB button (bottom-right)
2. Type: "What is ROS 2?"
3. Verify:
   - Loading indicator shows
   - Response streams progressively
   - Citations appear below answer
   - Clicking citation navigates to page

### Test Selected Text Mode

1. Highlight any text on a book page
2. Click "Ask about this" button
3. Type question about the selection
4. Verify response is grounded in selection only

### Test Session Persistence

1. Have a conversation
2. Refresh the page
3. Open chat - previous messages should load

---

## File Structure After Setup

```
src/
├── theme/
│   └── Root.tsx           # Swizzled (wrap ChatWidget)
├── components/
│   └── Chat/
│       ├── index.ts
│       ├── ChatWidget.tsx
│       ├── ChatToggle.tsx
│       ├── ChatPanel.tsx
│       ├── MessageList.tsx
│       ├── MessageInput.tsx
│       ├── CitationList.tsx
│       └── ErrorFallback.tsx
├── hooks/
│   ├── useChat.ts
│   ├── useSelectedText.ts
│   └── useSession.ts
├── services/
│   └── api.ts
├── contexts/
│   └── ChatContext.tsx
└── css/
    └── chat.module.css
```

---

## Environment Variables

### Frontend

| Variable | Default | Description |
|----------|---------|-------------|
| `DOCUSAURUS_API_URL` | `http://localhost:8000` | Backend API base URL |

### Backend

| Variable | Default | Description |
|----------|---------|-------------|
| `ALLOWED_ORIGINS` | `http://localhost:3000` | Comma-separated CORS origins |

---

## Troubleshooting

### CORS Errors

**Symptom**: Browser console shows CORS errors.

**Fix**:
1. Check `ALLOWED_ORIGINS` in backend `.env` includes your frontend URL
2. Restart backend after changing env vars
3. Check no trailing slashes in origins

### Chat Not Loading History

**Symptom**: Previous messages don't appear on refresh.

**Fix**:
1. Check localStorage has `chat_session_id`
2. Verify backend `/conversations/{session_id}` endpoint works
3. Check database connection in backend logs

### SSE Streaming Not Working

**Symptom**: Response appears all at once, not progressively.

**Fix**:
1. The frontend uses `fetch()` with `ReadableStream` for SSE-over-POST
2. Check response has `Content-Type: text/event-stream`
3. Check for proxy/CDN buffering issues
4. Verify the backend sends `data: {...}\n\n` format

### Error Boundary Shows "Something Went Wrong"

**Symptom**: Chat shows error fallback instead of messages.

**Fix**:
1. Check browser console for specific error
2. Click "Try Again" to reset the error boundary
3. Check that all API responses match expected types

---

## Production Deployment

### Frontend Environment

Create `.env` file in project root:
```bash
# Production API URL
DOCUSAURUS_API_URL=https://api.yourdomain.com
```

### Backend Environment

Update `backend/.env`:
```bash
# Production CORS origins (comma-separated, no trailing slashes)
ALLOWED_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

# Neon PostgreSQL connection
DATABASE_URL=postgresql://user:password@your-neon-host.neon.tech/neondb?sslmode=require
```

### Build and Deploy

```bash
# Build frontend
npm run build

# Deploy to Vercel/Netlify/etc.
npm run deploy
```

---

## Implementation Status

| Phase | Status | Notes |
|-------|--------|-------|
| Backend CORS | ✅ Complete | Environment-based allowlist |
| Backend Conversations API | ✅ Complete | GET /conversations/{session_id} |
| Frontend Types | ✅ Complete | src/types/chat.ts |
| API Service | ✅ Complete | SSE-over-POST streaming |
| Session Management | ✅ Complete | localStorage persistence |
| Chat Components | ✅ Complete | All core components |
| Error Boundary | ✅ Complete | Wrapped in ChatPanel |
| Message Queue | ✅ Complete | Debounce + queue logic |

---

## Next Steps

1. Run `npm start` to test locally
2. Verify all E2E test scenarios pass
3. Deploy to production environment
