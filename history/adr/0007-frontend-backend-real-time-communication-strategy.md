# ADR-0007: Frontend-Backend Real-Time Communication Strategy

- **Status:** Accepted
- **Date:** 2025-12-17
- **Feature:** 007-spec04-chatkit-frontend
- **Context:** The Spec-4 ChatKit frontend needs to receive streaming LLM responses from the FastAPI backend in real-time. The RAG agent generates answers progressively, and users expect to see text appear word-by-word rather than waiting for the complete response. This decision impacts user experience, browser compatibility, error handling complexity, and infrastructure requirements.

## Decision

**Use Server-Sent Events (SSE) for real-time streaming from backend to frontend.**

This decision clusters the following related choices:

- **Transport Protocol**: SSE via `fetch()` + `ReadableStream` (not native EventSource due to POST requirement)
- **Data Format**: Newline-delimited JSON (`data: {...}\n\n`)
- **Event Types**: `chunk`, `sources`, `done`, `error`
- **Error Recovery**: Manual retry with exponential backoff (since not using native EventSource)
- **Backend Implementation**: FastAPI `StreamingResponse` with `text/event-stream` content type

### Implementation Pattern

**Note**: Native `EventSource` only supports GET requests. Since `/chat/stream` requires POST (to send query payload), we use `fetch()` with `ReadableStream` to manually parse SSE events.

```typescript
// Frontend: fetch() + ReadableStream for SSE-over-POST
const response = await fetch('/chat/stream', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ query, session_id, mode, selected_text }),
});

const reader = response.body.getReader();
const decoder = new TextDecoder();
let buffer = '';

while (true) {
  const { done, value } = await reader.read();
  if (done) break;

  buffer += decoder.decode(value, { stream: true });
  const lines = buffer.split('\n\n');
  buffer = lines.pop() || '';

  for (const line of lines) {
    if (line.startsWith('data: ')) {
      const event = JSON.parse(line.slice(6));
      if (event.type === 'chunk') appendText(event.content);
      if (event.type === 'sources') setCitations(event.sources);
      if (event.type === 'done') setComplete();
    }
  }
}
```

**Full implementation details**: See `plan.md` ADR-001 section for complete code including retry logic.

```python
# Backend: FastAPI SSE
async def event_generator():
    async for chunk in run_agent_streamed(...):
        yield f"data: {json.dumps(chunk)}\n\n"

return StreamingResponse(event_generator(), media_type="text/event-stream")
```

## Consequences

### Positive

- **Simplicity**: SSE is purpose-built for unidirectional server→client streaming; no handshake negotiation or ping/pong frames
- **Native Browser Support**: EventSource API available in all target browsers (Chrome, Firefox, Safari, Edge) with built-in reconnection on disconnect
- **HTTP Compatibility**: Works through HTTP/2 multiplexing, proxies, and CDNs without special configuration
- **Existing Implementation**: Spec-3 backend already implements SSE in `/chat/stream` endpoint; no backend changes needed for streaming protocol
- **Debugging**: Standard HTTP traffic visible in browser DevTools Network tab; easier to debug than binary WebSocket frames
- **Resource Efficiency**: No persistent connection overhead when chat is idle; connection only active during streaming

### Negative

- **Unidirectional Only**: SSE only supports server→client; if bidirectional communication is needed later, would require additional mechanism
- **POST Limitation**: Native EventSource only supports GET; POST requires custom wrapper (fetch + ReadableStream) or polyfill
- **Connection Limits**: Browsers limit concurrent EventSource connections per domain (~6 in HTTP/1.1); mitigated by HTTP/2 multiplexing
- **No Binary Support**: SSE is text-only; if binary data needed (audio, images), would need base64 encoding or separate endpoint

## Alternatives Considered

### Alternative A: WebSocket (Rejected ❌)

**Description**: Full-duplex bidirectional communication over single TCP connection.

**Pros**:
- Bidirectional (client can send messages without new requests)
- Lower latency for back-and-forth communication
- Binary data support

**Cons**:
- **Overkill for this use case**: Chat requests are discrete; no need for persistent bidirectional channel
- **Complexity**: Requires WebSocket handshake, ping/pong heartbeats, custom reconnection logic
- **Proxy Issues**: Some corporate proxies block WebSocket upgrade; SSE works over standard HTTP
- **Backend Changes**: FastAPI would need Starlette WebSocket setup; Spec-3 already has SSE

**Why Rejected**: Our use case is strictly unidirectional (server streams response to client). WebSocket's complexity and potential proxy issues outweigh benefits we wouldn't use.

### Alternative B: Long Polling (Rejected ❌)

**Description**: Client repeatedly polls server for updates.

**Pros**:
- Works everywhere (just HTTP requests)
- No special server support needed

**Cons**:
- **Higher Latency**: Each poll has request/response overhead
- **Resource Intensive**: Multiple requests per response; higher server load
- **Complex Client Logic**: Must manage polling interval, backoff, deduplication
- **Poor UX**: Visible "jitter" as chunks arrive in batches

**Why Rejected**: Poor user experience and resource efficiency compared to SSE.

### Alternative C: HTTP/2 Server Push (Rejected ❌)

**Description**: Server proactively pushes resources to client.

**Pros**:
- Built into HTTP/2

**Cons**:
- **Wrong Use Case**: Designed for pushing static assets (CSS, JS), not dynamic streaming data
- **Browser Support**: Being deprecated in some contexts
- **No Control**: Client cannot specify what to receive

**Why Rejected**: Not designed for application-level streaming.

## References

- Feature Spec: [specs/007-spec04-chatkit-frontend/spec.md](../../specs/007-spec04-chatkit-frontend/spec.md)
- Implementation Plan: [specs/007-spec04-chatkit-frontend/plan.md](../../specs/007-spec04-chatkit-frontend/plan.md)
- Related ADRs: ADR-0006 (Spec-2/Spec-3 Integration Pattern - backend architecture)
- Existing Implementation: `backend/api.py:326-407` (`/chat/stream` endpoint)
