/**
 * API service for ChatKit RAG chatbot
 * Handles SSE-over-POST streaming communication with the backend
 */

import type {
  ChatRequest,
  ChatError,
  StreamEvent,
  Citation,
  ApiConfig,
  Message,
} from '../types/chat';

// Default configuration - use localhost in development, production URL otherwise
const DEFAULT_CONFIG: ApiConfig = {
  baseUrl: typeof window !== 'undefined' && window.location.hostname === 'localhost'
    ? 'http://localhost:8000'
    : 'https://bilalamir97-rag-deployment.hf.space',
  timeout: 30000,
};

/**
 * Get the API URL from Docusaurus config or environment
 */
function getApiUrl(): string {
  // Try to get from Docusaurus site config
  if (typeof window !== 'undefined') {
    // @ts-expect-error - Docusaurus global
    const siteConfig = window.__DOCUSAURUS__?.siteConfig;
    if (siteConfig?.customFields?.apiUrl) {
      return siteConfig.customFields.apiUrl as string;
    }
  }
  return DEFAULT_CONFIG.baseUrl;
}

/**
 * Transform snake_case keys to camelCase for Citation objects
 * Backend uses snake_case, frontend uses camelCase
 */
function transformCitation(raw: Record<string, unknown>): Citation {
  return {
    sourceUrl: (raw.source_url as string) || '',
    pageTitle: (raw.page_title as string) || '',
    sectionHeading: (raw.section_heading as string) || '',
    chunkText: (raw.chunk_text as string) || '',
    relevanceScore: typeof raw.relevance_score === 'number' ? raw.relevance_score : 0,
  };
}

/**
 * Parse a single SSE line into a StreamEvent
 */
function parseSSELine(line: string): StreamEvent | null {
  if (!line.startsWith('data: ')) {
    return null;
  }

  const data = line.slice(6).trim();
  if (!data || data === '[DONE]') {
    return null;
  }

  try {
    const parsed = JSON.parse(data);

    // Transform sources if present (snake_case -> camelCase)
    if (parsed.type === 'sources' && Array.isArray(parsed.sources)) {
      parsed.sources = parsed.sources.map(transformCitation);
    }

    return parsed as StreamEvent;
  } catch {
    console.warn('Failed to parse SSE data:', data);
    return null;
  }
}

/**
 * Stream chat response from the backend using SSE-over-POST
 *
 * @param request - The chat request payload
 * @param onChunk - Callback for each content chunk
 * @param onSources - Callback when sources are received
 * @param onDone - Callback when streaming is complete
 * @param onError - Callback on error
 * @param signal - AbortSignal for cancellation
 */
export async function streamChatResponse(
  request: ChatRequest,
  onChunk: (content: string) => void,
  onSources: (sources: Citation[]) => void,
  onDone: (metadata: { query_time_ms: number; chunks_retrieved: number; model: string }) => void,
  onError: (error: ChatError) => void,
  signal?: AbortSignal
): Promise<void> {
  const apiUrl = getApiUrl();
  const endpoint = `${apiUrl}/chat/stream`;

  try {
    const response = await fetch(endpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Accept': 'text/event-stream',
      },
      body: JSON.stringify(request),
      signal,
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw {
        error_code: `HTTP_${response.status}`,
        message: errorData.message || `Request failed with status ${response.status}`,
        retry_after: errorData.retry_after,
      } as ChatError;
    }

    const reader = response.body?.getReader();
    if (!reader) {
      throw {
        error_code: 'NO_STREAM',
        message: 'Response body is not readable',
      } as ChatError;
    }

    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();

      if (done) {
        break;
      }

      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');

      // Keep the last incomplete line in the buffer
      buffer = lines.pop() || '';

      for (const line of lines) {
        const trimmedLine = line.trim();
        if (!trimmedLine) continue;

        const event = parseSSELine(trimmedLine);
        if (!event) continue;

        switch (event.type) {
          case 'chunk':
            onChunk(event.content);
            break;
          case 'sources':
            onSources(event.sources);
            break;
          case 'done':
            onDone(event.metadata);
            break;
          case 'error':
            onError({
              error_code: event.error_code,
              message: event.message,
            });
            break;
        }
      }
    }

    // Process any remaining data in the buffer
    if (buffer.trim()) {
      const event = parseSSELine(buffer.trim());
      if (event) {
        switch (event.type) {
          case 'chunk':
            onChunk(event.content);
            break;
          case 'sources':
            onSources(event.sources);
            break;
          case 'done':
            onDone(event.metadata);
            break;
          case 'error':
            onError({
              error_code: event.error_code,
              message: event.message,
            });
            break;
        }
      }
    }
  } catch (error) {
    if (error instanceof Error && error.name === 'AbortError') {
      // Request was cancelled, don't treat as error
      return;
    }

    if (isChatError(error)) {
      onError(error);
    } else {
      onError({
        error_code: 'NETWORK_ERROR',
        message: error instanceof Error ? error.message : 'An unknown error occurred',
      });
    }
  }
}

/**
 * Non-streaming chat request fallback
 *
 * @param request - The chat request payload
 * @param signal - AbortSignal for cancellation
 * @returns The complete response with content and sources
 */
export async function sendChatRequest(
  request: ChatRequest,
  signal?: AbortSignal
): Promise<{ content: string; sources: Citation[] }> {
  const apiUrl = getApiUrl();
  const endpoint = `${apiUrl}/chat`;

  const response = await fetch(endpoint, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify(request),
    signal,
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw {
      error_code: `HTTP_${response.status}`,
      message: errorData.message || `Request failed with status ${response.status}`,
      retry_after: errorData.retry_after,
    } as ChatError;
  }

  return response.json();
}

/**
 * Check API health status
 */
export async function checkApiHealth(): Promise<boolean> {
  const apiUrl = getApiUrl();

  try {
    const response = await fetch(`${apiUrl}/health`, {
      method: 'GET',
    });
    return response.ok;
  } catch {
    return false;
  }
}

/**
 * Type guard for ChatError
 */
function isChatError(error: unknown): error is ChatError {
  return (
    typeof error === 'object' &&
    error !== null &&
    'error_code' in error &&
    'message' in error
  );
}

/**
 * Get user-friendly error message
 */
export function getErrorMessage(error: ChatError): string {
  switch (error.error_code) {
    case 'HTTP_429':
      return `Too many requests. Please wait ${error.retry_after || 60} seconds and try again.`;
    case 'HTTP_503':
      return 'The service is temporarily unavailable. Please try again later.';
    case 'HTTP_500':
      return 'An internal server error occurred. Please try again.';
    case 'NETWORK_ERROR':
      return 'Unable to connect to the server. Please check your internet connection.';
    case 'NO_STREAM':
      return 'Streaming is not supported. Please try again.';
    case 'TIMEOUT':
      return 'The request timed out. Please try again.';
    default:
      return error.message || 'An unexpected error occurred.';
  }
}

/**
 * Raw response from GET /conversations/{session_id}
 * Backend uses snake_case for all fields
 */
interface ConversationsResponseRaw {
  session_id: string;
  messages: Array<{
    id: string;
    role: 'user' | 'assistant';
    content: string;
    sources: Array<Record<string, unknown>> | null;
    mode: 'general' | 'selected_text';
    created_at: string;
  }>;
}

/**
 * Load conversation history for a session
 *
 * @param sessionId - UUID v4 session identifier
 * @param limit - Maximum number of conversation turns (default: 20, max: 20)
 * @returns Array of messages or empty array if none found
 */
export async function loadConversationHistory(
  sessionId: string,
  limit = 20
): Promise<Message[]> {
  const apiUrl = getApiUrl();
  const endpoint = `${apiUrl}/conversations/${sessionId}?limit=${Math.min(limit, 20)}`;

  try {
    const response = await fetch(endpoint, {
      method: 'GET',
      headers: {
        'Content-Type': 'application/json',
      },
    });

    // Return empty array for 404 (no history) or validation errors
    if (response.status === 404 || response.status === 400) {
      return [];
    }

    if (!response.ok) {
      console.warn(`Failed to load conversation history: ${response.status}`);
      return [];
    }

    const data: ConversationsResponseRaw = await response.json();

    // Transform API response to Message format (with snake_case -> camelCase for sources)
    return data.messages.map((msg) => ({
      id: msg.id,
      role: msg.role,
      content: msg.content,
      sources: msg.sources ? msg.sources.map(transformCitation) : undefined,
      mode: msg.mode,
      status: 'complete' as const,
      createdAt: new Date(msg.created_at),
    }));
  } catch (error) {
    console.warn('Failed to load conversation history:', error);
    return [];
  }
}
