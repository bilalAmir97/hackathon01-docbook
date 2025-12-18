/**
 * TypeScript types for the ChatKit RAG chatbot
 * These types define the contract between frontend and backend
 */

/**
 * Represents a single chat message
 */
export interface Message {
  /** Unique identifier for the message */
  id: string;
  /** Role of the message sender */
  role: 'user' | 'assistant';
  /** Text content of the message */
  content: string;
  /** Citations/sources for assistant responses */
  sources?: Citation[];
  /** Mode of the message - general question or based on selected text */
  mode: 'general' | 'selected_text';
  /** Current status of the message */
  status: 'pending' | 'streaming' | 'complete' | 'error';
  /** Timestamp when message was created */
  createdAt: Date;
  /** Optional error message if status is 'error' */
  errorMessage?: string;
}

/**
 * Represents a citation/source for an answer
 */
export interface Citation {
  /** URL to the source document */
  sourceUrl: string;
  /** Title of the page */
  pageTitle: string;
  /** Section heading within the page */
  sectionHeading: string;
  /** Relevant text chunk from the source */
  chunkText: string;
  /** Relevance score from vector search (0-1) */
  relevanceScore: number;
}

/**
 * Request payload for the chat API
 */
export interface ChatRequest {
  /** The user's question */
  query: string;
  /** Session ID for conversation continuity */
  session_id?: string;
  /** Mode of the question */
  mode?: 'general' | 'selected_text';
  /** Selected text context if mode is 'selected_text' */
  selected_text?: string;
}

/**
 * Error response from the chat API
 */
export interface ChatError {
  /** Error code for categorization */
  error_code: string;
  /** Human-readable error message */
  message: string;
  /** Seconds to wait before retrying (for rate limits) */
  retry_after?: number;
}

/**
 * Union type for all possible SSE stream events
 */
export type StreamEvent =
  | StreamChunkEvent
  | StreamSourcesEvent
  | StreamDoneEvent
  | StreamErrorEvent;

/**
 * SSE event containing a chunk of the response
 */
export interface StreamChunkEvent {
  type: 'chunk';
  content: string;
}

/**
 * SSE event containing source citations
 */
export interface StreamSourcesEvent {
  type: 'sources';
  sources: Citation[];
}

/**
 * SSE event indicating stream completion
 */
export interface StreamDoneEvent {
  type: 'done';
  metadata: {
    query_time_ms: number;
    chunks_retrieved: number;
    model: string;
  };
}

/**
 * SSE event indicating an error occurred
 */
export interface StreamErrorEvent {
  type: 'error';
  error_code: string;
  message: string;
}

/**
 * Selected text state from the document
 */
export interface SelectedTextState {
  /** The selected text content */
  text: string;
  /** Bounding rectangle for positioning the tooltip */
  rect: DOMRect | null;
  /** Whether text is currently selected */
  isActive: boolean;
}

/**
 * Chat widget state
 */
export interface ChatState {
  /** Whether the chat panel is open */
  isOpen: boolean;
  /** All messages in the conversation */
  messages: Message[];
  /** Whether a request is in progress */
  isLoading: boolean;
  /** Current error if any */
  error: ChatError | null;
  /** Selected text context */
  selectedText: SelectedTextState | null;
}

/**
 * Chat context actions
 */
export interface ChatActions {
  /** Toggle chat panel open/closed */
  toggleChat: () => void;
  /** Open chat panel */
  openChat: () => void;
  /** Close chat panel */
  closeChat: () => void;
  /** Send a message */
  sendMessage: (content: string, selectedText?: string) => Promise<void>;
  /** Clear chat history */
  clearMessages: () => void;
  /** Set selected text */
  setSelectedText: (state: SelectedTextState | null) => void;
  /** Clear any error */
  clearError: () => void;
}

/**
 * Combined chat context type
 */
export interface ChatContextType extends ChatState, ChatActions {}

/**
 * API configuration
 */
export interface ApiConfig {
  /** Base URL for the API */
  baseUrl: string;
  /** Request timeout in milliseconds */
  timeout?: number;
}
