/**
 * useChat hook for managing chat state and API communication
 * Handles streaming responses, message history, and error states
 */

import { useState, useCallback, useRef, useEffect } from 'react';
import type { Message, Citation, ChatError, SelectedTextState } from '../types/chat';
import { streamChatResponse, getErrorMessage } from '../services/api';
import { useSession } from './useSession';

/**
 * Maximum number of messages that can be queued
 */
const MAX_QUEUE_SIZE = 3;

/**
 * Debounce delay in milliseconds
 */
const DEBOUNCE_DELAY_MS = 300;

/**
 * Generate a unique message ID
 */
function generateMessageId(): string {
  return `msg_${Date.now()}_${Math.random().toString(36).slice(2, 9)}`;
}

/**
 * Hook configuration options
 */
interface UseChatOptions {
  /** Initial messages to populate the chat */
  initialMessages?: Message[];
  /** Callback when a message is sent */
  onMessageSent?: (message: Message) => void;
  /** Callback when a response is received */
  onResponseReceived?: (message: Message) => void;
  /** Callback on error */
  onError?: (error: ChatError) => void;
}

/**
 * Hook return type
 */
/**
 * Queued message type
 */
interface QueuedMessage {
  content: string;
  selectedText?: SelectedTextState | null;
}

interface UseChatReturn {
  /** All messages in the conversation */
  messages: Message[];
  /** Whether a request is in progress */
  isLoading: boolean;
  /** Whether currently sending (before streaming starts) */
  isSending: boolean;
  /** Number of messages in queue */
  queuedCount: number;
  /** Current error if any */
  error: ChatError | null;
  /** User-friendly error message */
  errorMessage: string | null;
  /** Send a new message */
  sendMessage: (content: string, selectedText?: SelectedTextState | null) => Promise<void>;
  /** Clear all messages */
  clearMessages: () => void;
  /** Clear error state */
  clearError: () => void;
  /** Cancel ongoing request */
  cancelRequest: () => void;
  /** Session ID for the current conversation */
  sessionId: string;
  /** Reset session and clear messages */
  resetConversation: () => void;
}

/**
 * Hook for managing chat functionality
 *
 * @param options - Configuration options
 * @returns Chat state and control functions
 *
 * @example
 * ```tsx
 * const {
 *   messages,
 *   isLoading,
 *   error,
 *   sendMessage,
 *   clearMessages
 * } = useChat();
 *
 * // Send a message
 * await sendMessage('What is ROS 2?');
 *
 * // Send with selected text context
 * await sendMessage('Explain this', selectedText);
 * ```
 */
export function useChat(options: UseChatOptions = {}): UseChatReturn {
  const { initialMessages = [], onMessageSent, onResponseReceived, onError } = options;

  const [messages, setMessages] = useState<Message[]>(initialMessages);
  const [isLoading, setIsLoading] = useState(false);
  const [isSending, setIsSending] = useState(false);
  const [error, setError] = useState<ChatError | null>(null);
  const [messageQueue, setMessageQueue] = useState<QueuedMessage[]>([]);

  const { sessionId, resetSession } = useSession();
  const abortControllerRef = useRef<AbortController | null>(null);
  const lastSubmitTimeRef = useRef<number>(0);
  const processingQueueRef = useRef<boolean>(false);

  // Add a message to the list
  const addMessage = useCallback((message: Message) => {
    setMessages((prev) => [...prev, message]);
  }, []);

  // Update a message by ID
  const updateMessage = useCallback((id: string, updates: Partial<Message>) => {
    setMessages((prev) =>
      prev.map((msg) => (msg.id === id ? { ...msg, ...updates } : msg))
    );
  }, []);

  // Clear error
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  // Cancel ongoing request
  const cancelRequest = useCallback(() => {
    if (abortControllerRef.current) {
      abortControllerRef.current.abort();
      abortControllerRef.current = null;
    }
    setIsLoading(false);
  }, []);

  // Clear all messages and queue
  const clearMessages = useCallback(() => {
    setMessages([]);
    setError(null);
    setMessageQueue([]);
    cancelRequest();
  }, [cancelRequest]);

  // Reset conversation (new session + clear messages)
  const resetConversation = useCallback(() => {
    clearMessages();
    resetSession();
  }, [clearMessages, resetSession]);

  // Internal function to process a message (without debounce/queue checks)
  const processMessage = useCallback(
    async (content: string, selectedText?: SelectedTextState | null) => {
      // Cancel any existing request
      cancelRequest();

      // Create abort controller for this request
      abortControllerRef.current = new AbortController();

      // Determine mode based on selected text
      const mode = selectedText?.isActive ? 'selected_text' : 'general';

      // Create user message
      const userMessage: Message = {
        id: generateMessageId(),
        role: 'user',
        content: content.trim(),
        mode,
        status: 'complete',
        createdAt: new Date(),
      };

      // Add user message immediately
      addMessage(userMessage);
      onMessageSent?.(userMessage);

      // Create placeholder for assistant response
      const assistantMessageId = generateMessageId();
      const assistantMessage: Message = {
        id: assistantMessageId,
        role: 'assistant',
        content: '',
        mode,
        status: 'pending',
        createdAt: new Date(),
      };

      addMessage(assistantMessage);
      setIsLoading(true);
      setError(null);

      try {
        // Update status to streaming once we start receiving data
        let hasReceivedData = false;

        await streamChatResponse(
          {
            query: content.trim(),
            session_id: sessionId,
            mode,
            selected_text: selectedText?.isActive ? selectedText.text : undefined,
          },
          // On chunk received
          (chunk) => {
            if (!hasReceivedData) {
              hasReceivedData = true;
              updateMessage(assistantMessageId, { status: 'streaming' });
            }
            setMessages((prev) =>
              prev.map((msg) =>
                msg.id === assistantMessageId
                  ? { ...msg, content: msg.content + chunk }
                  : msg
              )
            );
          },
          // On sources received
          (sources: Citation[]) => {
            updateMessage(assistantMessageId, { sources });
          },
          // On done
          () => {
            updateMessage(assistantMessageId, { status: 'complete' });
            setIsLoading(false);

            // Get final message for callback
            setMessages((prev) => {
              const finalMessage = prev.find((m) => m.id === assistantMessageId);
              if (finalMessage) {
                onResponseReceived?.(finalMessage);
              }
              return prev;
            });
          },
          // On error
          (chatError: ChatError) => {
            updateMessage(assistantMessageId, {
              status: 'error',
              errorMessage: getErrorMessage(chatError),
            });
            setError(chatError);
            setIsLoading(false);
            onError?.(chatError);
          },
          abortControllerRef.current.signal
        );
      } catch (err) {
        const chatError: ChatError = {
          error_code: 'UNKNOWN',
          message: err instanceof Error ? err.message : 'An unknown error occurred',
        };
        updateMessage(assistantMessageId, {
          status: 'error',
          errorMessage: getErrorMessage(chatError),
        });
        setError(chatError);
        setIsLoading(false);
        onError?.(chatError);
      }
    },
    [
      sessionId,
      addMessage,
      updateMessage,
      cancelRequest,
      onMessageSent,
      onResponseReceived,
      onError,
    ]
  );

  // Send a message with debounce and queue support
  const sendMessage = useCallback(
    async (content: string, selectedText?: SelectedTextState | null) => {
      if (!content.trim()) return;

      const now = Date.now();
      const timeSinceLastSubmit = now - lastSubmitTimeRef.current;

      // Debounce: Ignore rapid double-clicks within DEBOUNCE_DELAY_MS
      if (timeSinceLastSubmit < DEBOUNCE_DELAY_MS && isLoading) {
        return;
      }

      lastSubmitTimeRef.current = now;

      // If currently loading, queue the message instead
      if (isLoading) {
        if (messageQueue.length < MAX_QUEUE_SIZE) {
          setMessageQueue((prev) => [...prev, { content, selectedText }]);
        } else {
          console.warn(`Message queue is full (max ${MAX_QUEUE_SIZE}). Message not queued.`);
        }
        return;
      }

      setIsSending(true);
      try {
        await processMessage(content, selectedText);
      } finally {
        setIsSending(false);
      }
    },
    [isLoading, messageQueue.length, processMessage]
  );

  // Process queued messages after current response completes
  useEffect(() => {
    const processQueue = async () => {
      if (isLoading || processingQueueRef.current || messageQueue.length === 0) {
        return;
      }

      processingQueueRef.current = true;

      // Get next message from queue
      const [nextMessage, ...rest] = messageQueue;
      setMessageQueue(rest);

      setIsSending(true);
      try {
        await processMessage(nextMessage.content, nextMessage.selectedText);
      } finally {
        setIsSending(false);
        processingQueueRef.current = false;
      }
    };

    // Only process queue when not loading and queue has items
    if (!isLoading && messageQueue.length > 0 && !processingQueueRef.current) {
      processQueue();
    }
  }, [isLoading, messageQueue, processMessage]);

  // Clear queue on error
  useEffect(() => {
    if (error) {
      setMessageQueue([]);
    }
  }, [error]);

  return {
    messages,
    isLoading,
    isSending,
    queuedCount: messageQueue.length,
    error,
    errorMessage: error ? getErrorMessage(error) : null,
    sendMessage,
    clearMessages,
    clearError,
    cancelRequest,
    sessionId,
    resetConversation,
  };
}

export default useChat;
