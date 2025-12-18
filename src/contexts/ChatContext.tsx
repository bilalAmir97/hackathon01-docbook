/**
 * ChatContext - React context provider for chat state management
 * Provides global access to chat functionality across the application
 */

import React, {
  createContext,
  useContext,
  useState,
  useCallback,
  useMemo,
  type ReactNode,
} from 'react';
import type {
  ChatContextType,
  Message,
  ChatError,
  SelectedTextState,
} from '../types/chat';
import { useChat } from '../hooks/useChat';
import { useSelectedText } from '../hooks/useSelectedText';

/**
 * Default context value (used when provider is not present)
 */
const defaultContext: ChatContextType = {
  isOpen: false,
  messages: [],
  isLoading: false,
  error: null,
  selectedText: null,
  toggleChat: () => {},
  openChat: () => {},
  closeChat: () => {},
  sendMessage: async () => {},
  clearMessages: () => {},
  setSelectedText: () => {},
  clearError: () => {},
};

/**
 * Chat context instance
 */
const ChatContext = createContext<ChatContextType>(defaultContext);

/**
 * Props for the ChatProvider component
 */
interface ChatProviderProps {
  children: ReactNode;
  /** Initial open state */
  defaultOpen?: boolean;
  /** Callback when chat is opened */
  onOpen?: () => void;
  /** Callback when chat is closed */
  onClose?: () => void;
}

/**
 * Chat context provider component
 * Wraps the application to provide chat state and functions
 *
 * @example
 * ```tsx
 * // In your root component (e.g., Root.tsx)
 * <ChatProvider>
 *   <App />
 * </ChatProvider>
 *
 * // In any child component
 * const { sendMessage, messages, isOpen } = useChatContext();
 * ```
 */
export function ChatProvider({
  children,
  defaultOpen = false,
  onOpen,
  onClose,
}: ChatProviderProps): JSX.Element {
  // Panel open/close state
  const [isOpen, setIsOpen] = useState(defaultOpen);

  // Use the chat hook for message management
  const {
    messages,
    isLoading,
    error,
    sendMessage: sendChatMessage,
    clearMessages,
    clearError,
  } = useChat({
    onError: (err) => {
      console.error('Chat error:', err);
    },
  });

  // Use selected text hook
  const { selectedText, clearSelection, setSelection } = useSelectedText({
    enabled: true,
    containerSelector: '.markdown, article, .theme-doc-markdown, main',
  });

  // Toggle chat panel
  const toggleChat = useCallback(() => {
    setIsOpen((prev) => {
      const newState = !prev;
      if (newState) {
        onOpen?.();
      } else {
        onClose?.();
      }
      return newState;
    });
  }, [onOpen, onClose]);

  // Open chat panel
  const openChat = useCallback(() => {
    if (!isOpen) {
      setIsOpen(true);
      onOpen?.();
    }
  }, [isOpen, onOpen]);

  // Close chat panel
  const closeChat = useCallback(() => {
    if (isOpen) {
      setIsOpen(false);
      onClose?.();
    }
  }, [isOpen, onClose]);

  // Send message with optional selected text
  const sendMessage = useCallback(
    async (content: string, contextText?: string) => {
      // Create a SelectedTextState if contextText is provided
      const textState: SelectedTextState | null = contextText
        ? { text: contextText, rect: null, isActive: true }
        : selectedText;

      await sendChatMessage(content, textState);

      // Clear selection after sending
      if (selectedText?.isActive) {
        clearSelection();
      }
    },
    [sendChatMessage, selectedText, clearSelection]
  );

  // Set selected text state
  const setSelectedText = useCallback(
    (state: SelectedTextState | null) => {
      setSelection(state);
    },
    [setSelection]
  );

  // Memoize context value to prevent unnecessary re-renders
  const contextValue = useMemo<ChatContextType>(
    () => ({
      isOpen,
      messages,
      isLoading,
      error,
      selectedText,
      toggleChat,
      openChat,
      closeChat,
      sendMessage,
      clearMessages,
      setSelectedText,
      clearError,
    }),
    [
      isOpen,
      messages,
      isLoading,
      error,
      selectedText,
      toggleChat,
      openChat,
      closeChat,
      sendMessage,
      clearMessages,
      setSelectedText,
      clearError,
    ]
  );

  return (
    <ChatContext.Provider value={contextValue}>{children}</ChatContext.Provider>
  );
}

/**
 * Hook to access chat context
 *
 * @returns Chat context value
 * @throws Error if used outside of ChatProvider
 *
 * @example
 * ```tsx
 * function ChatButton() {
 *   const { toggleChat, isOpen } = useChatContext();
 *   return (
 *     <button onClick={toggleChat}>
 *       {isOpen ? 'Close' : 'Open'} Chat
 *     </button>
 *   );
 * }
 * ```
 */
export function useChatContext(): ChatContextType {
  const context = useContext(ChatContext);

  if (context === defaultContext) {
    console.warn(
      'useChatContext must be used within a ChatProvider. Using default context.'
    );
  }

  return context;
}

export default ChatContext;
