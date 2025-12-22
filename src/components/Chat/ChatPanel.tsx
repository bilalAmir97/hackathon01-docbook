/**
 * ChatPanel - Main chat interface panel
 * Contains header, message list, and input area
 * Wrapped in error boundary for graceful error handling
 */

import React, { useCallback } from 'react';
import { useChatContext } from '../../contexts/ChatContext';
import { MessageList } from './MessageList';
import { MessageInput } from './MessageInput';
import { ChatErrorBoundary } from './ErrorFallback';
import styles from './chat.module.css';

/**
 * Props for the ChatPanel component
 */
interface ChatPanelProps {
  /** Custom class name */
  className?: string;
}

/**
 * Close icon SVG
 */
function CloseIcon(): JSX.Element {
  return (
    <svg
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <line x1="18" y1="6" x2="6" y2="18" />
      <line x1="6" y1="6" x2="18" y2="18" />
    </svg>
  );
}

/**
 * Trash icon SVG
 */
function TrashIcon(): JSX.Element {
  return (
    <svg
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <polyline points="3 6 5 6 21 6" />
      <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2" />
    </svg>
  );
}

/**
 * Bot icon SVG for header
 */
function BotIcon(): JSX.Element {
  return (
    <svg
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="currentColor"
      aria-hidden="true"
    >
      <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z" />
    </svg>
  );
}

/**
 * Main chat panel component
 * Contains the full chat interface
 *
 * @example
 * ```tsx
 * <ChatPanel className="custom-panel" />
 * ```
 */
export function ChatPanel({ className = '' }: ChatPanelProps): JSX.Element {
  const {
    isOpen,
    messages,
    isLoading,
    selectedText,
    closeChat,
    sendMessage,
    clearMessages,
    setSelectedText,
  } = useChatContext();

  // Handle message send
  const handleSend = useCallback(
    (content: string) => {
      sendMessage(content);
    },
    [sendMessage]
  );

  // Handle clear selected text
  const handleClearSelectedText = useCallback(() => {
    setSelectedText(null);
  }, [setSelectedText]);

  // Handle clear messages with confirmation
  const handleClearMessages = useCallback(() => {
    if (messages.length > 0) {
      const confirmed = window.confirm(
        'Are you sure you want to clear the conversation?'
      );
      if (confirmed) {
        clearMessages();
      }
    }
  }, [messages.length, clearMessages]);

  if (!isOpen) {
    return <></>;
  }

  return (
    <div
      id="chat-panel"
      className={`${styles.chatPanel} ${className}`}
      role="dialog"
      aria-label="Chat assistant"
      aria-modal="false"
      data-chat-element="panel"
    >
      {/* Header - outside error boundary so close button always works */}
      <header className={styles.chatPanelHeader}>
        <div className={styles.chatPanelHeaderTitle}>
          <span className={styles.chatPanelHeaderIcon}>
            <BotIcon />
          </span>
          <div className={styles.chatPanelHeaderText}>
            <h2 className={styles.chatPanelTitle}>AI Assistant</h2>
            <span className={styles.chatPanelSubtitle}>
              Ask questions about the documentation
            </span>
          </div>
        </div>

        <div className={styles.chatPanelHeaderActions}>
          {messages.length > 0 && (
            <button
              type="button"
              onClick={handleClearMessages}
              className={styles.chatPanelHeaderButton}
              aria-label="Clear conversation"
              title="Clear conversation"
            >
              <TrashIcon />
            </button>
          )}
          <button
            type="button"
            onClick={closeChat}
            className={styles.chatPanelHeaderButton}
            aria-label="Close chat"
            title="Close chat"
          >
            <CloseIcon />
          </button>
        </div>
      </header>

      {/* Main content wrapped in error boundary */}
      <ChatErrorBoundary
        onError={(error, errorInfo) => {
          console.error('ChatPanel error:', error, errorInfo);
        }}
      >
        {/* Messages */}
        <div className={styles.chatPanelBody}>
          <MessageList messages={messages} isLoading={isLoading} />
        </div>

        {/* Input */}
        <footer className={styles.chatPanelFooter}>
          <MessageInput
            onSend={handleSend}
            disabled={isLoading}
            selectedText={selectedText}
            onClearSelectedText={handleClearSelectedText}
            autoFocus={isOpen}
          />
        </footer>
      </ChatErrorBoundary>
    </div>
  );
}

export default ChatPanel;
