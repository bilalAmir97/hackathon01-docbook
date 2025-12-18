/**
 * ChatToggle - Floating Action Button (FAB) for toggling the chat panel
 * Positioned in the bottom-right corner of the viewport
 */

import React from 'react';
import { useChatContext } from '../../contexts/ChatContext';
import styles from './chat.module.css';

/**
 * Props for the ChatToggle component
 */
interface ChatToggleProps {
  /** Custom class name */
  className?: string;
  /** Accessible label for the button */
  ariaLabel?: string;
}

/**
 * Chat icon SVG component
 */
function ChatIcon(): JSX.Element {
  return (
    <svg
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
    </svg>
  );
}

/**
 * Close icon SVG component
 */
function CloseIcon(): JSX.Element {
  return (
    <svg
      width="24"
      height="24"
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
 * Floating action button for toggling the chat panel
 *
 * @example
 * ```tsx
 * <ChatToggle ariaLabel="Open AI Assistant" />
 * ```
 */
export function ChatToggle({
  className = '',
  ariaLabel = 'Toggle chat assistant',
}: ChatToggleProps): JSX.Element {
  const { isOpen, toggleChat, isLoading, messages } = useChatContext();

  // Count unread messages (messages since last close)
  const hasUnreadMessages =
    !isOpen && messages.length > 0 && messages[messages.length - 1].role === 'assistant';

  return (
    <button
      type="button"
      onClick={toggleChat}
      className={`${styles.chatToggle} ${isOpen ? styles.chatToggleOpen : ''} ${className}`}
      aria-label={ariaLabel}
      aria-expanded={isOpen}
      aria-controls="chat-panel"
      data-chat-element="toggle"
    >
      <span className={styles.chatToggleIcon}>
        {isOpen ? <CloseIcon /> : <ChatIcon />}
      </span>

      {/* Loading indicator */}
      {isLoading && (
        <span className={styles.chatToggleLoading} aria-hidden="true">
          <span className={styles.chatToggleLoadingDot} />
          <span className={styles.chatToggleLoadingDot} />
          <span className={styles.chatToggleLoadingDot} />
        </span>
      )}

      {/* Unread indicator */}
      {hasUnreadMessages && !isOpen && (
        <span className={styles.chatToggleUnread} aria-label="New message">
          <span className={styles.chatToggleUnreadDot} />
        </span>
      )}
    </button>
  );
}

export default ChatToggle;
