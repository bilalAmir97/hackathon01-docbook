/**
 * MessageInput - Text input component for sending chat messages
 * Supports Enter key submission and displays selected text badge
 */

import React, { useState, useRef, useCallback, useEffect, forwardRef } from 'react';
import type { SelectedTextState } from '../../types/chat';
import { SelectedTextBadge } from './SelectedTextBadge';
import styles from './chat.module.css';

/**
 * Props for the MessageInput component
 */
interface MessageInputProps {
  /** Callback when message is submitted */
  onSend: (message: string) => void;
  /** Whether input should be disabled */
  disabled?: boolean;
  /** Placeholder text */
  placeholder?: string;
  /** Selected text context */
  selectedText?: SelectedTextState | null;
  /** Callback to clear selected text */
  onClearSelectedText?: () => void;
  /** Follow-up citation from AI response selection */
  followUpCitation?: string | null;
  /** Callback to clear follow-up citation */
  onClearFollowUpCitation?: () => void;
  /** Auto-focus the input */
  autoFocus?: boolean;
}

/**
 * Send icon SVG
 */
function SendIcon(): JSX.Element {
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
      <line x1="22" y1="2" x2="11" y2="13" />
      <polygon points="22 2 15 22 11 13 2 9 22 2" />
    </svg>
  );
}

/**
 * Quote icon for follow-up citation
 */
function QuoteIcon(): JSX.Element {
  return (
    <svg
      width="12"
      height="12"
      viewBox="0 0 24 24"
      fill="currentColor"
      aria-hidden="true"
    >
      <path d="M6 17h3l2-4V7H5v6h3zm8 0h3l2-4V7h-6v6h3z" />
    </svg>
  );
}

/**
 * Message input component with selected text support
 *
 * @example
 * ```tsx
 * <MessageInput
 *   onSend={(msg) => sendMessage(msg)}
 *   disabled={isLoading}
 *   selectedText={selectedText}
 *   onClearSelectedText={clearSelection}
 * />
 * ```
 */
export const MessageInput = forwardRef<HTMLTextAreaElement, MessageInputProps>(
  function MessageInput(
    {
      onSend,
      disabled = false,
      placeholder = 'Ask a question...',
      selectedText,
      onClearSelectedText,
      followUpCitation,
      onClearFollowUpCitation,
      autoFocus = false,
    },
    ref
  ) {
  const [value, setValue] = useState('');
  const internalRef = useRef<HTMLTextAreaElement>(null);
  const textareaRef = (ref as React.RefObject<HTMLTextAreaElement>) || internalRef;

  // Focus input when autoFocus is true
  useEffect(() => {
    if (autoFocus && textareaRef.current) {
      textareaRef.current.focus();
    }
  }, [autoFocus]);

  // Auto-resize textarea based on content
  const adjustHeight = useCallback(() => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      const newHeight = Math.min(textarea.scrollHeight, 150); // Max height of 150px
      textarea.style.height = `${newHeight}px`;
    }
  }, []);

  // Handle input change
  const handleChange = useCallback(
    (e: React.ChangeEvent<HTMLTextAreaElement>) => {
      setValue(e.target.value);
      adjustHeight();
    },
    [adjustHeight]
  );

  // Handle form submission
  const handleSubmit = useCallback(
    (e?: React.FormEvent) => {
      e?.preventDefault();

      const trimmedValue = value.trim();
      if (!trimmedValue || disabled) return;

      onSend(trimmedValue);
      setValue('');

      // Reset textarea height
      if (textareaRef.current) {
        textareaRef.current.style.height = 'auto';
      }
    },
    [value, disabled, onSend]
  );

  // Handle keyboard events
  const handleKeyDown = useCallback(
    (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
      // Submit on Enter (without Shift)
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        handleSubmit();
      }
    },
    [handleSubmit]
  );

  const hasSelectedText = selectedText?.isActive && selectedText.text;
  const hasFollowUpCitation = followUpCitation && followUpCitation.length > 0;
  const canSend = value.trim().length > 0 && !disabled;

  // Determine placeholder based on context
  const getPlaceholder = () => {
    if (hasFollowUpCitation) return 'Ask a follow-up question...';
    if (hasSelectedText) return 'Ask about the selected text...';
    return placeholder;
  };

  return (
    <form onSubmit={handleSubmit} className={styles.messageInputForm}>
      {/* Follow-up citation badge */}
      {hasFollowUpCitation && (
        <div className={styles.followUpCitation}>
          <div className={styles.followUpCitationHeader}>
            <QuoteIcon />
            <span>Follow-up on:</span>
            {onClearFollowUpCitation && (
              <button
                type="button"
                onClick={onClearFollowUpCitation}
                className={styles.followUpCitationClear}
                aria-label="Clear citation"
              >
                ×
              </button>
            )}
          </div>
          <blockquote className={styles.followUpCitationText}>
            "{followUpCitation.length > 100
              ? followUpCitation.slice(0, 100) + '...'
              : followUpCitation}"
          </blockquote>
        </div>
      )}

      {/* Selected text badge */}
      {hasSelectedText && !hasFollowUpCitation && (
        <SelectedTextBadge
          text={selectedText.text}
          onClear={onClearSelectedText}
        />
      )}

      <div className={styles.messageInputContainer}>
        <textarea
          ref={textareaRef}
          value={value}
          onChange={handleChange}
          onKeyDown={handleKeyDown}
          placeholder={getPlaceholder()}
          disabled={disabled}
          className={styles.messageInputTextarea}
          rows={1}
          aria-label="Message input"
          aria-describedby={
            hasFollowUpCitation
              ? 'follow-up-citation'
              : hasSelectedText
                ? 'selected-text-context'
                : undefined
          }
        />

        <button
          type="submit"
          disabled={!canSend}
          className={styles.messageInputSend}
          aria-label="Send message"
        >
          <SendIcon />
        </button>
      </div>

      {/* Keyboard hint */}
      <div className={styles.messageInputHint}>
        Press <kbd>Enter</kbd> to send, <kbd>Shift+Enter</kbd> for new line
      </div>
    </form>
  );
  }
);

export default MessageInput;
