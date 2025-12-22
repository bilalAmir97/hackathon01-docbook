/**
 * MessageInput - Text input component for sending chat messages
 * Supports Enter key submission and displays selected text badge
 */

import React, { useState, useRef, useCallback, useEffect } from 'react';
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
export function MessageInput({
  onSend,
  disabled = false,
  placeholder = 'Ask a question...',
  selectedText,
  onClearSelectedText,
  autoFocus = false,
}: MessageInputProps): JSX.Element {
  const [value, setValue] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

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
  const canSend = value.trim().length > 0 && !disabled;

  return (
    <form onSubmit={handleSubmit} className={styles.messageInputForm}>
      {/* Selected text badge */}
      {hasSelectedText && (
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
          placeholder={hasSelectedText ? 'Ask about the selected text...' : placeholder}
          disabled={disabled}
          className={styles.messageInputTextarea}
          rows={1}
          aria-label="Message input"
          aria-describedby={hasSelectedText ? 'selected-text-context' : undefined}
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

export default MessageInput;
