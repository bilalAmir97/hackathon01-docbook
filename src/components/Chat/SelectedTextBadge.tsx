/**
 * SelectedTextBadge - Displays captured selected text context
 * Shows in the input area when text is selected from the document
 */

import React from 'react';
import styles from './chat.module.css';

/**
 * Props for the SelectedTextBadge component
 */
interface SelectedTextBadgeProps {
  /** The selected text content */
  text: string;
  /** Callback to clear the selection */
  onClear?: () => void;
  /** Maximum characters to display before truncating */
  maxLength?: number;
}

/**
 * Quote icon SVG
 */
function QuoteIcon(): JSX.Element {
  return (
    <svg
      width="14"
      height="14"
      viewBox="0 0 24 24"
      fill="currentColor"
      aria-hidden="true"
    >
      <path d="M6 17h3l2-4V7H5v6h3zm8 0h3l2-4V7h-6v6h3z" />
    </svg>
  );
}

/**
 * Close icon SVG
 */
function CloseIcon(): JSX.Element {
  return (
    <svg
      width="12"
      height="12"
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
 * Truncate text with ellipsis
 */
function truncateText(text: string, maxLength: number): string {
  if (text.length <= maxLength) return text;
  return text.slice(0, maxLength).trim() + '...';
}

/**
 * Badge showing selected text context
 * Appears above the input when text is selected from documentation
 *
 * @example
 * ```tsx
 * {selectedText && (
 *   <SelectedTextBadge
 *     text={selectedText}
 *     onClear={handleClear}
 *   />
 * )}
 * ```
 */
export function SelectedTextBadge({
  text,
  onClear,
  maxLength = 100,
}: SelectedTextBadgeProps): JSX.Element {
  const displayText = truncateText(text, maxLength);

  return (
    <div
      className={styles.selectedTextBadge}
      id="selected-text-context"
      role="status"
      aria-label="Selected text context"
    >
      <div className={styles.selectedTextBadgeHeader}>
        <span className={styles.selectedTextBadgeIcon}>
          <QuoteIcon />
        </span>
        <span className={styles.selectedTextBadgeLabel}>
          Asking about selected text
        </span>
        {onClear && (
          <button
            type="button"
            onClick={onClear}
            className={styles.selectedTextBadgeClear}
            aria-label="Clear selected text"
          >
            <CloseIcon />
          </button>
        )}
      </div>
      <blockquote className={styles.selectedTextBadgeContent}>
        {displayText}
      </blockquote>
    </div>
  );
}

export default SelectedTextBadge;
