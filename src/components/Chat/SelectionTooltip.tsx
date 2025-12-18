/**
 * SelectionTooltip - Floating tooltip that appears when text is selected
 * Provides "Ask about this" button to use selected text as context
 */

import React, { useEffect, useState, useCallback } from 'react';
import type { SelectedTextState } from '../../types/chat';
import styles from './chat.module.css';

/**
 * Props for the SelectionTooltip component
 */
interface SelectionTooltipProps {
  /** Selected text state from useSelectedText hook */
  selectedText: SelectedTextState | null;
  /** Callback when user wants to ask about selected text */
  onAskAbout: (text: string) => void;
}

/**
 * Chat icon SVG
 */
function ChatIcon(): JSX.Element {
  return (
    <svg
      width="14"
      height="14"
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
 * Calculate tooltip position from selection rect
 */
function calculatePosition(rect: DOMRect): { top: number; left: number } {
  const padding = 8;
  const tooltipHeight = 40;

  // Position above the selection
  let top = rect.top - tooltipHeight - padding + window.scrollY;
  let left = rect.left + rect.width / 2 + window.scrollX;

  // Keep within viewport horizontally
  const tooltipWidth = 150; // Approximate width
  const halfWidth = tooltipWidth / 2;
  const viewportWidth = window.innerWidth;

  if (left - halfWidth < padding) {
    left = halfWidth + padding;
  } else if (left + halfWidth > viewportWidth - padding) {
    left = viewportWidth - halfWidth - padding;
  }

  // If tooltip would be above viewport, show below selection
  if (top < padding) {
    top = rect.bottom + padding + window.scrollY;
  }

  return { top, left };
}

/**
 * Floating tooltip for text selection
 * Shows "Ask about this" button when text is selected
 *
 * @example
 * ```tsx
 * <SelectionTooltip
 *   selectedText={selectedText}
 *   onAskAbout={(text) => {
 *     openChat();
 *     setContext(text);
 *   }}
 * />
 * ```
 */
export function SelectionTooltip({
  selectedText,
  onAskAbout,
}: SelectionTooltipProps): JSX.Element | null {
  const [position, setPosition] = useState<{ top: number; left: number } | null>(
    null
  );
  const [isVisible, setIsVisible] = useState(false);

  // Update position when selection changes
  useEffect(() => {
    if (selectedText?.isActive && selectedText.rect) {
      const newPosition = calculatePosition(selectedText.rect);
      setPosition(newPosition);
      // Small delay for animation
      requestAnimationFrame(() => {
        setIsVisible(true);
      });
    } else {
      setIsVisible(false);
      // Clear position after fade out
      const timeout = setTimeout(() => {
        setPosition(null);
      }, 200);
      return () => clearTimeout(timeout);
    }
  }, [selectedText]);

  // Handle click on the tooltip button
  const handleClick = useCallback(() => {
    if (selectedText?.text) {
      onAskAbout(selectedText.text);
    }
  }, [selectedText, onAskAbout]);

  // Don't render if no position
  if (!position) {
    return null;
  }

  return (
    <div
      className={`${styles.selectionTooltip} ${isVisible ? styles.selectionTooltipVisible : ''}`}
      style={{
        top: `${position.top}px`,
        left: `${position.left}px`,
      }}
      role="tooltip"
      aria-hidden={!isVisible}
      data-chat-element="selection-tooltip"
    >
      <button
        type="button"
        onClick={handleClick}
        className={styles.selectionTooltipButton}
        aria-label="Ask about selected text"
      >
        <ChatIcon />
        <span>Ask about this</span>
      </button>
    </div>
  );
}

export default SelectionTooltip;
