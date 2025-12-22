/**
 * ResponseSelectionTooltip - Tooltip that appears when text is selected within AI responses
 * Provides "Add to Follow-Up" and "Check Sources" actions
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { createPortal } from 'react-dom';
import styles from './chat.module.css';

/**
 * Props for the ResponseSelectionTooltip component
 */
interface ResponseSelectionTooltipProps {
  /** Callback when user wants to add selection to follow-up */
  onAddToFollowUp: (text: string) => void;
  /** Callback when user wants to check sources */
  onCheckSources: (text: string) => void;
  /** Whether sources are available for the current context */
  hasSources: boolean;
  /** Container element to listen for selections */
  containerRef: React.RefObject<HTMLElement>;
}

/**
 * Selection state within the response
 */
interface ResponseSelection {
  text: string;
  rect: DOMRect;
}

/**
 * Plus icon for "Add to Follow-Up"
 */
function PlusIcon(): JSX.Element {
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
      <line x1="12" y1="5" x2="12" y2="19" />
      <line x1="5" y1="12" x2="19" y2="12" />
    </svg>
  );
}

/**
 * Book/Source icon for "Check Sources"
 */
function SourceIcon(): JSX.Element {
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
      <path d="M4 19.5A2.5 2.5 0 0 1 6.5 17H20" />
      <path d="M6.5 2H20v20H6.5A2.5 2.5 0 0 1 4 19.5v-15A2.5 2.5 0 0 1 6.5 2z" />
    </svg>
  );
}

/**
 * ResponseSelectionTooltip component
 * Shows action buttons when text is selected within AI responses
 */
export function ResponseSelectionTooltip({
  onAddToFollowUp,
  onCheckSources,
  hasSources,
}: ResponseSelectionTooltipProps): JSX.Element | null {
  const [selection, setSelection] = useState<ResponseSelection | null>(null);
  const [isVisible, setIsVisible] = useState(false);
  const [mounted, setMounted] = useState(false);
  const tooltipRef = useRef<HTMLDivElement>(null);

  // Only render on client side
  useEffect(() => {
    setMounted(true);
  }, []);

  // Handle text selection within assistant messages
  const handleMouseUp = useCallback(() => {
    // Small delay to ensure selection is complete
    setTimeout(() => {
      const sel = window.getSelection();

      if (!sel || sel.isCollapsed || sel.rangeCount === 0) {
        setIsVisible(false);
        setTimeout(() => setSelection(null), 150);
        return;
      }

      const text = sel.toString().trim();
      if (text.length < 3) {
        setIsVisible(false);
        return;
      }

      // Check if selection is within an assistant message body
      const range = sel.getRangeAt(0);
      const container = range.commonAncestorContainer;
      const element = container.nodeType === Node.TEXT_NODE
        ? container.parentElement
        : container as Element;

      if (!element) {
        setIsVisible(false);
        return;
      }

      // Look for the data-message-body="assistant" attribute
      const messageBody = element.closest('[data-message-body="assistant"]');
      if (!messageBody) {
        setIsVisible(false);
        return;
      }

      const rect = range.getBoundingClientRect();

      // Ensure we have valid dimensions
      if (rect.width === 0 || rect.height === 0) {
        setIsVisible(false);
        return;
      }

      setSelection({ text, rect });
      setIsVisible(true);
    }, 10);
  }, []);

  // Handle click to potentially hide tooltip
  const handleClick = useCallback((e: MouseEvent) => {
    // If clicking on the tooltip itself, don't hide
    if (tooltipRef.current?.contains(e.target as Node)) {
      return;
    }

    // Small delay to allow selection to be checked first
    setTimeout(() => {
      const sel = window.getSelection();
      if (!sel || sel.isCollapsed) {
        setIsVisible(false);
        setSelection(null);
      }
    }, 50);
  }, []);

  // Set up event listeners
  useEffect(() => {
    if (!mounted) return;

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('click', handleClick);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('click', handleClick);
    };
  }, [mounted, handleMouseUp, handleClick]);

  // Handle click on "Add to Follow-Up"
  const handleAddToFollowUp = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    if (selection?.text) {
      onAddToFollowUp(selection.text);
      window.getSelection()?.removeAllRanges();
      setIsVisible(false);
      setSelection(null);
    }
  }, [selection, onAddToFollowUp]);

  // Handle click on "Check Sources"
  const handleCheckSources = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    if (selection?.text) {
      onCheckSources(selection.text);
      setIsVisible(false);
      setSelection(null);
    }
  }, [selection, onCheckSources]);

  // Don't render anything if not mounted or no selection
  if (!mounted || !selection || !isVisible) {
    return null;
  }

  // Calculate position (above the selection)
  const padding = 8;
  const tooltipHeight = 40;
  let top = selection.rect.top - tooltipHeight - padding;
  let left = selection.rect.left + selection.rect.width / 2;

  // Keep within viewport
  const tooltipWidth = hasSources ? 280 : 150;
  if (left - tooltipWidth / 2 < padding) {
    left = tooltipWidth / 2 + padding;
  } else if (left + tooltipWidth / 2 > window.innerWidth - padding) {
    left = window.innerWidth - tooltipWidth / 2 - padding;
  }

  // If tooltip would be above viewport, show below
  if (top < padding) {
    top = selection.rect.bottom + padding;
  }

  // Tooltip content
  const tooltipContent = (
    <div
      ref={tooltipRef}
      className={`${styles.responseSelectionTooltip} ${styles.responseSelectionTooltipVisible}`}
      style={{
        top: `${top}px`,
        left: `${left}px`,
      }}
      role="tooltip"
      data-chat-element="response-selection-tooltip"
    >
      <button
        type="button"
        onClick={handleAddToFollowUp}
        className={styles.responseSelectionButton}
        aria-label="Add to follow-up question"
      >
        <PlusIcon />
        <span>Add to Follow-Up</span>
      </button>

      {hasSources && (
        <button
          type="button"
          onClick={handleCheckSources}
          className={styles.responseSelectionButton}
          aria-label="Check sources for this text"
        >
          <SourceIcon />
          <span>Check Sources</span>
        </button>
      )}
    </div>
  );

  // Render via portal to document body
  return createPortal(tooltipContent, document.body);
}

export default ResponseSelectionTooltip;
