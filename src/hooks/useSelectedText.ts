/**
 * useSelectedText hook for capturing text selection from the document
 * Implements browser Selection API with bounding rect calculation
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import type { SelectedTextState } from '../types/chat';

/**
 * Configuration options for the hook
 */
interface UseSelectedTextOptions {
  /** Minimum character length to consider as valid selection */
  minLength?: number;
  /** Maximum character length to capture */
  maxLength?: number;
  /** Debounce delay in milliseconds */
  debounceMs?: number;
  /** CSS selector for the container to listen for selections */
  containerSelector?: string;
  /** Whether the hook is enabled */
  enabled?: boolean;
}

const DEFAULT_OPTIONS: Required<UseSelectedTextOptions> = {
  minLength: 3,
  maxLength: 2000,
  debounceMs: 150,
  containerSelector: '.markdown, article, .theme-doc-markdown',
  enabled: true,
};

/**
 * Hook return type
 */
interface UseSelectedTextReturn {
  /** Current selected text state */
  selectedText: SelectedTextState | null;
  /** Clear the current selection */
  clearSelection: () => void;
  /** Manually set selection (useful for testing) */
  setSelection: (state: SelectedTextState | null) => void;
}

/**
 * Check if the selection is within a valid content container
 */
function isSelectionInContainer(
  selection: Selection,
  containerSelector: string
): boolean {
  if (!selection.rangeCount) return false;

  const range = selection.getRangeAt(0);
  const container = range.commonAncestorContainer;

  // Get the element (handle text nodes)
  const element =
    container.nodeType === Node.TEXT_NODE
      ? container.parentElement
      : (container as Element);

  if (!element) return false;

  // Check if the element or any ancestor matches the selector
  return element.closest(containerSelector) !== null;
}

/**
 * Get clean text from selection, handling edge cases
 */
function getCleanSelectedText(selection: Selection, maxLength: number): string {
  const text = selection.toString().trim();

  // Normalize whitespace
  const normalized = text
    .replace(/\s+/g, ' ')
    .replace(/[\r\n]+/g, ' ')
    .trim();

  // Truncate if too long
  if (normalized.length > maxLength) {
    return normalized.slice(0, maxLength) + '...';
  }

  return normalized;
}

/**
 * Get the bounding rectangle for the selection
 */
function getSelectionRect(selection: Selection): DOMRect | null {
  if (!selection.rangeCount) return null;

  const range = selection.getRangeAt(0);
  const rect = range.getBoundingClientRect();

  // Return null if rect has no dimensions (collapsed selection)
  if (rect.width === 0 && rect.height === 0) {
    return null;
  }

  return rect;
}

/**
 * Hook for capturing text selection from document content
 *
 * @param options - Configuration options
 * @returns Selected text state and control functions
 *
 * @example
 * ```tsx
 * const { selectedText, clearSelection } = useSelectedText({
 *   minLength: 5,
 *   containerSelector: '.docs-content'
 * });
 *
 * if (selectedText?.isActive) {
 *   // Show tooltip at selectedText.rect position
 *   // Use selectedText.text for context
 * }
 * ```
 */
export function useSelectedText(
  options: UseSelectedTextOptions = {}
): UseSelectedTextReturn {
  const opts = { ...DEFAULT_OPTIONS, ...options };
  const [selectedText, setSelectedText] = useState<SelectedTextState | null>(null);
  const timeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  // Clear selection handler
  const clearSelection = useCallback(() => {
    setSelectedText(null);
    // Also clear browser selection
    window.getSelection()?.removeAllRanges();
  }, []);

  // Check if an element is inside a chat component
  const isInsideChatElement = useCallback((element: Element | null): boolean => {
    if (!element) return false;
    return !!(
      element.closest('[data-chat-element]') ||
      element.closest('[class*="chatWidget"]') ||
      element.closest('[class*="chatPanel"]') ||
      element.closest('[class*="selectionTooltip"]')
    );
  }, []);

  // Handle selection change with debouncing
  const handleSelectionChange = useCallback((event?: Event) => {
    if (!opts.enabled) return;

    // Don't process selection changes if the event originated from within chat elements
    // This prevents clearing the selectedText when clicking on the chat input
    if (event?.target && isInsideChatElement(event.target as Element)) {
      return;
    }

    // Clear any pending timeout
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }

    timeoutRef.current = setTimeout(() => {
      const selection = window.getSelection();

      // No selection or empty selection
      if (!selection || selection.isCollapsed) {
        // Don't clear if we're inside a chat element (preserve selected text context)
        const activeElement = document.activeElement;
        if (activeElement && isInsideChatElement(activeElement)) {
          return;
        }
        setSelectedText(null);
        return;
      }

      // Check if selection is in valid container
      if (!isSelectionInContainer(selection, opts.containerSelector)) {
        setSelectedText(null);
        return;
      }

      const text = getCleanSelectedText(selection, opts.maxLength);

      // Check minimum length
      if (text.length < opts.minLength) {
        setSelectedText(null);
        return;
      }

      const rect = getSelectionRect(selection);

      setSelectedText({
        text,
        rect,
        isActive: true,
      });
    }, opts.debounceMs);
  }, [opts.enabled, opts.containerSelector, opts.maxLength, opts.minLength, opts.debounceMs, isInsideChatElement]);

  // Handle click outside to clear selection
  const handleClick = useCallback(
    (event: MouseEvent) => {
      if (!selectedText?.isActive) return;

      // Don't clear if clicking on chat elements
      const target = event.target as Element;
      if (isInsideChatElement(target)) {
        return;
      }

      // Check if the click is on a new selection
      const selection = window.getSelection();
      if (selection && !selection.isCollapsed) {
        // Let the selection change handler deal with it
        return;
      }

      // Clear selection after a short delay to allow for new selections
      setTimeout(() => {
        const currentSelection = window.getSelection();
        if (!currentSelection || currentSelection.isCollapsed) {
          setSelectedText(null);
        }
      }, 50);
    },
    [selectedText?.isActive, isInsideChatElement]
  );

  // Set up event listeners
  useEffect(() => {
    if (!opts.enabled || typeof window === 'undefined') return;

    // Listen for selection changes on mouseup (more reliable than selectionchange)
    document.addEventListener('mouseup', handleSelectionChange);
    document.addEventListener('keyup', handleSelectionChange);
    document.addEventListener('click', handleClick);

    return () => {
      document.removeEventListener('mouseup', handleSelectionChange);
      document.removeEventListener('keyup', handleSelectionChange);
      document.removeEventListener('click', handleClick);

      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [opts.enabled, handleSelectionChange, handleClick]);

  return {
    selectedText,
    clearSelection,
    setSelection: setSelectedText,
  };
}

export default useSelectedText;
