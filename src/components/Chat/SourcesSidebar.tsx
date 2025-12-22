/**
 * SourcesSidebar - Sliding panel that displays all sources for a message
 * Opens when user clicks "Check Sources" from the response selection tooltip
 */

import React, { useEffect, useRef, useCallback } from 'react';
import type { Citation } from '../../types/chat';
import styles from './chat.module.css';

/**
 * Props for the SourcesSidebar component
 */
interface SourcesSidebarProps {
  /** Whether the sidebar is open */
  isOpen: boolean;
  /** Callback to close the sidebar */
  onClose: () => void;
  /** Sources to display */
  sources: Citation[];
  /** The selected text that triggered the sidebar (optional) */
  selectedText?: string;
}

/**
 * Close icon
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
 * External link icon
 */
function ExternalLinkIcon(): JSX.Element {
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
      <path d="M18 13v6a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h6" />
      <polyline points="15 3 21 3 21 9" />
      <line x1="10" y1="14" x2="21" y2="3" />
    </svg>
  );
}

/**
 * Format relevance score as percentage
 */
function formatRelevance(score: number): string {
  return `${Math.round(score * 100)}%`;
}

/**
 * Get relevance level for styling
 */
function getRelevanceLevel(score: number): 'high' | 'medium' | 'low' {
  if (score >= 0.8) return 'high';
  if (score >= 0.6) return 'medium';
  return 'low';
}

/**
 * SourcesSidebar component
 */
export function SourcesSidebar({
  isOpen,
  onClose,
  sources,
  selectedText,
}: SourcesSidebarProps): JSX.Element | null {
  const sidebarRef = useRef<HTMLDivElement>(null);

  // Filter out "selected_text" sources
  const realSources = sources.filter((s) => s.sourceUrl !== 'selected_text');

  // Close on escape key
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isOpen, onClose]);

  // Close when clicking outside
  const handleBackdropClick = useCallback(
    (e: React.MouseEvent) => {
      if (e.target === e.currentTarget) {
        onClose();
      }
    },
    [onClose]
  );

  // Determine if link is internal
  const isInternalLink = (url: string): boolean => {
    return url.startsWith('/') || url.includes(window.location.hostname);
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div
      className={styles.sourcesSidebarBackdrop}
      onClick={handleBackdropClick}
      role="dialog"
      aria-modal="true"
      aria-label="Sources panel"
    >
      <div
        ref={sidebarRef}
        className={`${styles.sourcesSidebar} ${isOpen ? styles.sourcesSidebarOpen : ''}`}
        data-chat-element="sources-sidebar"
      >
        {/* Header */}
        <div className={styles.sourcesSidebarHeader}>
          <h3 className={styles.sourcesSidebarTitle}>
            Sources
            <span className={styles.sourcesSidebarCount}>
              {realSources.length}
            </span>
          </h3>
          <button
            type="button"
            onClick={onClose}
            className={styles.sourcesSidebarClose}
            aria-label="Close sources panel"
          >
            <CloseIcon />
          </button>
        </div>

        {/* Selected text context */}
        {selectedText && (
          <div className={styles.sourcesSidebarContext}>
            <span className={styles.sourcesSidebarContextLabel}>
              Selected text:
            </span>
            <blockquote className={styles.sourcesSidebarContextText}>
              "{selectedText.length > 150
                ? selectedText.slice(0, 150) + '...'
                : selectedText}"
            </blockquote>
          </div>
        )}

        {/* Sources list */}
        <div className={styles.sourcesSidebarContent}>
          {realSources.length === 0 ? (
            <div className={styles.sourcesSidebarEmpty}>
              <p>No documentation sources available for this response.</p>
              <p className={styles.sourcesSidebarEmptyHint}>
                This response was based on the selected text context only.
              </p>
            </div>
          ) : (
            <ul className={styles.sourcesSidebarList}>
              {realSources.map((source, index) => (
                <li
                  key={`${source.sourceUrl}-${index}`}
                  className={styles.sourcesSidebarItem}
                  data-relevance={getRelevanceLevel(source.relevanceScore)}
                >
                  <div className={styles.sourcesSidebarItemHeader}>
                    <span className={styles.sourcesSidebarItemIndex}>
                      [{index + 1}]
                    </span>
                    <a
                      href={source.sourceUrl}
                      className={styles.sourcesSidebarItemLink}
                      target={isInternalLink(source.sourceUrl) ? undefined : '_blank'}
                      rel={isInternalLink(source.sourceUrl) ? undefined : 'noopener noreferrer'}
                    >
                      {source.pageTitle || 'Source'}
                      {!isInternalLink(source.sourceUrl) && (
                        <ExternalLinkIcon />
                      )}
                    </a>
                    <span
                      className={styles.sourcesSidebarItemRelevance}
                      data-level={getRelevanceLevel(source.relevanceScore)}
                    >
                      {formatRelevance(source.relevanceScore)}
                    </span>
                  </div>

                  {source.sectionHeading && (
                    <div className={styles.sourcesSidebarItemSection}>
                      {source.sectionHeading}
                    </div>
                  )}

                  {source.chunkText && (
                    <blockquote className={styles.sourcesSidebarItemText}>
                      {source.chunkText.length > 300
                        ? source.chunkText.slice(0, 300) + '...'
                        : source.chunkText}
                    </blockquote>
                  )}
                </li>
              ))}
            </ul>
          )}
        </div>
      </div>
    </div>
  );
}

export default SourcesSidebar;
