/**
 * CitationList - Displays source citations for assistant responses
 * Shows clickable links to source documents with relevance scores
 */

import React, { useState } from 'react';
import type { Citation } from '../../types/chat';
import styles from './chat.module.css';

/**
 * Props for the CitationList component
 */
interface CitationListProps {
  /** Array of citations to display */
  citations: Citation[];
  /** Maximum citations to show before "show more" */
  maxVisible?: number;
}

/**
 * Props for individual citation items
 */
interface CitationItemProps {
  citation: Citation;
  index: number;
}

/**
 * Link icon SVG
 */
function LinkIcon(): JSX.Element {
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
      <path d="M10 13a5 5 0 0 0 7.54.54l3-3a5 5 0 0 0-7.07-7.07l-1.72 1.71" />
      <path d="M14 11a5 5 0 0 0-7.54-.54l-3 3a5 5 0 0 0 7.07 7.07l1.71-1.71" />
    </svg>
  );
}

/**
 * Chevron down icon SVG
 */
function ChevronDownIcon(): JSX.Element {
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
      <polyline points="6 9 12 15 18 9" />
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
 * Truncate text to a maximum length
 */
function truncateText(text: string, maxLength: number): string {
  if (text.length <= maxLength) return text;
  return text.slice(0, maxLength).trim() + '...';
}

/**
 * Individual citation item component
 */
function CitationItem({ citation, index }: CitationItemProps): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(false);
  const relevanceLevel = getRelevanceLevel(citation.relevanceScore);

  // Determine if this is an internal or external link
  const isInternalLink =
    citation.sourceUrl.startsWith('/') ||
    citation.sourceUrl.includes(window.location.hostname);

  return (
    <div className={styles.citationItem} data-relevance={relevanceLevel}>
      <div className={styles.citationHeader}>
        <span className={styles.citationIndex}>[{index + 1}]</span>
        <a
          href={citation.sourceUrl}
          className={styles.citationLink}
          target={isInternalLink ? undefined : '_blank'}
          rel={isInternalLink ? undefined : 'noopener noreferrer'}
        >
          <LinkIcon />
          <span className={styles.citationTitle}>
            {citation.pageTitle || 'Source'}
          </span>
        </a>
        <span
          className={styles.citationRelevance}
          title={`Relevance: ${formatRelevance(citation.relevanceScore)}`}
        >
          {formatRelevance(citation.relevanceScore)}
        </span>
      </div>

      {citation.sectionHeading && (
        <div className={styles.citationSection}>
          {citation.sectionHeading}
        </div>
      )}

      {citation.chunkText && (
        <div className={styles.citationPreview}>
          <button
            type="button"
            onClick={() => setIsExpanded(!isExpanded)}
            className={styles.citationExpandButton}
            aria-expanded={isExpanded}
          >
            <ChevronDownIcon />
            <span>{isExpanded ? 'Hide excerpt' : 'Show excerpt'}</span>
          </button>

          {isExpanded && (
            <blockquote className={styles.citationText}>
              {truncateText(citation.chunkText, 300)}
            </blockquote>
          )}
        </div>
      )}
    </div>
  );
}

/**
 * Citation list component
 * Displays source citations with expandable excerpts
 *
 * @example
 * ```tsx
 * <CitationList citations={message.sources} maxVisible={3} />
 * ```
 */
export function CitationList({
  citations,
  maxVisible = 3,
}: CitationListProps): JSX.Element | null {
  const [showAll, setShowAll] = useState(false);

  if (!citations || citations.length === 0) {
    return null;
  }

  // Filter out "selected_text" sources - user already knows they selected text
  // Only show real documentation sources
  const realCitations = citations.filter(
    (c) => c.sourceUrl !== 'selected_text'
  );

  // Don't render if no real citations remain
  if (realCitations.length === 0) {
    return null;
  }

  // Sort by relevance score
  const sortedCitations = [...realCitations].sort(
    (a, b) => b.relevanceScore - a.relevanceScore
  );

  const visibleCitations = showAll
    ? sortedCitations
    : sortedCitations.slice(0, maxVisible);

  const hiddenCount = sortedCitations.length - maxVisible;

  return (
    <div className={styles.citationList} aria-label="Sources">
      <div className={styles.citationListHeader}>
        <span className={styles.citationListTitle}>Sources</span>
        <span className={styles.citationListCount}>
          {realCitations.length} {realCitations.length === 1 ? 'source' : 'sources'}
        </span>
      </div>

      <div className={styles.citationListItems}>
        {visibleCitations.map((citation, index) => (
          <CitationItem
            key={`${citation.sourceUrl}-${index}`}
            citation={citation}
            index={index}
          />
        ))}
      </div>

      {hiddenCount > 0 && !showAll && (
        <button
          type="button"
          onClick={() => setShowAll(true)}
          className={styles.citationShowMore}
        >
          Show {hiddenCount} more {hiddenCount === 1 ? 'source' : 'sources'}
        </button>
      )}

      {showAll && hiddenCount > 0 && (
        <button
          type="button"
          onClick={() => setShowAll(false)}
          className={styles.citationShowMore}
        >
          Show less
        </button>
      )}
    </div>
  );
}

export default CitationList;
