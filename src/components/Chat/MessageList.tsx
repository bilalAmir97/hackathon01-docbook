/**
 * MessageList - Displays the conversation history
 * Handles user/assistant message differentiation and auto-scroll
 */

import React, { useEffect, useRef, useState, useCallback } from 'react';
import type { Message, Citation } from '../../types/chat';
import { CitationList } from './CitationList';
import { ResponseSelectionTooltip } from './ResponseSelectionTooltip';
import { SourcesSidebar } from './SourcesSidebar';
import styles from './chat.module.css';

/**
 * Props for the MessageList component
 */
interface MessageListProps {
  /** Array of messages to display */
  messages: Message[];
  /** Whether a response is being loaded */
  isLoading?: boolean;
  /** Callback when user wants to add text to follow-up */
  onAddToFollowUp?: (text: string) => void;
}

/**
 * Props for individual message items
 */
interface MessageItemProps {
  message: Message;
  /** Ref to attach to the message body for selection detection */
  bodyRef?: React.RefObject<HTMLDivElement>;
}

/**
 * User icon SVG
 */
function UserIcon(): JSX.Element {
  return (
    <svg
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="currentColor"
      aria-hidden="true"
    >
      <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
    </svg>
  );
}

/**
 * Assistant icon SVG
 */
function AssistantIcon(): JSX.Element {
  return (
    <svg
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="currentColor"
      aria-hidden="true"
    >
      <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z" />
    </svg>
  );
}

/**
 * Typing indicator animation
 */
function TypingIndicator(): JSX.Element {
  return (
    <div className={styles.typingIndicator} aria-label="Assistant is typing">
      <span className={styles.typingDot} />
      <span className={styles.typingDot} />
      <span className={styles.typingDot} />
    </div>
  );
}

/**
 * Format message content with basic markdown support
 */
function formatContent(content: string): JSX.Element {
  // Split by code blocks first
  const parts = content.split(/(```[\s\S]*?```|`[^`]+`)/g);

  return (
    <>
      {parts.map((part, index) => {
        // Code block
        if (part.startsWith('```')) {
          const codeContent = part.replace(/```(\w*)\n?/, '').replace(/```$/, '');
          return (
            <pre key={index} className={styles.messageCodeBlock}>
              <code>{codeContent}</code>
            </pre>
          );
        }

        // Inline code
        if (part.startsWith('`') && part.endsWith('`')) {
          return (
            <code key={index} className={styles.messageInlineCode}>
              {part.slice(1, -1)}
            </code>
          );
        }

        // Regular text - preserve line breaks
        return (
          <span key={index}>
            {part.split('\n').map((line, lineIndex, arr) => (
              <React.Fragment key={lineIndex}>
                {line}
                {lineIndex < arr.length - 1 && <br />}
              </React.Fragment>
            ))}
          </span>
        );
      })}
    </>
  );
}

/**
 * Individual message item component
 */
function MessageItem({ message, bodyRef }: MessageItemProps): JSX.Element {
  const isUser = message.role === 'user';
  const isStreaming = message.status === 'streaming';
  const isError = message.status === 'error';

  return (
    <div
      className={`${styles.messageItem} ${isUser ? styles.messageUser : styles.messageAssistant}`}
      data-status={message.status}
    >
      <div className={styles.messageAvatar}>
        {isUser ? <UserIcon /> : <AssistantIcon />}
      </div>

      <div className={styles.messageContent}>
        <div className={styles.messageHeader}>
          <span className={styles.messageRole}>
            {isUser ? 'You' : 'Assistant'}
          </span>
          {message.mode === 'selected_text' && (
            <span className={styles.messageModeBadge}>Context</span>
          )}
        </div>

        <div
          ref={!isUser ? bodyRef : undefined}
          className={styles.messageBody}
          data-message-body={!isUser ? 'assistant' : 'user'}
        >
          {isError ? (
            <div className={styles.messageError}>
              <span className={styles.messageErrorIcon}>!</span>
              {message.errorMessage || 'An error occurred'}
            </div>
          ) : (
            formatContent(message.content)
          )}

          {/* Show typing indicator for streaming messages with no content yet */}
          {isStreaming && !message.content && <TypingIndicator />}

          {/* Show cursor for streaming messages */}
          {isStreaming && message.content && (
            <span className={styles.streamingCursor} aria-hidden="true" />
          )}
        </div>

        {/* Citations for assistant messages */}
        {!isUser && message.sources && message.sources.length > 0 && (
          <CitationList citations={message.sources} />
        )}
      </div>
    </div>
  );
}

/**
 * Empty state component
 */
function EmptyState(): JSX.Element {
  return (
    <div className={styles.emptyState}>
      <div className={styles.emptyStateIcon}>
        <AssistantIcon />
      </div>
      <h3 className={styles.emptyStateTitle}>Ask me anything</h3>
      <p className={styles.emptyStateDescription}>
        I can help you understand ROS 2 concepts, explain code examples, and
        answer questions about the documentation.
      </p>
      <div className={styles.emptyStateSuggestions}>
        <p className={styles.emptyStateSuggestionsLabel}>Try asking:</p>
        <ul>
          <li>What is a ROS 2 node?</li>
          <li>How do publishers and subscribers work?</li>
          <li>Explain the difference between topics and services</li>
        </ul>
      </div>
    </div>
  );
}

/**
 * Message list component
 * Displays conversation history with auto-scroll
 *
 * @example
 * ```tsx
 * <MessageList messages={messages} isLoading={isLoading} />
 * ```
 */
export function MessageList({
  messages,
  isLoading = false,
  onAddToFollowUp,
}: MessageListProps): JSX.Element {
  const listRef = useRef<HTMLDivElement>(null);
  const bottomRef = useRef<HTMLDivElement>(null);
  const messageBodyRef = useRef<HTMLDivElement>(null);

  // State for sources sidebar
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [sidebarSources, setSidebarSources] = useState<Citation[]>([]);
  const [sidebarSelectedText, setSidebarSelectedText] = useState<string>('');

  // Get all sources from assistant messages
  const allSources = messages
    .filter((m) => m.role === 'assistant' && m.sources)
    .flatMap((m) => m.sources || []);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // Handle "Add to Follow-Up" action
  const handleAddToFollowUp = useCallback(
    (text: string) => {
      onAddToFollowUp?.(text);
    },
    [onAddToFollowUp]
  );

  // Handle "Check Sources" action
  const handleCheckSources = useCallback(
    (text: string) => {
      setSidebarSelectedText(text);
      setSidebarSources(allSources);
      setSidebarOpen(true);
    },
    [allSources]
  );

  // Close sidebar
  const handleCloseSidebar = useCallback(() => {
    setSidebarOpen(false);
  }, []);

  if (messages.length === 0) {
    return <EmptyState />;
  }

  return (
    <>
      <div
        ref={listRef}
        className={styles.messageList}
        role="log"
        aria-live="polite"
        aria-label="Chat messages"
      >
        {messages.map((message, index) => (
          <MessageItem
            key={message.id}
            message={message}
            bodyRef={
              message.role === 'assistant' &&
              index === messages.length - 1
                ? messageBodyRef
                : undefined
            }
          />
        ))}

        {/* Loading indicator when waiting for response */}
        {isLoading &&
          messages.length > 0 &&
          messages[messages.length - 1].role === 'user' && (
            <div className={`${styles.messageItem} ${styles.messageAssistant}`}>
              <div className={styles.messageAvatar}>
                <AssistantIcon />
              </div>
              <div className={styles.messageContent}>
                <div className={styles.messageBody}>
                  <TypingIndicator />
                </div>
              </div>
            </div>
          )}

        {/* Scroll anchor */}
        <div ref={bottomRef} />
      </div>

      {/* Response selection tooltip */}
      <ResponseSelectionTooltip
        containerRef={listRef}
        hasSources={allSources.length > 0}
        onAddToFollowUp={handleAddToFollowUp}
        onCheckSources={handleCheckSources}
      />

      {/* Sources sidebar */}
      <SourcesSidebar
        isOpen={sidebarOpen}
        onClose={handleCloseSidebar}
        sources={sidebarSources}
        selectedText={sidebarSelectedText}
      />
    </>
  );
}

export default MessageList;
