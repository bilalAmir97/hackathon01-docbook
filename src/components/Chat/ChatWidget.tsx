/**
 * ChatWidget - Main container component for the chat interface
 * Combines all chat components into a single widget
 */

import React, { useCallback } from 'react';
import { useChatContext } from '../../contexts/ChatContext';
import { ChatToggle } from './ChatToggle';
import { ChatPanel } from './ChatPanel';
import { SelectionTooltip } from './SelectionTooltip';
import { ChatErrorBoundary } from './ErrorFallback';
import styles from './chat.module.css';

/**
 * Props for the ChatWidget component
 */
interface ChatWidgetProps {
  /** Custom class name for the widget container */
  className?: string;
  /** Position of the widget */
  position?: 'bottom-right' | 'bottom-left';
}

/**
 * Main chat widget component
 * Renders the FAB toggle, chat panel, and selection tooltip
 *
 * @example
 * ```tsx
 * // In your Root.tsx or layout
 * <ChatProvider>
 *   <ChatWidget position="bottom-right" />
 *   {children}
 * </ChatProvider>
 * ```
 */
export function ChatWidget({
  className = '',
  position = 'bottom-right',
}: ChatWidgetProps): JSX.Element {
  const { selectedText, openChat, setSelectedText } = useChatContext();

  // Handle "Ask about this" from selection tooltip
  const handleAskAbout = useCallback(
    (text: string) => {
      // Set the selected text in context
      setSelectedText({
        text,
        rect: null,
        isActive: true,
      });
      // Open the chat panel
      openChat();
    },
    [openChat, setSelectedText]
  );

  return (
    <ChatErrorBoundary>
      <div
        className={`${styles.chatWidget} ${styles[`chatWidget${position === 'bottom-left' ? 'Left' : 'Right'}`]} ${className}`}
        data-chat-element="widget"
      >
        {/* Selection tooltip - appears when text is selected */}
        <SelectionTooltip
          selectedText={selectedText}
          onAskAbout={handleAskAbout}
        />

        {/* Chat panel */}
        <ChatPanel />

        {/* Floating action button */}
        <ChatToggle />
      </div>
    </ChatErrorBoundary>
  );
}

export default ChatWidget;
