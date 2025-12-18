/**
 * Chat component exports
 * Barrel file for easy importing
 */

// Main widget
export { ChatWidget } from './ChatWidget';

// Individual components
export { ChatToggle } from './ChatToggle';
export { ChatPanel } from './ChatPanel';
export { MessageList } from './MessageList';
export { MessageInput } from './MessageInput';
export { CitationList } from './CitationList';
export { SelectedTextBadge } from './SelectedTextBadge';
export { SelectionTooltip } from './SelectionTooltip';
export { ErrorFallback, ChatErrorBoundary } from './ErrorFallback';

// Re-export context for convenience
export { ChatProvider, useChatContext } from '../../contexts/ChatContext';

// Re-export types
export type {
  Message,
  Citation,
  ChatRequest,
  ChatError,
  StreamEvent,
  SelectedTextState,
  ChatState,
  ChatActions,
  ChatContextType,
} from '../../types/chat';
