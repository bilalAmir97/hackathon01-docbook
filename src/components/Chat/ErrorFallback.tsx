/**
 * ErrorFallback - Error boundary fallback component
 * Displays when an error occurs within the chat widget
 */

import React from 'react';
import styles from './chat.module.css';

/**
 * Props for the ErrorFallback component
 */
interface ErrorFallbackProps {
  /** The error that was caught */
  error?: Error;
  /** Function to reset the error state */
  onReset?: () => void;
  /** Custom error message */
  message?: string;
}

/**
 * Alert icon SVG
 */
function AlertIcon(): JSX.Element {
  return (
    <svg
      width="24"
      height="24"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <circle cx="12" cy="12" r="10" />
      <line x1="12" y1="8" x2="12" y2="12" />
      <line x1="12" y1="16" x2="12.01" y2="16" />
    </svg>
  );
}

/**
 * Refresh icon SVG
 */
function RefreshIcon(): JSX.Element {
  return (
    <svg
      width="16"
      height="16"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <polyline points="23 4 23 10 17 10" />
      <polyline points="1 20 1 14 7 14" />
      <path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15" />
    </svg>
  );
}

/**
 * Error fallback component for the chat widget
 * Provides a friendly error message and retry option
 *
 * @example
 * ```tsx
 * <ErrorBoundary fallback={(error, reset) => (
 *   <ErrorFallback error={error} onReset={reset} />
 * )}>
 *   <ChatWidget />
 * </ErrorBoundary>
 * ```
 */
export function ErrorFallback({
  error,
  onReset,
  message = 'Something went wrong with the chat assistant.',
}: ErrorFallbackProps): JSX.Element {
  return (
    <div className={styles.errorFallback} role="alert">
      <div className={styles.errorFallbackIcon}>
        <AlertIcon />
      </div>

      <h3 className={styles.errorFallbackTitle}>Oops!</h3>

      <p className={styles.errorFallbackMessage}>{message}</p>

      {/* Show error details in development */}
      {process.env.NODE_ENV === 'development' && error && (
        <details className={styles.errorFallbackDetails}>
          <summary>Error details</summary>
          <pre className={styles.errorFallbackStack}>
            {error.message}
            {error.stack && (
              <>
                {'\n\n'}
                {error.stack}
              </>
            )}
          </pre>
        </details>
      )}

      {onReset && (
        <button
          type="button"
          onClick={onReset}
          className={styles.errorFallbackButton}
        >
          <RefreshIcon />
          <span>Try again</span>
        </button>
      )}
    </div>
  );
}

/**
 * Simple error boundary wrapper for class component compatibility
 */
interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
}

interface ErrorBoundaryProps {
  children: React.ReactNode;
  fallback?: React.ReactNode;
  onError?: (error: Error, errorInfo: React.ErrorInfo) => void;
}

export class ChatErrorBoundary extends React.Component<
  ErrorBoundaryProps,
  ErrorBoundaryState
> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo): void {
    console.error('Chat widget error:', error, errorInfo);
    this.props.onError?.(error, errorInfo);
  }

  handleReset = (): void => {
    this.setState({ hasError: false, error: null });
  };

  render(): React.ReactNode {
    if (this.state.hasError) {
      if (this.props.fallback) {
        return this.props.fallback;
      }

      return (
        <ErrorFallback
          error={this.state.error || undefined}
          onReset={this.handleReset}
        />
      );
    }

    return this.props.children;
  }
}

export default ErrorFallback;
