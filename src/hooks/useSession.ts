/**
 * useSession hook for managing chat session IDs
 * Persists session across page refreshes using localStorage
 */

import { useState, useEffect, useCallback } from 'react';

const SESSION_STORAGE_KEY = 'chatkit_session_id';
const SESSION_EXPIRY_KEY = 'chatkit_session_expiry';
const SESSION_DURATION_MS = 24 * 60 * 60 * 1000; // 24 hours

/**
 * Generate a unique session ID using crypto.randomUUID() with fallback
 */
function generateSessionId(): string {
  // Try crypto.randomUUID() first (modern browsers)
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }

  // Fallback for older browsers
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Check if running in browser environment
 */
function isBrowser(): boolean {
  return typeof window !== 'undefined' && typeof localStorage !== 'undefined';
}

/**
 * Get stored session if valid
 */
function getStoredSession(): string | null {
  if (!isBrowser()) return null;

  try {
    const sessionId = localStorage.getItem(SESSION_STORAGE_KEY);
    const expiryStr = localStorage.getItem(SESSION_EXPIRY_KEY);

    if (!sessionId || !expiryStr) {
      return null;
    }

    const expiry = parseInt(expiryStr, 10);
    if (isNaN(expiry) || Date.now() > expiry) {
      // Session expired, clear it
      localStorage.removeItem(SESSION_STORAGE_KEY);
      localStorage.removeItem(SESSION_EXPIRY_KEY);
      return null;
    }

    return sessionId;
  } catch (error) {
    console.warn('Failed to read session from localStorage:', error);
    return null;
  }
}

/**
 * Store session ID with expiry
 */
function storeSession(sessionId: string): void {
  if (!isBrowser()) return;

  try {
    const expiry = Date.now() + SESSION_DURATION_MS;
    localStorage.setItem(SESSION_STORAGE_KEY, sessionId);
    localStorage.setItem(SESSION_EXPIRY_KEY, expiry.toString());
  } catch (error) {
    console.warn('Failed to store session in localStorage:', error);
  }
}

/**
 * Clear stored session
 */
function clearStoredSession(): void {
  if (!isBrowser()) return;

  try {
    localStorage.removeItem(SESSION_STORAGE_KEY);
    localStorage.removeItem(SESSION_EXPIRY_KEY);
  } catch (error) {
    console.warn('Failed to clear session from localStorage:', error);
  }
}

/**
 * Hook return type
 */
interface UseSessionReturn {
  /** Current session ID */
  sessionId: string;
  /** Reset the session with a new ID */
  resetSession: () => void;
  /** Check if session is initialized */
  isInitialized: boolean;
}

/**
 * Hook for managing chat session persistence
 *
 * @returns Session ID and management functions
 *
 * @example
 * ```tsx
 * const { sessionId, resetSession } = useSession();
 *
 * // Use sessionId in API requests
 * sendMessage({ query: 'Hello', session_id: sessionId });
 *
 * // Reset session when user wants to start fresh
 * <button onClick={resetSession}>New Conversation</button>
 * ```
 */
export function useSession(): UseSessionReturn {
  const [sessionId, setSessionId] = useState<string>('');
  const [isInitialized, setIsInitialized] = useState(false);

  // Initialize session on mount
  useEffect(() => {
    let id = getStoredSession();

    if (!id) {
      id = generateSessionId();
      storeSession(id);
    }

    setSessionId(id);
    setIsInitialized(true);
  }, []);

  // Reset session with a new ID
  const resetSession = useCallback(() => {
    const newId = generateSessionId();
    storeSession(newId);
    setSessionId(newId);
  }, []);

  return {
    sessionId,
    resetSession,
    isInitialized,
  };
}

export default useSession;
