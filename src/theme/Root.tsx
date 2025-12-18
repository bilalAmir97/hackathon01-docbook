/**
 * Root.tsx - Swizzled Docusaurus theme component
 * Wraps the entire application with the ChatProvider
 * This enables the chat widget to be available globally on all pages
 */

import React, { type ReactNode } from 'react';
import { ChatProvider, ChatWidget } from '../components/Chat';

/**
 * Props for the Root component
 */
interface RootProps {
  children: ReactNode;
}

/**
 * Root component that wraps the entire Docusaurus application
 * Provides the chat context and renders the chat widget
 *
 * This is a "swizzled" component that Docusaurus uses as the root wrapper.
 * See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <ChatProvider>
      {/* Main application content */}
      {children}

      {/* Chat widget - positioned fixed in bottom-right */}
      <ChatWidget position="bottom-right" />
    </ChatProvider>
  );
}
