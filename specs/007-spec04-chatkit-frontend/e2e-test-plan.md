# E2E Test Plan: Docusaurus ChatKit Frontend

**Feature**: 007-spec04-chatkit-frontend
**Date**: 2025-12-17

---

## Test Environment Setup

### Prerequisites
1. Backend running at `http://localhost:8000`
2. Frontend running at `http://localhost:3000`
3. Browser with DevTools open (Console tab)

### Start Commands
```bash
# Terminal 1: Backend
cd backend
uvicorn api:app --reload --port 8000

# Terminal 2: Frontend
npm start
```

---

## T026: Chat Widget Renders

**Status**: Ready for Manual Verification

### Steps
1. Navigate to `http://localhost:3000`
2. Look for floating chat button in bottom-right corner
3. Navigate to any documentation page
4. Verify button persists

### Expected Results
- [ ] Chat FAB button visible on homepage
- [ ] Chat FAB button visible on documentation pages
- [ ] Button has chat icon
- [ ] Button is clickable

---

## T027: Chat Opens and Closes

**Status**: Ready for Manual Verification

### Steps
1. Click the chat FAB button
2. Verify panel slides in from right
3. Click the X close button
4. Press Escape key when panel is open
5. Click FAB again to reopen

### Expected Results
- [ ] Panel opens on FAB click
- [ ] Panel has header with "AI Assistant" title
- [ ] Panel closes on X button click
- [ ] Panel closes on Escape key
- [ ] Smooth animation on open/close

---

## T028: Message Streaming

**Status**: Ready for Manual Verification (requires backend)

### Steps
1. Open chat panel
2. Type "What is ROS 2?"
3. Click send or press Enter
4. Watch response appear

### Expected Results
- [ ] User message appears immediately
- [ ] Loading indicator shows (typing dots)
- [ ] Response text streams in progressively
- [ ] Citations appear after response completes
- [ ] Loading indicator disappears when complete

---

## T029: No CORS Errors

**Status**: Ready for Manual Verification

### Steps
1. Open browser DevTools Console
2. Send a chat message
3. Check console for errors

### Expected Results
- [ ] No "Access-Control-Allow-Origin" errors
- [ ] No "CORS policy" errors
- [ ] Network tab shows 200 responses
- [ ] SSE stream connects successfully

---

## T030: Citation Navigation

**Status**: Ready for Manual Verification

### Steps
1. Get a response with citations
2. Click on a citation link
3. Observe page navigation

### Expected Results
- [ ] Citation links are clickable
- [ ] Internal links navigate without full reload (SPA)
- [ ] Chat panel remains open after navigation
- [ ] Correct documentation page loads

---

## T031: Selected Text Mode

**Status**: Ready for Manual Verification

### Steps
1. Go to a documentation page with content
2. Select some text by clicking and dragging
3. Look for "Ask about this" tooltip
4. Click the tooltip
5. Type a question about the selection

### Expected Results
- [ ] Tooltip appears near text selection
- [ ] Clicking tooltip opens chat panel
- [ ] Selected text badge shows in chat
- [ ] Badge shows truncated preview
- [ ] Clear button removes selection
- [ ] Response is grounded in selected text

---

## T032: Session Persistence

**Status**: Ready for Manual Verification

### Steps
1. Send a message and receive response
2. Check localStorage for `chatkit_session_id`
3. Refresh the page (F5)
4. Open chat panel
5. Verify previous messages appear

### Expected Results
- [ ] Session ID saved in localStorage
- [ ] Messages persist after refresh
- [ ] Chat history loads on panel open
- [ ] Messages in correct order (oldest first)

---

## T033: New Conversation

**Status**: Ready for Manual Verification

### Steps
1. Have a conversation with multiple messages
2. Click the trash/clear button in header
3. Confirm the dialog
4. Check localStorage for new session ID

### Expected Results
- [ ] Confirmation dialog appears
- [ ] Messages cleared from UI
- [ ] New session ID generated
- [ ] Previous session ID replaced
- [ ] Can start fresh conversation

---

## Summary Checklist

| Test | Status | Notes |
|------|--------|-------|
| T026 | ⏳ Pending | Requires `npm start` |
| T027 | ⏳ Pending | Requires `npm start` |
| T028 | ⏳ Pending | Requires backend + frontend |
| T029 | ⏳ Pending | Requires backend + frontend |
| T030 | ⏳ Pending | Requires backend + frontend |
| T031 | ⏳ Pending | Requires `npm start` |
| T032 | ⏳ Pending | Requires backend + frontend |
| T033 | ⏳ Pending | Requires `npm start` |

---

## Automated Test Future Work

These manual tests could be automated using:
- **Playwright** or **Cypress** for browser automation
- **Mock Service Worker (MSW)** for API mocking
- **Vitest** or **Jest** for unit tests

Example Playwright test:
```typescript
import { test, expect } from '@playwright/test';

test('chat widget renders on all pages', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('[data-chat-element="toggle"]')).toBeVisible();

  await page.goto('/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts');
  await expect(page.locator('[data-chat-element="toggle"]')).toBeVisible();
});
```
