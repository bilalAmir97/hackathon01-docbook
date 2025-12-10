# ADR-0004: CSS-Only Animations Approach for Homepage UI

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** 001-futuristic-homepage-ui
- **Context:** The Physical AI & Humanoid Robotics textbook homepage requires animated UI elements to create an engaging, futuristic experience. Two primary approaches exist: CSS-only animations with minimal vanilla JavaScript for scroll triggers and 3D tilt effects, or using a JavaScript animation library like GSAP. This decision impacts performance, bundle size, accessibility, and long-term maintenance.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use CSS-only animations with minimal vanilla JavaScript for scroll triggers and 3D tilt effects:
- CSS keyframes for all motion animations (fade-in, slide-up, hover effects)
- Minimal vanilla JavaScript for Intersection Observer API (scroll-triggered animations)
- Minimal vanilla JavaScript for CSS 3D transforms (card tilt effect on hover)
- No external animation libraries (no GSAP, Framer Motion, etc.)
- Pure CSS for all visual effects and transitions

## Consequences

### Positive

- Smaller bundle size with no additional animation library dependencies
- Better performance on lower-end devices (CSS animations run on GPU)
- Improved accessibility with better support for `prefers-reduced-motion` media query
- Simpler debugging and maintenance (no external library API to learn)
- Better compatibility with Docusaurus static site generation
- Faster initial load times without external animation library initialization

### Negative

- More complex CSS code for sophisticated animations compared to library APIs
- Less advanced animation features (no advanced easing, complex timelines, etc.)
- Requires more manual JavaScript for scroll-triggered animations
- Potentially more browser-specific CSS prefixes needed for consistent behavior

## Alternatives Considered

**GSAP Animation Library Approach**: GSAP 3.12+ with ScrollTrigger plugin for all animations
- Why rejected: Larger bundle size, additional dependency, potential compatibility issues with static site generation, over-engineering for simple animations
- Tradeoffs: More powerful animation features but at the cost of complexity and performance

**CSS + JavaScript Animation Library Hybrid**: CSS for simple transitions, lightweight library (like Anime.js) for complex sequences
- Why rejected: Still introduces external dependency without significant benefit over pure CSS for this use case
- Tradeoffs: Slightly smaller than GSAP but still adds unnecessary complexity

## References

- Feature Spec: specs/001-futuristic-homepage-ui/spec.md
- Implementation Plan: specs/001-futuristic-homepage-ui/plan.md
- Related ADRs: ADR-0003-frontend-technology-stack-for-futuristic-homepage.md
- Evaluator Evidence: history/prompts/001-futuristic-homepage-ui/1-futuristic-homepage-ui-spec.spec.prompt.md
