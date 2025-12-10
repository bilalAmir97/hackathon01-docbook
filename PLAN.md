/sp.plan

Project: Futuristic Homepage UI - Physical AI Textbook (Feature: 001-futuristic-homepage-ui)
Stack: Docusaurus v3+, TypeScript, GSAP 3.12+, SVG animations

---

ARCHITECTURE SKETCH

Components:
- Homepage/index.tsx → layout orchestrator
- HeroSection.tsx → 60/40 split, headline + CTA + stats
- HumanoidDiagram.tsx → SVG + 3 neural pathways + pulse animations
- ModuleCards.tsx → 2×2 grid, glassmorphism, scroll-trigger
- Footer/index.tsx → 3-column links

Animation flow: Hero timeline (sequential) → ScrollTrigger cards (viewport) → continuous diagram loops
Performance: Lazy-load SVG, will-change CSS, RAF-based pulses, GSAP cleanup on unmount

---

SECTION STRUCTURE (Phases)

Phase 1 - Foundation (Day 1): GSAP setup, color palette CSS, fonts, folder structure
Phase 2 - Hero (Day 1-2): 60/40 layout, headline with cyan underline, dual CTAs, diagram placeholder
Phase 3 - Animations (Day 2-3): Headline stagger, underline draw, diagram pulses/breathing, node lighting
Phase 4 - Module Cards (Day 3): Grid layout, glassmorphism, hover effects, ScrollTrigger fade-up
Phase 5 - Footer + Responsive (Day 4): 3-column layout, breakpoints (1200px/768px), mobile optimizations, prefers-reduced-motion
Phase 6 - Testing (Day 4-5): 60fps validation, accessibility audit, cross-browser checks, lazy-load optimization

---

RESEARCH APPROACH (Concurrent)

Per-phase checkpoints:
- Phase 1: Docusaurus CSS injection, font loading optimization
- Phase 3: GSAP drawSVG/morphSVG patterns, will-change performance
- Phase 4: backdrop-filter support (caniuse), ScrollTrigger mobile behavior
- Phase 5: prefers-reduced-motion detection, Docusaurus routing
- Phase 6: Lighthouse thresholds, axe DevTools automation

Citation: Inline refs to GSAP docs, MDN, Docusaurus official docs (APA style)

---

KEY DECISIONS & TRADEOFFS

1. **SVG vs Canvas for diagram**: Choose SVG (accessibility, DOM control, scalability) over Canvas (performance) or WebGL (overkill)
2. **Animation architecture**: Hybrid → master timeline for hero, individual tweens for scroll-triggered cards
3. **Icon library**: Custom SVG sprites (brand-accurate ROS 2/Isaac icons) over React Icons (limited) or images (no customization)
4. **Mobile animations**: Disable ScrollTrigger, keep hover/tap → balances performance + interactivity

---

TESTING STRATEGY

Validation checks:
- **Functional**: FR-001 to FR-016 → visual regression (Percy), RTL tests for headline/cards/footer
- **Performance**: Chrome DevTools → 60fps check, Lighthouse ≥90 score, 4G throttle test
- **Accessibility**: axe DevTools → 0 violations, keyboard nav, NVDA screen reader, reduced-motion toggle
- **Responsive**: Test 1920px/1200px/768px/375px → verify hero stacking, card grid collapse
- **Cross-browser**: Chrome/Firefox/Safari → SVG animations, backdrop-filter fallbacks

Edge cases: Slow network (lazy-load), no JS (semantic HTML), high contrast mode (cyan visibility)

Quality gates: 60fps, <3s load, Lighthouse ≥90/95/90, 0 axe violations, <150KB gzipped, TS strict mode

---

OUTPUT DELIVERABLES

1. Daily task breakdown (6 phases)
2. Component + animation flow diagram
3. Research notes (concurrent findings, APA citations)
4. Testing checklist (mapped to US-001/002/003)
5. Risk mitigation plan (performance, browser compat)
