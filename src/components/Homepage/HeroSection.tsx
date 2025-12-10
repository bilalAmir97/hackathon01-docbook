import React, { useEffect } from 'react';
import { initScrollAnimations, prefersReducedMotion, initPerformanceAwareAnimations, initReducedAnimationMode } from '../../utils/animation-helpers';

const HeroSection: React.FC = () => {
  // Split the headline into words for sequential animation
  const headlineWords = "Master robots that bridge digital intelligence and physical reality.".split(" ");

  useEffect(() => {
    // Initialize scroll animations
    initScrollAnimations('.hero-section', 'animate-in');

    // Initialize performance-aware animations
    initPerformanceAwareAnimations((shouldReduce) => {
      initReducedAnimationMode(shouldReduce);
    });

    // Handle prefers-reduced-motion changes
    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
    const handleReducedMotionChange = (e: MediaQueryListEvent) => {
      if (e.matches) {
        document.body.classList.add('reduce-animation');
      } else {
        document.body.classList.remove('reduce-animation');
      }
    };

    mediaQuery.addEventListener('change', handleReducedMotionChange);

    return () => {
      mediaQuery.removeEventListener('change', handleReducedMotionChange);
    };
  }, []);

  // Handle keyboard navigation for the CTA buttons
  const handleKeyDown = (e: React.KeyboardEvent<HTMLButtonElement>, action: string) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      // Trigger the appropriate action
      if (action === 'start-module') {
        // Navigate to Module 1
        window.location.href = '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts';
      } else if (action === 'browse-modules') {
        // Navigate to modules page
        window.location.href = '/tutorial';
      }
    }
  };

  return (
    <section className="hero-section" aria-labelledby="hero-headline">
      <div className="hero-content">
        <h1 id="hero-headline" className="hero-headline" tabIndex={0}>
          {headlineWords.map((word, index) => (
            <span key={index} className="word" aria-hidden="true">
              {word === "bridge" ? (
                <span className="highlight" aria-label="bridge (highlighted)">{word}</span>
              ) : (
                word
              )}
              {index < headlineWords.length - 1 ? " " : ""}
            </span>
          ))}
        </h1>
        <p className="hero-subheadline" tabIndex={0}>
          Learn ROS 2, simulation, and humanoid control to build the next generation of intelligent robots.
        </p>
        <div className="hero-cta-buttons" role="group" aria-label="Primary navigation actions">
          <button
            className="cta-button primary-cta focusable"
            aria-label="Get Started with Module 1"
            tabIndex={0}
            onKeyDown={(e) => handleKeyDown(e, 'start-module')}
            onClick={() => window.location.href = '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts'}
          >
            Get Started with Module 1
          </button>
          <button
            className="cta-button secondary-cta focusable"
            aria-label="Browse All Modules"
            tabIndex={0}
            onKeyDown={(e) => handleKeyDown(e, 'browse-modules')}
            onClick={() => window.location.href = '/docs/'}
          >
            Browse All Modules
          </button>
        </div>
      </div>
      <div className="hero-background-glow" aria-hidden="true"></div>
    </section>
  );
};

export default HeroSection;