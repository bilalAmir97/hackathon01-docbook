/**
 * Animation utilities for the Futuristic Homepage UI
 * Contains Intersection Observer for scroll-triggered animations and 3D tilt effects
 */

/**
 * Initialize scroll-triggered animations using Intersection Observer
 * @param selector - CSS selector for elements to animate
 * @param animationClass - CSS class to add when element is in view
 * @param options - Intersection Observer options
 */
export const initScrollAnimations = (
  selector: string,
  animationClass: string = 'animate',
  options: IntersectionObserverInit = {
    threshold: 0.3,
    rootMargin: '0px 0px -50px 0px'
  }
): void => {
  const elements = document.querySelectorAll(selector);

  if (elements.length === 0) {
    console.warn(`No elements found for selector: ${selector}`);
    return;
  }

  const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (entry.isIntersecting) {
        entry.target.classList.add(animationClass);
        // Stop observing this element after animation is triggered
        observer.unobserve(entry.target);
      }
    });
  }, options);

  elements.forEach((element) => {
    observer.observe(element);
  });
};

/**
 * Initialize staggered animations for a list of elements
 * @param selector - CSS selector for elements to animate
 * @param animationClass - CSS class to add when element is in view
 * @param staggerDelay - Delay in milliseconds between each element's animation
 */
export const initStaggeredAnimations = (
  selector: string,
  animationClass: string = 'animate',
  staggerDelay: number = 150,
  options: IntersectionObserverInit = {
    threshold: 0.3,
    rootMargin: '0px 0px -50px 0px'
  }
): void => {
  const elements = document.querySelectorAll(selector);

  if (elements.length === 0) {
    console.warn(`No elements found for selector: ${selector}`);
    return;
  }

  const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry, index) => {
      if (entry.isIntersecting) {
        // Add delay based on index for staggered effect
        setTimeout(() => {
          entry.target.classList.add(animationClass);
        }, index * staggerDelay);

        // Stop observing this element after animation is triggered
        observer.unobserve(entry.target);
      }
    });
  }, options);

  elements.forEach((element) => {
    observer.observe(element);
  });
};

/**
 * Initialize 3D tilt effect for elements
 * @param selector - CSS selector for elements to apply 3D tilt
 * @param options - Configuration options for the tilt effect
 */
export const initTiltEffect = (
  selector: string,
  options: {
    maxTilt?: number;
    perspective?: number;
    scale?: number;
    speed?: number;
  } = {}
): void => {
  const {
    maxTilt = 10,
    perspective = 1000,
    scale = 1.05,
    speed = 300
  } = options;

  const elements = document.querySelectorAll(selector);

  if (elements.length === 0) {
    console.warn(`No elements found for selector: ${selector}`);
    return;
  }

  elements.forEach((element) => {
    const el = element as HTMLElement;

    // Store original styles
    const originalTransition = el.style.transition;

    const handleMouseMove = (e: MouseEvent) => {
      const rect = el.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;

      const centerX = rect.width / 2;
      const centerY = rect.height / 2;

      const rotateX = ((y - centerY) / centerY) * maxTilt;
      const rotateY = ((centerX - x) / centerX) * maxTilt;

      el.style.transform = `
        perspective(${perspective}px)
        rotateX(${rotateX}deg)
        rotateY(${rotateY}deg)
        scale3d(${scale}, ${scale}, ${scale})
      `;
    };

    const handleMouseEnter = () => {
      el.style.transition = `transform ${speed}ms ease`;
    };

    const handleMouseLeave = () => {
      el.style.transform = 'perspective(1000px) rotateX(0) rotateY(0) scale3d(1, 1, 1)';
      el.style.transition = originalTransition;
    };

    // Add event listeners
    el.addEventListener('mousemove', handleMouseMove);
    el.addEventListener('mouseenter', handleMouseEnter);
    el.addEventListener('mouseleave', handleMouseLeave);

    // Store cleanup function as a dataset property for potential later cleanup
    (element as HTMLElement).dataset.tiltCleanup = 'true';
  });
};

/**
 * Check if the user prefers reduced motion
 */
export const prefersReducedMotion = (): boolean => {
  return window.matchMedia('(prefers-reduced-motion: reduce)').matches;
};

/**
 * Initialize performance-aware animations that adapt based on device capabilities
 * Handles edge cases: slow networks, low-performance devices, and FPS monitoring
 * @param callback - Function to call when animation should be adjusted based on performance
 */
export const initPerformanceAwareAnimations = (callback: (shouldReduce: boolean) => void): void => {
  // Check if the browser supports performance API
  if ('connection' in navigator) {
    const connection = (navigator as any).connection;
    if (connection) {
      // If user is on a slow connection, reduce animations
      if (connection.effectiveType === 'slow-2g' || connection.effectiveType === '2g') {
        callback(true);
        return;
      }
    }
  }

  // Check for low-performance devices based on hardware concurrency
  if (navigator.hardwareConcurrency && navigator.hardwareConcurrency <= 2) {
    // Likely a low-end device, reduce animations
    callback(true);
    return;
  }

  // If we have performance API, measure frame rate
  if ('performance' in window && 'now' in window.performance) {
    let frameCount = 0;
    let startTime = performance.now();

    const checkFrameRate = () => {
      frameCount++;
      const currentTime = performance.now();

      if (currentTime - startTime >= 1000) { // 1 second has passed
        const fps = frameCount;
        if (fps < 30) { // If FPS is below 30, reduce animations
          callback(true);
        }
        frameCount = 0;
        startTime = currentTime;
      }

      requestAnimationFrame(checkFrameRate);
    };

    requestAnimationFrame(checkFrameRate);
  }
};

/**
 * FPS Monitor class to track and respond to frame rate changes
 * Handles edge cases: low FPS detection and automatic animation reduction
 */
export class FPSMonitor {
  private frameCount: number = 0;
  private lastTime: number = performance.now();
  private fps: number = 60;
  private isLowPerformance: boolean = false;
  private threshold: number = 30; // FPS threshold
  private callback: ((fps: number, isLowPerformance: boolean) => void) | null = null;

  constructor(threshold: number = 30) {
    this.threshold = threshold;
    this.startMonitoring();
  }

  private startMonitoring() {
    const updateFPS = () => {
      this.frameCount++;
      const now = performance.now();
      const delta = now - this.lastTime;

      if (delta >= 1000) {
        this.fps = Math.round((this.frameCount * 1000) / delta);
        this.frameCount = 0;
        this.lastTime = now;

        // Check if performance is below threshold
        this.isLowPerformance = this.fps < this.threshold;

        // Call the callback if set
        if (this.callback) {
          this.callback(this.fps, this.isLowPerformance);
        }
      }

      requestAnimationFrame(updateFPS);
    };

    requestAnimationFrame(updateFPS);
  }

  public getFPS(): number {
    return this.fps;
  }

  public isPerformanceLow(): boolean {
    return this.isLowPerformance;
  }

  public setThreshold(threshold: number): void {
    this.threshold = threshold;
  }

  public setCallback(callback: (fps: number, isLowPerformance: boolean) => void): void {
    this.callback = callback;
  }
}

// Initialize FPS Monitor only in browser environment
let fpsMonitor: FPSMonitor | null = null;

if (typeof window !== 'undefined') {
  fpsMonitor = new FPSMonitor(30);
}

export { fpsMonitor };

/**
 * Initialize reduced animation mode for slower devices
 * @param shouldReduce - Whether to reduce animations
 */
export const initReducedAnimationMode = (shouldReduce: boolean): void => {
  if (shouldReduce) {
    document.body.classList.add('low-performance');
  } else {
    document.body.classList.remove('low-performance');
  }
};

/**
 * Cleanup function to remove tilt effect event listeners
 * @param selector - CSS selector for elements to cleanup
 */
export const cleanupTiltEffect = (selector: string): void => {
  const elements = document.querySelectorAll(selector);

  elements.forEach((element) => {
    if ((element as HTMLElement).dataset.tiltCleanup) {
      // Remove event listeners by recreating the element without listeners
      // This is a simple way to remove all event listeners
      const newElement = element.cloneNode(true) as HTMLElement;
      element.parentNode?.replaceChild(newElement, element);
    }
  });
};