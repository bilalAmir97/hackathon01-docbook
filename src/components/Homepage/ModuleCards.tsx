import React, { useEffect } from 'react';
import { initStaggeredAnimations, initTiltEffect, prefersReducedMotion, initPerformanceAwareAnimations, initReducedAnimationMode } from '../../utils/animation-helpers';
import './modulecards.css';

// Define module data type
interface ModuleData {
  id: number;
  title: string;
  tagline: string;
  techIcons: string[];
}

const ModuleCards: React.FC = () => {
  // Module data
  const modules: ModuleData[] = [
    {
      id: 1,
      title: "The Robotic Nervous System",
      tagline: "ROS 2, Python, and Docker",
      techIcons: ["ROS 2", "Python", "Docker"]
    },
    {
      id: 2,
      title: "The Digital Twin",
      tagline: "Gazebo, Unity, and C#",
      techIcons: ["Gazebo", "Unity", "C#"]
    },
    {
      id: 3,
      title: "The AI-Robot Brain",
      tagline: "NVIDIA Isaac, CUDA, and AI",
      techIcons: ["NVIDIA", "Isaac", "CUDA"]
    },
    {
      id: 4,
      title: "Vision-Language-Action",
      tagline: "OpenAI, Whisper, and LLMs",
      techIcons: ["OpenAI", "Whisper", "LLM"]
    }
  ];

  useEffect(() => {
    // Initialize staggered animations for the module cards
    initStaggeredAnimations('.module-grid .module-card', 'animate-in', 100);

    // Initialize 3D tilt effect for module cards (skip on mobile and if reduced motion is preferred)
    if (window.innerWidth > 768 && !prefersReducedMotion()) {
      initTiltEffect('.module-card', { maxTilt: 5, perspective: 1000, scale: 1.02, speed: 300 });
    }

    // Initialize performance-aware animations
    initPerformanceAwareAnimations((shouldReduce) => {
      initReducedAnimationMode(shouldReduce);
    });

    // Handle prefers-reduced-motion changes
    const mediaQuery = window.matchMedia('(prefers-reduced-motion: reduce)');
    const handleReducedMotionChange = (e: MediaQueryListEvent) => {
      if (e.matches) {
        initReducedAnimationMode(true);
      } else {
        initReducedAnimationMode(false);
      }
    };

    mediaQuery.addEventListener('change', handleReducedMotionChange);

    return () => {
      mediaQuery.removeEventListener('change', handleReducedMotionChange);
    };
  }, []);

  // Handle keyboard events for module cards
  const handleCardKeyDown = (e: React.KeyboardEvent<HTMLDivElement>, moduleId: number) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      // Navigate to the specific module
      window.location.href = `/tutorial/module-0${moduleId}`;
    }
  };

  return (
    <section className="module-cards-section" aria-labelledby="module-cards-title">
      <h2 id="module-cards-title" className="sr-only">Course Modules</h2>

      {/* Module Preview Strip */}
      <div className="module-preview-strip" role="region" aria-label="Module overview">
        {modules.map((module) => (
          <div key={module.id} className="preview-item" tabIndex={0}>
            <span className="preview-number" aria-label={`Module ${module.id}`}>0{module.id}</span>
            <span className="preview-title">{module.title}</span>
          </div>
        ))}
      </div>

      {/* Module Cards Grid */}
      <div className="module-grid" role="list" aria-label="Learning modules">
        {modules.map((module, index) => (
          <div
            key={module.id}
            className="module-card focusable"
            data-index={index}
            role="listitem"
            tabIndex={0}
            aria-label={`${module.title}. ${module.tagline}. Technologies: ${module.techIcons.join(', ')}`}
            onKeyDown={(e) => handleCardKeyDown(e, module.id)}
            onClick={() => {
              // Navigate to the appropriate module documentation based on module ID
              let modulePath = '';
              switch(module.id) {
                case 1:
                  modulePath = '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts';
                  break;
                case 2:
                  modulePath = '/docs/module-02-digital-twin/setup/index';
                  break;
                case 3:
                  modulePath = '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts'; // Will be updated when module is created
                  break;
                case 4:
                  modulePath = '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts'; // Will be updated when module is created
                  break;
                default:
                  modulePath = '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts';
              }
              window.location.href = modulePath;
            }}
          >
            <div className="module-number-badge" aria-label={`Module ${module.id}`}>
              0{module.id}
            </div>
            <h3 className="module-title" tabIndex={0}>{module.title}</h3>
            <p className="module-tagline" tabIndex={0}>{module.tagline}</p>
            <div className="tech-stack-icons" role="list" aria-label="Technology stack">
              {module.techIcons.map((tech, techIndex) => (
                <span
                  key={techIndex}
                  className="tech-icon"
                  role="listitem"
                  tabIndex={0}
                  aria-label={tech}
                >
                  {tech}
                </span>
              ))}
            </div>
          </div>
        ))}
      </div>
    </section>
  );
};

export default ModuleCards;