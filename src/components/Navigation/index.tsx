import React, { useState, useEffect } from 'react';
import './navigation.css';

const Navigation: React.FC = () => {
  const [scrolled, setScrolled] = useState(false);

  useEffect(() => {
    const handleScroll = () => {
      if (window.scrollY > 10) {
        setScrolled(true);
      } else {
        setScrolled(false);
      }
    };

    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  return (
    <nav className={`navbar ${scrolled ? 'scrolled' : ''}`}>
      <div className="navbar-container">
        <div className="navbar-logo">
          <span>Physical AI Textbook</span>
        </div>
        <div className="navbar-links">
          <a href="/tutorial">Tutorial</a>
          <a href="https://github.com/muhammad-bilal-amir/physical-ai-textbook" target="_blank" rel="noopener noreferrer">GitHub</a>
        </div>
        <div className="navbar-search">
          <span>🔍</span>
        </div>
      </div>
    </nav>
  );
};

export default Navigation;