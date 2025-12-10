import React from 'react';
import './footer.css';

const Footer: React.FC = () => {
  return (
    <footer className="footer">
      <div className="footer-container">
        <div className="footer-column">
          <h3>Course Modules</h3>
          <ul>
            <li><a href="/tutorial/module-01">Module 1: ROS 2</a></li>
            <li><a href="/tutorial/module-02">Module 2: Digital Twin</a></li>
            <li><a href="/tutorial/module-03">Module 3: NVIDIA Isaac</a></li>
            <li><a href="/tutorial/module-04">Module 4: Vision-Language-Action</a></li>
          </ul>
        </div>

        <div className="footer-column">
          <h3>Resources</h3>
          <ul>
            <li><a href="https://github.com/muhammad-bilal-amir/physical-ai-textbook" target="_blank" rel="noopener noreferrer">GitHub Repository</a></li>
          </ul>
        </div>

        <div className="footer-column">
          <h3>About</h3>
          <ul className="footer-about-list">
            <li>Created by Muhammad Bilal Amir</li>
            <li>© 2025 Physical AI Textbook</li>
          </ul>
        </div>
      </div>
    </footer>
  );
};

export default Footer;