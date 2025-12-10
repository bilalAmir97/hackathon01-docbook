import React from 'react';
import HeroSection from './HeroSection';
import ModuleCards from './ModuleCards';
import '../../styles/homepage.css';

const Homepage: React.FC = () => {
  return (
    <div className="homepage-wrapper">
      <HeroSection />
      <ModuleCards />
    </div>
  );
};

export default Homepage;