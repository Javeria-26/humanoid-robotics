import React from 'react';

const AIBrainIcon = ({ className = '', size = '24' }) => {
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      className={className}
      aria-label="AI Brain Icon"
    >
      {/* Main brain shape */}
      <path d="M12 2a10 10 0 0 1 8 4.5c0 3.5-2 5.5-2 5.5s-2 2-2 4.5a2 2 0 0 1-4 0c0-2.5-2-4.5-2-4.5s-2-2-2-5.5A10 10 0 0 1 12 2Z" />

      {/* Neural network connections */}
      <path d="M8 10s1 1 2 2c1.5-1.5 3-1 4 0s2.5 2 4-1" />
      <path d="M7 14s.5-1 2-2c2 1 4 1 6 0s1.5 1 2 2" />
      <path d="M9 18s.5-1 1-2c1 1 2 1 3 0s2-1 3 0" />

      {/* Central processing unit */}
      <circle cx="12" cy="12" r="1.5" fill="currentColor" />
    </svg>
  );
};

export default AIBrainIcon;