import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './HeroBackground.module.css';

const HeroBackground = () => {
  const [isLoaded, setIsLoaded] = useState(false);

  useEffect(() => {
    // Set loaded state after a brief delay to allow for initial render
    const timer = setTimeout(() => {
      setIsLoaded(true);
    }, 100);

    return () => clearTimeout(timer);
  }, []);

  return (
    <div className={clsx(styles.backgroundLayer, {
      [styles.backgroundLoaded]: isLoaded
    })}>
      {/* Gradient mesh for 3D depth effect */}
      <div className={styles.gradientMesh}></div>
      {/* Particle layer for subtle animation */}
      <div className={styles.particleLayer}></div>
      {/* Additional visual elements for depth */}
      <div className={styles.depthLayer}></div>
      {/* Geometric pattern layer for enhanced depth */}
      <div className={styles.geometricLayer}></div>

      {/* Loading state overlay */}
      {!isLoaded && (
        <div className={styles.loadingOverlay}>
          <div className={styles.loadingSpinner}></div>
        </div>
      )}
    </div>
  );
};

export default HeroBackground;