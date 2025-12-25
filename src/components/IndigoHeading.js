import React from 'react';
import clsx from 'clsx';
import styles from './IndigoHeading.module.css';

const IndigoHeading = ({ title, subtitle }) => {
  return (
    <div className={styles.indigoHeadingContainer}>
      <div className={styles.indigoBackground}>
        <div className={styles.movingPattern}></div>
        <div className={styles.movingPattern2}></div>
        <div className={styles.depthPattern}></div>
        <div className={styles.contentOverlay}>
          <h1 className={styles.headingText}>{title}</h1>
          <p className={styles.subheadingText}>{subtitle}</p>
        </div>
      </div>
    </div>
  );
};

export default IndigoHeading;