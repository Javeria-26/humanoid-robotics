import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import HomepageFeatures from '../components/HomepageFeatures';
import IndigoHeading from '../components/IndigoHeading';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      {/* Indigo Background with Moving Patterns */}
      <IndigoHeading title={siteConfig.title} subtitle={siteConfig.tagline} />

      {/* Content Layer with proper z-index */}
      <div className={styles.contentLayer}>
        <div className="container">
          <div className={styles.ctaSection}>
            {/* Prominent "Start Reading" primary button with neon accent */}
            <div className={styles.primaryButtonContainer}>
              <Link
                className="button button--primary button--lg"
                to="/docs">
                Start Reading
              </Link>
            </div>
            <div className={styles.secondaryActions}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/modules/ros2-nervous-system/">
                Start ROS 2 Educational Module - 15min ⏱️
              </Link>
              <Link
                className="button button--secondary button--lg"
                to="/docs/module-2/digital-twins-robotics">
                Start Digital Twin Module - 20min ⏱️
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
