import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      {/* Content Layer with AI-themed design */}
      <div className={styles.contentLayer}>
        <div className="container">
          {/* Centered laptop/screen graphic displaying flowing code */}
          <div className={styles.laptopGraphic}>
            <div className={styles.screen}>
              <div className={styles.codeLines}>
                <div className={styles.codeLine}>function AI() {"{"}</div>
                <div className={styles.codeLine}>&nbsp;&nbsp;const development = "spec-driven";</div>
                <div className={styles.codeLine}>&nbsp;&nbsp;const native = true;</div>
                <div className={styles.codeLine}>&nbsp;&nbsp;return {"{"} development, native {"}"};</div>
                <div className={styles.codeLine}>{"}"}</div>
              </div>
            </div>
          </div>

          {/* Main title with glowing text effect */}
          <h1 className={styles.heroTitle}>
            AI Native Software Development
          </h1>

          {/* Subtitle explaining AI-driven, spec-driven development */}
          <p className={styles.heroSubtitle}>
            Experience the future of software development with AI-powered tools and specification-driven methodologies that transform how we build, test, and deploy applications.
          </p>

          {/* CTA buttons: "Start Reading", "Open Chatbot", "View Architecture" */}
          <div className={styles.ctaSection}>
            <Link
              className="button button--primary button--lg"
              to="/docs">
              Start Reading
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/test-chatbot">
              Open Chatbot
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/docs/modules/ros2-nervous-system/">
              View Architecture
            </Link>
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
      title={`AI Native Software Development`}
      description="AI-powered software development with specification-driven methodologies">
      <HomepageHeader />
    </Layout>
  );
}
