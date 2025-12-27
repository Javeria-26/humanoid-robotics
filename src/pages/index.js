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
          {/* Centered AI brain graphic */}
          <div className={styles.aiBrainGraphic}>
            <div className={styles.brainContainer}>
              <div className={styles.neuralNetwork}>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
                <div className={styles.neuron}></div>
              </div>
              <div className={styles.brainCore}>
                <div className={styles.coreGlow}></div>
              </div>
            </div>
          </div>

          {/* Main title with enhanced glowing text effect */}
          <h1 className={styles.heroTitle}>
            AI-Powered Knowledge Assistant
          </h1>

          {/* Subtitle explaining RAG-based AI system */}
          <p className={styles.heroSubtitle}>
            Transform your documents into intelligent, searchable knowledge bases with our advanced RAG (Retrieval-Augmented Generation) system powered by AI.
          </p>

          {/* Enhanced CTA buttons with better visual hierarchy */}
          <div className={styles.ctaSection}>
            <Link
              className="button button--primary button--lg"
              to="/docs">
              Explore Documentation
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="/test-chatbot">
              Interact with AI
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

function HomepageFeatures() {
  const features = [
    {
      title: 'Intelligent Document Processing',
      description: 'Upload and process complex documents with AI-powered extraction and understanding.',
      icon: '🧠',
    },
    {
      title: 'RAG-Powered Search',
      description: 'Retrieve relevant information using advanced vector search and semantic understanding.',
      icon: '🔍',
    },
    {
      title: 'Natural Conversations',
      description: 'Engage in human-like conversations with your document collection through our chat interface.',
      icon: '💬',
    },
    {
      title: 'Real-time Analysis',
      description: 'Get instant answers and insights from your knowledge base with low-latency responses.',
      icon: '⚡',
    },
    {
      title: 'Secure & Private',
      description: 'Your data stays secure with on-premise processing and end-to-end encryption.',
      icon: '🔒',
    },
    {
      title: 'Scalable Architecture',
      description: 'Built with modern technologies to handle large document collections efficiently.',
      icon: '📈',
    },
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
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
