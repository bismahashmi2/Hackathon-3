import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link'; 
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import ModuleTiers from '@site/src/components/ModuleTiers';
import styles from './index.module.css';

function BookHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBook)}>
      <div className={clsx('container', styles.heroContainer)}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <div className={styles.titleBadge}>
              <span className={styles.bookIcon}>📚</span>
              <span>Comprehensive Textbook</span>
            </div>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>
              {siteConfig.tagline}
            </p>

            <div className={styles.statsRow}>
              <div className={styles.stat}>
                <strong>14</strong>
                <span>Modules</span>
              </div>
              <div className={styles.stat}>
                <strong>170</strong>
                <span>Hours</span>
              </div>
              <div className={styles.stat}>
                <strong>42</strong>
                <span>Labs</span>
              </div>
            </div>
          </div>

          <div className={styles.heroButtons}>
            <Link
              className={clsx('button', 'button--primary', 'button--lg', styles.ctaButton)}
              to={`/docs/textbook/modules/introduction-physical-ai/01`}
              rel="noopener noreferrer">
              Start Learning →
            </Link>
            <Link
              className={clsx('button', 'button--outline', 'button--lg', styles.secondaryButton)}
              to="/docs/textbook/appendices/module-progression"
              target="_blank"
              rel="noopener noreferrer">
              View Curriculum
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function LearningPaths() {
  return (
    <section className={styles.learningPathsSection}>
      <div className="container">
        <div className={styles.pathsHeader}>
          <Heading as="h2" className={styles.pathsTitle}>
            Learning Paths
          </Heading>
          <p className={styles.pathsSubtitle}>
            Choose your focus area or follow the complete curriculum
          </p>
        </div>
        <div className={styles.pathsGrid}>
          <div className={clsx(styles.pathCard, styles.pathControl)}>
            <div className={styles.pathBadge}>Control Systems</div>
            <Heading as="h3" className={styles.pathTitle}>Path A</Heading>
            <p className={styles.pathDescription}>
              Focus on dynamics, control theory, and manipulation. Ideal for robotics engineers and control specialists.
            </p>
            <div className={styles.pathModules}>
              <span>01 → 02 → 03 → 05 → 07/08 → 11 → 14</span>
            </div>
          </div>
          <div className={clsx(styles.pathCard, styles.pathPerception)}>
            <div className={styles.pathBadge}>Perception</div>
            <Heading as="h3" className={styles.pathTitle}>Path B</Heading>
            <p className={styles.pathDescription}>
              Focus on sensors, vision, planning, and human-robot interaction. Ideal for computer vision researchers.
            </p>
            <div className={styles.pathModules}>
              <span>01 → 04 → 06 → 09 → 10 → 12 → 14</span>
            </div>
          </div>
          <div className={clsx(styles.pathCard, styles.pathFullStack)}>
            <div className={styles.pathBadge}>Complete</div>
            <Heading as="h3" className={styles.pathTitle}>Path C</Heading>
            <p className={styles.pathDescription}>
              Full-stack robotics covering all topics. Recommended for comprehensive understanding and capstone projects.
            </p>
            <div className={styles.pathModules}>
              <span>01 → 02 → 03 → 04 → 05 → 06 → 07 → 08 → 09 → 10 → 11 → 12 → 13 → 14</span>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  return (
    <Layout
      title="Physical AI & Humanoid Robotics Textbook"
      description="A comprehensive 14-module textbook covering Physical AI and Humanoid Robotics from fundamentals to advanced integration.">
      <BookHero />
      <main>
        <ModuleTiers />
        <LearningPaths />
      </main>
    </Layout>
  );
}