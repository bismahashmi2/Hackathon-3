import type {ReactNode} from 'react';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import clsx from 'clsx';
import styles from './styles.module.css';

// Mapping function to get the correct directory name for each module
function getModuleDirectoryName(moduleNumber: string, moduleTitle: string): string {
  // Define explicit mappings for modules where the pattern doesn't follow a simple rule
  const explicitMappings: Record<string, string> = {
    '01': '01-introduction-physical-ai',
    '04': '04-sensors-perception', // "Sensors and Perception" -> removes "and"
    '05': '05-dynamics-control', // "Dynamics and Control" -> removes "and"
    '10': '10-simulation-to-real', // "Simulation to Real" -> keeps "to"
  };

  if (explicitMappings[moduleNumber]) {
    return explicitMappings[moduleNumber];
  }

  // For other modules, use a standard kebab-case conversion
  const kebabCaseTitle = moduleTitle
    .toLowerCase()
    .replace(/[^\w\s-]/g, '') // Remove special characters
    .replace(/\s+/g, '-') // Replace spaces with hyphens
    .replace(/--+/g, '-') // Replace multiple hyphens with single hyphen
    .trim();

  return `${moduleNumber.padStart(2, '0')}-${kebabCaseTitle}`;
}

// Function to generate the correct doc ID format
function getModuleDocId(moduleNumber: string, moduleTitle: string): string {
  // Define explicit mappings for doc IDs based on the available IDs from Docusaurus
  const explicitDocMappings: Record<string, string> = {
    '01': 'introduction-physical-ai',
    '02': 'rigid-body-dynamics',
    '03': 'kinematics-fundamentals',
    '04': 'sensors-perception',
    '05': 'dynamics-control',
    '06': 'motion-planning',
    '07': 'manipulation',
    '08': 'locomotion',
    '09': 'ros2-integration',
    '10': 'simulation-to-real',
    '11': 'learning-based-control',
    '12': 'human-robot-interaction',
    '13': 'full-body-autonomy',
    '14': 'capstone-integration',
  };

  let modulePath: string;
  if (explicitDocMappings[moduleNumber]) {
    modulePath = explicitDocMappings[moduleNumber];
  } else {
    // Fallback to converting the title to kebab case
    modulePath = moduleTitle
      .toLowerCase()
      .replace(/[^\w\s-]/g, '') // Remove special characters
      .replace(/\s+/g, '-') // Replace spaces with hyphens
      .replace(/--+/g, '-') // Replace multiple hyphens with single hyphen
      .trim();
  }

  return `${modulePath}/${moduleNumber}`;
}

interface Module {
  number: string;
  title: string;
  description: string;
  hours: number;
  labs: number;
}

interface Tier {
  name: string;
  weeks: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  modules: Module[];
}

const tierData: Tier[] = [
  {
    name: 'Beginner',
    weeks: 'Weeks 1-4',
    difficulty: 'beginner',
    modules: [
      {
        number: '01',
        title: 'Introduction to Physical AI',
        description: 'Physical AI definition, embodiment, history, humanoids',
        hours: 10,
        labs: 4
      },
      {
        number: '02',
        title: 'Rigid Body Dynamics',
        description: 'Newton-Euler, forces, torques, inertia, momentum',
        hours: 12,
        labs: 5
      },
      {
        number: '03',
        title: 'Kinematics Fundamentals',
        description: 'Forward/inverse kinematics, DH parameters, workspaces',
        hours: 12,
        labs: 5
      },
      {
        number: '04',
        title: 'Sensors and Perception',
        description: 'IMU, cameras, LIDAR, sensor fusion, state estimation',
        hours: 12,
        labs: 6
      }
    ]
  },
  {
    name: 'Intermediate',
    weeks: 'Weeks 5-10',
    difficulty: 'intermediate',
    modules: [
      {
        number: '05',
        title: 'Dynamics and Control',
        description: 'PID, computed torque, impedance control, stability',
        hours: 14,
        labs: 7
      },
      {
        number: '06',
        title: 'Motion Planning',
        description: 'RRT, A*, trajectory optimization, collision avoidance',
        hours: 14,
        labs: 7
      },
      {
        number: '07',
        title: 'Manipulation',
        description: 'Grasping, force control, dexterous manipulation',
        hours: 14,
        labs: 7
      },
      {
        number: '08',
        title: 'Locomotion',
        description: 'ZMP, whole-body control, bipedal walking, balance',
        hours: 14,
        labs: 7
      },
      {
        number: '09',
        title: 'ROS2 Integration',
        description: 'Nodes, topics, services, robot state, transforms',
        hours: 12,
        labs: 6
      },
      {
        number: '10',
        title: 'Simulation to Real',
        description: 'Domain randomization, reality gap, deployment',
        hours: 12,
        labs: 6
      }
    ]
  },
  {
    name: 'Advanced',
    weeks: 'Weeks 11-14',
    difficulty: 'advanced',
    modules: [
      {
        number: '11',
        title: 'Learning-Based Control',
        description: 'RL, imitation learning, policy optimization',
        hours: 14,
        labs: 7
      },
      {
        number: '12',
        title: 'Human-Robot Interaction',
        description: 'Safety, collaboration, natural interfaces',
        hours: 12,
        labs: 6
      },
      {
        number: '13',
        title: 'Full-Body Autonomy',
        description: 'Integrated systems, decision making, autonomy',
        hours: 14,
        labs: 7
      },
      {
        number: '14',
        title: 'Capstone Integration',
        description: 'System design, deployment, future directions',
        hours: 14,
        labs: 8
      }
    ]
  }
];

function ModuleCard({ module }: { module: Module }) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleHeader}>
        <span className={styles.moduleNumber}>Module {module.number}</span>
        <span className={styles.moduleHours}>{module.hours}h</span>
      </div>
      <Heading as="h4" className={styles.moduleTitle}>
        {module.title}
      </Heading>
      <p className={styles.moduleDescription}>{module.description}</p>
      <div className={styles.moduleFooter}>
        <span className={styles.labBadge}>{module.labs} labs</span>
        <Link
          className={styles.moduleLink}
          to={`doc:${module.number}`}
          target="_blank"
          rel="noopener noreferrer">
          View →
        </Link>
      </div>
    </div>
  );
}

function TierCard({ tier }: { tier: Tier }) {
  const badgeColor = {
    beginner: styles.beginner,
    intermediate: styles.intermediate,
    advanced: styles.advanced
  }[tier.difficulty];

  return (
    <div className={clsx(styles.tierCard, styles[tier.difficulty])}>
      <div className={clsx(styles.tierHeader, badgeColor)}>
        <div>
          <span className={styles.tierName}>{tier.name}</span>
          <span className={styles.tierWeeks}>{tier.weeks}</span>
        </div>
        <span className={styles.tierCount}>{tier.modules.length} modules</span>
      </div>
      <div className={styles.modulesGrid}>
        {tier.modules.map((module) => (
          <ModuleCard key={module.number} module={module} />
        ))}
      </div>
    </div>
  );
}

export default function ModuleTiers(): ReactNode {
  const totalHours = 170;
  const totalLabs = 42;
  const totalModules = 14;

  return (
    <section className={styles.tiersSection}>
      <div className="container">
        <div className={styles.overviewHeader}>
          <div>
            <Heading as="h2" className={styles.sectionTitle}>
              Textbook Curriculum
            </Heading>
            <p className={styles.sectionSubtitle}>
              A comprehensive 14-module journey through Physical AI and Humanoid Robotics
            </p>
          </div>
          <div className={styles.statsCard}>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>{totalModules}</span>
              <span className={styles.statLabel}>Modules</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>{totalHours}</span>
              <span className={styles.statLabel}>Total Hours</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>{totalLabs}</span>
              <span className={styles.statLabel}>Hands-on Labs</span>
            </div>
          </div>
        </div>

        <div className={styles.tiersGrid}>
          {tierData.map((tier) => (
            <TierCard key={tier.name} tier={tier} />
          ))}
        </div>

        <div className={styles.ctaSection}>
          <div className={styles.ctaContent}>
            <Heading as="h3" className={styles.ctaTitle}>
              Ready to Start Learning?
            </Heading>
            <p className={styles.ctaDescription}>
              Begin with Module 01 or jump to your preferred learning path.
            </p>
            <div className={styles.ctaButtons}>
              <Link
                className={clsx('button', 'button--primary', 'button--lg', styles.primaryButton)}
                to="doc:01"
                target="_blank"
                rel="noopener noreferrer">
                Start with Module 01
              </Link>
              <Link
                className={clsx('button', 'button--outline', 'button--lg', styles.secondaryButton)}
                to="/docs/textbook/appendices/module-progression"
                target="_blank"
                rel="noopener noreferrer">
                View Learning Paths
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}