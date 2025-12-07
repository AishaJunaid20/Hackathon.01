import React from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import styles from './about.module.css';

const SKILLS = [
  { name: "Frontend", icon: "🎨" },
  { name: "Backend", icon: "⚙️" },
  { name: "Python", icon: "🐍" },
  { name: "AI Agentic", icon: "🧠" },
  { name: "Graphic Designing", icon: "🖌️" },
  { name: "Video Editing", icon: "🎬" },
];

const PROJECTS = [
  {
    title: "AI Assistant",
    desc: "Smart conversational system with context-aware reasoning.",
    tags: ["AI", "Agentic"],
    link: "https://github.com/AishaJunaid20",
  },
  {
    title: "Creative Portfolio",
    desc: "Modern UI portfolio showcasing powerful React components.",
    tags: ["Frontend", "UI/UX"],
    link: "https://github.com/AishaJunaid20",
  },
  {
    title: "Python Automation",
    desc: "Automating workflows and data processes with Python.",
    tags: ["Python", "Automation"],
    link: "https://github.com/AishaJunaid20",
  },
];

function HeroSection() {
  return (
    <section className={styles.heroSection}>
      {/* Background Effects */}
      <div className={styles.auroraBg}>
        <div className={styles.aurora1}></div>
        <div className={styles.aurora2}></div>
        <div className={styles.aurora3}></div>
      </div>

      <div className={clsx("container", styles.heroContainer)}>
        {/* LEFT CONTENT */}
        <div className={styles.heroContent}>
          <div className={styles.introLabel}>
            <span className={styles.line}></span>
            <span>Meet The Developer</span>
          </div>

          <h1 className={styles.heroTitle}>
            Aisha Junaid<span className={styles.dot}>.</span>
          </h1>

          <p className={styles.heroSubtitle}>
            Crafting <strong>Intelligent Solutions</strong> &
            <strong> Creative Experiences</strong>.
            <br />
            Specializing in <span className={styles.highlight}>AI Agentic</span>
            & <span className={styles.highlight}>Creative Tech</span>.
          </p>

          <div className={styles.statRow}>
            <div className={styles.statItem}>
              <span className={styles.statNum}>10+</span>
              <span className={styles.statLabel}>Projects</span>
            </div>
            <div className={styles.statSeparator}></div>

            <div className={styles.statItem}>
              <span className={styles.statNum}>AI</span>
              <span className={styles.statLabel}>Specialist</span>
            </div>
            <div className={styles.statSeparator}></div>

            <div className={styles.statItem}>
              <span className={styles.statNum}>Creative</span>
              <span className={styles.statLabel}>Designer</span>
            </div>
          </div>

          <div className={styles.actionRow}>
            <a href="https://github.com/AishaJunaid20" className={styles.primaryBtn}>
              GitHub Profile
            </a>
            <a href="#expertise" className={styles.secondaryBtn}>
              View Expertise
            </a>
          </div>
        </div>

        {/* RIGHT SIDE IMAGE (Re-added as you wanted) */}
        <div className={styles.heroImageWrapper}>
          <div className={styles.imageGlassCard}>
            <img 
              src="https://thumbs.dreamstime.com/b/futuristic-high-tech-android-portrait-glowing-cybernetic-details-dark-background-368825004.jpg"
              alt="Aisha Junaid"
              className={styles.profileImage}
            />
            <div className={styles.glassReflection}></div>
          </div>

          {/* Decorative Elements */}
          <div className={styles.decoCircle}></div>
          <div className={styles.decoDots}>
            {[...Array(25)].map((_, i) => (
              <div key={i} className={styles.dotGridItem}></div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

function ExpertiseSection() {
  return (
    <section id="expertise" className={styles.expertiseSection}>
      <div className="container">
        <div className={styles.expertiseHeader}>
          <h2>The Toolkit</h2>
        </div>

        <div className={styles.skillsMarquee}>
          <div className={styles.marqueeTrack}>
            {[...SKILLS, ...SKILLS, ...SKILLS].map((skill, idx) => (
              <div key={idx} className={styles.skillBadge}>
                <span className={styles.skillIcon}>{skill.icon}</span>
                <span className={styles.skillName}>{skill.name}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

function WorkSection() {
  return (
    <section className={styles.workSection}>
      <div className="container">
        <div className={styles.workHeader}>
          <h2>Featured Creations</h2>
        </div>

        <div className={styles.projectList}>
          {PROJECTS.map((pr, i) => (
            <a key={i} href={pr.link} target="_blank" rel="noreferrer" className={styles.projectRow}>
              <div className={styles.projectInfo}>
                <h3 className={styles.projectTitle}>{pr.title}</h3>
                <p className={styles.projectDesc}>{pr.desc}</p>
              </div>

              <div className={styles.projectMeta}>
                <div className={styles.projectTags}>
                  {pr.tags.map((tag, t) => (
                    <span key={t} className={styles.tag}>{tag}</span>
                  ))}
                </div>
                <span className={styles.arrowIcon}>→</span>
              </div>
            </a>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function About() {
  return (
    <Layout
      title="Aisha Junaid | Creative Developer"
      description="Creative Portfolio of Aisha Junaid"
    >
      <main className={styles.mainWrapper}>
        <HeroSection />
        <ExpertiseSection />
        <WorkSection />
      </main>
    </Layout>
  );
}
