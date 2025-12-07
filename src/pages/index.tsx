import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';
import ChatbotWidget from '../components/ChatbotWidget';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className="heroBanner">
      <div className="heroContainer container">
       

        {/* Updated Bigger Heading */}
        <h1 className="heroTitle">
          Elevating Intelligence:<br />
          <span className="heroTitleAccent">Where AI Controls the Physical World</span>
        </h1>

        {/* Updated Subtitle */}
        <p className="heroSubtitle">
          Step into the future of embodied intelligence. From robots to agents experience unified, adaptive, and powerful Physical AI.
        </p>

        {/* Button */}
        <div>
          <Link
            className="heroButton"
            to="/docs/intro">
            Connect Dapp
          </Link>
        </div>

        <ChatbotWidget />
      </div>
    </header>
  );
}

function CreativeFeaturesSection() {
  return (
    <section className="featuresSection">
      <div className="featuresGrid">

        {/* Chatbot Card */}
        <div className="creativeCard">
          <div className="cardContent">
            <h3 className="cardTitle">Interactive AI Tutor</h3>
            <p className="cardDesc">
              Get step-by-step guidance from an advanced integrated AI assistant.
            </p>
          </div>
          <div className="visualContainer">
            <div className="chatVisual">
              <div className="chatBubble bubbleLeft">How do I launch Gazebo?</div>
              <div className="chatBubble bubbleRight">Run `ros2 launch gazebo_ros...`</div>
            </div>
          </div>
        </div>

        {/* Translation Card */}
        <div className="creativeCard">
          <div className="cardContent">
            <h3 className="cardTitle">Learn in Urdu</h3>
            <p className="cardDesc">
              Break learning barriers with a complete Urdu-friendly robotics curriculum.
            </p>
          </div>
          <div className="visualContainer">
            <div className="translateVisual">
              <div className="langNode">EN</div>
              <div className="langArrow">⇄</div>
              <div className="langNode active">UR</div>
            </div>
          </div>
        </div>

      </div>
    </section>
  );
}

function TechStackSection() {
  return (
    <section className="techStackSection">
      <div className="container">
        <div className="sectionHeader">
          <h2 className="sectionTitle">The Intelligence Stack</h2>
          <p className="sectionSubtitle">Powered by industry-leading tools for robotics & embodied AI.</p>
        </div>
        <div className="techGrid">

          <div className="techCard">
            <div className="techIcon">⚛️</div>
            <h3 className="techName">ROS 2</h3>
            <p className="techDesc">Robotic Nervous System</p>
          </div>

          <div className="techCard">
            <div className="techIcon">🦾</div>
            <h3 className="techName">Isaac Sim</h3>
            <p className="techDesc">Photorealistic Physics</p>
          </div>

          <div className="techCard">
            <div className="techIcon">🧠</div>
            <h3 className="techName">PyTorch</h3>
            <p className="techDesc">Deep Learning Brain</p>
          </div>

          <div className="techCard">
            <div className="techIcon">⚡</div>
            <h3 className="techName">Jetson Orin</h3>
            <p className="techDesc">Edge Compute</p>
          </div>

        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course">
      <HomepageHeader />
      <main>
        <CreativeFeaturesSection />
        <TechStackSection />
      </main>
    </Layout>
  );
}