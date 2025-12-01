import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

export default function Home() {
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    setIsVisible(true);
  }, []);

  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Build Autonomous Humanoid Robots from Scratch - A comprehensive textbook covering ROS 2, NVIDIA Isaac, Reinforcement Learning, and Vision-Language-Action systems">
      <main className={styles.main}>
        {/* Hero Section with Robotic Theme */}
        <section className={`${styles.hero} ${isVisible ? styles.fadeIn : ''}`}>
          <div className={styles.heroContainer}>
            <div className={styles.heroContent}>
              {/* Feature Pills */}
              <div className={styles.pillContainer}>
                <span className={styles.pill}>
                  <span className={styles.pillIcon}>🤖</span>
                  Open Source
                </span>
                <span className={styles.pill}>
                  <span className={styles.pillIcon}>🧠</span>
                  AI-Powered Learning
                </span>
                <span className={styles.pill}>
                  <span className={styles.pillIcon}>⚡</span>
                  Hands-On
                </span>
              </div>

              {/* Main Title */}
              <h1 className={styles.heroTitle}>
                <span className={styles.titleLine1}>Physical AI &</span>
                <span className={styles.titleLine2}>Humanoid Robotics</span>
              </h1>

              {/* Subtitle */}
              <p className={styles.heroSubtitle}>
                Build Autonomous Humanoid Robots from Scratch
              </p>

              {/* Description */}
              <p className={styles.heroDescription}>
                Master ROS 2, NVIDIA Isaac, Reinforcement Learning, and Vision-Language-Action systems
                through 11 comprehensive hands-on chapters. From simulation to real-world deployment.
              </p>

              {/* CTA Buttons */}
              <div className={styles.ctaButtons}>
                <Link
                  className={styles.primaryCta}
                  to="/docs">
                  Start Reading →
                </Link>
                <Link
                  className={styles.secondaryCta}
                  to="https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook"
                  target="_blank">
                  <span className={styles.githubIcon}>⭐</span>
                  Star on GitHub
                </Link>
              </div>

              {/* Tech Stack Pills */}
              <div className={styles.techStack}>
                <span className={styles.techPill}>ROS 2</span>
                <span className={styles.techPill}>Isaac Sim</span>
                <span className={styles.techPill}>PyTorch</span>
                <span className={styles.techPill}>GPT-4</span>
              </div>
            </div>

            {/* Robot Illustration */}
            <div className={styles.heroVisual}>
              <div className={styles.robotContainer}>
                <div className={styles.glowOrb}></div>
                <img
                  src="https://raw.githubusercontent.com/Tarikul-Islam-Anik/Animated-Fluent-Emojis/master/Emojis/People/Robot.png"
                  alt="Humanoid Robot"
                  className={styles.robotImage}
                />
                <div className={styles.floatingCircle} style={{top: '10%', left: '10%'}}></div>
                <div className={styles.floatingCircle} style={{top: '70%', right: '15%'}}></div>
                <div className={styles.floatingCircle} style={{bottom: '20%', left: '20%'}}></div>
              </div>
            </div>
          </div>

          {/* Animated Grid Background */}
          <div className={styles.gridBackground}></div>
        </section>

        {/* Features Section */}
        <section className={styles.features}>
          <div className={styles.container}>
            <h2 className={styles.sectionTitle}>
              <span className={styles.titleGradient}>What You'll Master</span>
            </h2>

            <div className={styles.featureGrid}>
              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>🔧</div>
                <h3>Robotics Framework</h3>
                <p>Master ROS 2, Nav2, and URDF. Build modular robot software with professional-grade tools.</p>
                <div className={styles.techBadges}>
                  <span>ROS 2 Humble</span>
                  <span>Navigation</span>
                  <span>URDF</span>
                </div>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>🎮</div>
                <h3>Simulation & Testing</h3>
                <p>Simulate robots in Gazebo and NVIDIA Isaac Sim with physics-accurate environments.</p>
                <div className={styles.techBadges}>
                  <span>Gazebo</span>
                  <span>Isaac Sim</span>
                  <span>Unity</span>
                </div>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>👁️</div>
                <h3>Perception & Vision</h3>
                <p>Implement VSLAM, object detection, and tracking with cutting-edge computer vision.</p>
                <div className={styles.techBadges}>
                  <span>VSLAM</span>
                  <span>YOLO</span>
                  <span>OpenCV</span>
                </div>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>🧠</div>
                <h3>AI & Deep Learning</h3>
                <p>Integrate GPT-4, Claude, and custom neural networks for intelligent robotics.</p>
                <div className={styles.techBadges}>
                  <span>LLMs</span>
                  <span>PyTorch</span>
                  <span>Voice AI</span>
                </div>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>🎯</div>
                <h3>Reinforcement Learning</h3>
                <p>Train locomotion and manipulation policies with PPO and advanced RL algorithms.</p>
                <div className={styles.techBadges}>
                  <span>PPO</span>
                  <span>Planning</span>
                  <span>Control</span>
                </div>
              </div>

              <div className={styles.featureCard}>
                <div className={styles.cardIcon}>⚙️</div>
                <h3>Hardware Deployment</h3>
                <p>Deploy on Jetson Orin with real sensors and actuators for production robots.</p>
                <div className={styles.techBadges}>
                  <span>Jetson Orin</span>
                  <span>RealSense</span>
                  <span>LiDAR</span>
                </div>
              </div>
            </div>
          </div>
        </section>

        {/* Interactive AI Features */}
        <section className={styles.aiSection}>
          <div className={styles.container}>
            <h2 className={styles.sectionTitle}>
              <span className={styles.titleGradient}>🤖 AI-Powered Learning Experience</span>
            </h2>

            <div className={styles.aiGrid}>
              <div className={styles.aiFeatureCard}>
                <div className={styles.aiIcon}>🧠</div>
                <h3>RAG Chatbot</h3>
                <p>Context-aware AI assistant with instant answers from the textbook</p>
                <ul className={styles.aiFeatureList}>
                  <li>Select text for instant explanations</li>
                  <li>Bullet-point responses in 1-2 seconds</li>
                  <li>Conversation memory</li>
                </ul>
              </div>

              <div className={styles.aiFeatureCard}>
                <div className={styles.aiIcon}>👤</div>
                <h3>Smart Personalization</h3>
                <p>Content adapts to your skill level automatically</p>
                <ul className={styles.aiFeatureList}>
                  <li>Beginner: Simplified explanations</li>
                  <li>Intermediate: Standard content</li>
                  <li>Advanced: Deep research insights</li>
                </ul>
              </div>

              <div className={styles.aiFeatureCard}>
                <div className={styles.aiIcon}>🌍</div>
                <h3>Fast Translation</h3>
                <p>Multilingual support for accessibility</p>
                <ul className={styles.aiFeatureList}>
                  <li>Translate to Urdu in 2-3 seconds</li>
                  <li>Technical terms preserved</li>
                  <li>RTL text support</li>
                </ul>
              </div>
            </div>
          </div>
        </section>

        {/* Curriculum Overview */}
        <section className={styles.curriculum}>
          <div className={styles.container}>
            <h2 className={styles.sectionTitle}>
              <span className={styles.titleGradient}>📚 11 Comprehensive Chapters</span>
            </h2>

            <div className={styles.curriculumGrid}>
              <div className={styles.curriculumLayer}>
                <div className={styles.layerHeader}>
                  <span className={styles.layerBadge}>Layer 1</span>
                  <h3>Foundation</h3>
                </div>
                <ul>
                  <li>Ch 1: Introduction to Physical AI</li>
                  <li>Ch 2: ROS 2 Fundamentals</li>
                  <li>Ch 3: ROS 2 Packages & URDF</li>
                  <li>Ch 4: Gazebo Simulation</li>
                </ul>
              </div>

              <div className={styles.curriculumLayer}>
                <div className={styles.layerHeader}>
                  <span className={styles.layerBadge}>Layer 2</span>
                  <h3>AI-Assisted Development</h3>
                </div>
                <ul>
                  <li>Ch 5: Unity Visualization & ROS 2</li>
                </ul>
              </div>

              <div className={styles.curriculumLayer}>
                <div className={styles.layerHeader}>
                  <span className={styles.layerBadge}>Layer 3</span>
                  <h3>Intelligence Design</h3>
                </div>
                <ul>
                  <li>Ch 6: NVIDIA Isaac & VSLAM</li>
                  <li>Ch 7: Path Planning & RL</li>
                </ul>
              </div>

              <div className={styles.curriculumLayer}>
                <div className={styles.layerHeader}>
                  <span className={styles.layerBadge}>Layer 4</span>
                  <h3>Spec-Driven Integration</h3>
                </div>
                <ul>
                  <li>Ch 8: Voice-Language-Action</li>
                  <li>Ch 9: Capstone Project</li>
                  <li>Ch 10: Hardware & Lab Setup</li>
                  <li>Ch 11: Safety & Best Practices</li>
                </ul>
              </div>
            </div>
          </div>
        </section>

        {/* Final CTA */}
        <section className={styles.finalCta}>
          <div className={styles.ctaCard}>
            <h2>Ready to Build the Future?</h2>
            <p>Start your journey into Physical AI and Humanoid Robotics today</p>
            <Link className={styles.ctaButtonLarge} to="/docs">
              Start Learning Now →
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
