import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

const modulesData = [
  {
    title: "Module-0: ROS2 Fundamentals",
    chapters: [
      { name: "Chapter 1: Introduction to ROS2", link: "/docs/ros2/chapter1" },
      { name: "Chapter 2: ROS2 Workspaces and Your First Package", link: "/docs/ros2/chapter2" },
      { name: "Chapter 3: Writing Your First ROS2 Nodes (Publishers & Subscribers)", link: "/docs/ros2/chapter3" },
      { name: "Chapter 4: Defining Custom ROS2 Message Types", link: "/docs/ros2/chapter4" },
    ],
  },
  {
    title: "Module-1: Robot Simulation",
    chapters: [
      { name: "Chapter 5: Introduction to Robot Simulation", link: "/docs/simulation/chapter5" },
      { name: "Chapter 6: Gazebo for Robot Simulation", link: "/docs/simulation/chapter6" },
      { name: "Chapter 7: Defining and Spawning Robots in Gazebo", link: "/docs/simulation/chapter7" },
      { name: "Chapter 8: Controlling Simulated Robots with ROS2(ros2_control)", link: "/docs/simulation/chapter8" },
    ],
  },
  {
    title: "Module-2: AI-Robot Brain (NVIDIA Isaac)",
    chapters: [
      { name: "Chapter 9: NVIDIA Isaac Platform Overview", link: "/docs/isaac/chapter9" },
      { name: "Chapter 10: Setting Up Isaac Sim and Omniverse", link: "/docs/isaac/chapter10" },
      { name: "Chapter 11: Working with USD and Robot Assets in Isaac Sim", link: "/docs/isaac/chapter11" },
      { name: "Chapter 12: Accelerated AI Perception with Isaac ROS", link: "/docs/isaac/chapter12" },
    ],
  },
  {
    title: "Module-3: Vision-Language-Action (VLA)",
    chapters: [
      { name: "Chapter 13: Introduction to Vision-Language-Action (VLA)", link: "/docs/vla/chapter13" },
      { name: "Chapter 14: Large Language Models for Robotics", link: "/docs/vla/chapter14" },
      { name: "Chapter 15: Visual Grounding and Embodied Perception", link: "/docs/vla/chapter15" },
      { name: "Chapter 16: Learning Action Policies from Vision and Language", link: "/docs/vla/chapter16" },
    ],
  },
  {
    title: "Module-4: Capstone Project",
    chapters: [
      { name: "Chapter 17: Capstone Project Introduction and Problem Statement", link: "/docs/capstone/chapter17" },
      { name: "Chapter 18: System Architecture and ROS2 Integration", link: "/docs/capstone/chapter18" },
      { name: "Chapter 19: Simulated Environment Setup and Humanoid Model", link: "/docs/capstone/chapter19" },
      { name: "Chapter 20: Testing, Evaluation, and Future Work", link: "/docs/capstone/chapter20" },
    ],
  },
];

const quizData = [
  { name: "Quiz for Module 0: ROS2 Fundamentals", link: "/docs/ros2/quiz_module0" },
  { name: "Quiz for Module 1: Robot Simulation", link: "/docs/simulation/quiz_module1" },
  { name: "Quiz for Module 2: AI-Robot Brain (NVIDIA Isaac)", link: "/docs/isaac/quiz_module2" },
  { name: "Quiz for Module 3: Vision-Language-Action (VLA)", link: "/docs/vla/quiz_module3" },
  { name: "Quiz for Module 4: Capstone Project", link: "/docs/capstone/quiz_module4" },
];

function Home() {
  return (
    <Layout
      title={`MyBook Index`}
      description="Index page for Physical AI & Humanoid Robotics book"
    >
      <header className='hero hero--primary'>
        <div className='container'style={{ textAlign: 'center' }}>
          <h1 className='hero__title'>Physical AI & Humanoid Robotics book</h1>
        </div>
      </header>
      <main>
        <div className='container padding-vert--xl'>
          <div className='row'>
            {modulesData.map((module, idx) => (
              <div key={idx} className='col col--6 margin-bottom--lg'>
                <div className='card'>
                  <div className='card__header'>
                    <h3>{module.title}</h3>
                  </div>
                  <div className='card__body'>
                    <ul>
                      {module.chapters.map((chapter, chapterIdx) => (
                        <li key={chapterIdx}>
                          <Link to={chapter.link}>{chapter.name}</Link>
                        </li>
                      ))}
                    </ul>
                  </div>
                </div>
              </div>
            ))}

            <div className='col col--6 margin-bottom--lg'>
              <div className='card'>
                <div className='card__header'>
                  <h3>Assessment & Quiz</h3>
                </div>
                <div className='card__body'>
                  <ul>
                    {quizData.map((quiz, idx) => (
                      <li key={idx}>
                        <Link to={quiz.link}>{quiz.name}</Link>
                      </li>
                    ))}
                  </ul>
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}

export default Home;
