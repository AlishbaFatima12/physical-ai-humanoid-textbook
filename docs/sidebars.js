module.exports = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Course Overview',
    },
    {
      type: 'category',
      label: '🔷 Layer 1: Foundation',
      collapsed: false,
      items: [
        'chapter-1-introduction-physical-ai',
        'chapter-2-ros2-fundamentals',
        'chapter-3-ros2-packages-urdf',
      ],
    },
    {
      type: 'category',
      label: '🤖 Layer 2: AI-Assisted',
      collapsed: false,
      items: [
        'chapter-4-gazebo-simulation',
        'chapter-5-unity-visualization',
      ],
    },
    {
      type: 'category',
      label: '🧠 Layer 3: Intelligence Design',
      collapsed: false,
      items: [
        'chapter-6-isaac-perception',
        'chapter-7-path-planning-rl',
      ],
    },
    {
      type: 'category',
      label: '🚀 Layer 4: Spec-Driven',
      collapsed: false,
      items: [
        'chapter-8-vla-humanoid',
        'chapter-9-capstone',
      ],
    },
    {
      type: 'category',
      label: '⚙️ Practice',
      collapsed: false,
      items: [
        'chapter-10-hardware-lab-setup',
        'chapter-11-safety-best-practices',
      ],
    },
  ],
};
