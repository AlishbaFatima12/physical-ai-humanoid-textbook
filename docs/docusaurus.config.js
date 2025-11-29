const {themes} = require('prism-react-renderer');

module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital AI to Embodied Intelligence - Build Autonomous Robots from Scratch',
  url: 'https://alishbafatima12.github.io',
  baseUrl: '/physical-ai-humanoid-textbook/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  // GitHub Pages deployment config
  organizationName: 'AlishbaFatima12',
  projectName: 'physical-ai-humanoid-textbook',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
          editUrl: 'https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook/edit/main/docs/',
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: false,
        pages: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
    // Announcement bar
    announcementBar: {
      id: 'course_launch',
      content: '🎉 <strong>New Course!</strong> Build autonomous humanoid robots with ROS 2, AI, and RL. Start with <a href="/">Chapter 1</a>!',
      backgroundColor: '#4f46e5',
      textColor: '#ffffff',
      isCloseable: true,
    },

    // Navbar
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      hideOnScroll: false,
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: '📚 Chapters',
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    // Footer
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {
              label: 'Course Overview',
              to: '/',
            },
            {
              label: 'Chapter 1: Introduction',
              to: '/chapter-1-introduction-physical-ai',
            },
            {
              label: 'Capstone Project',
              to: '/chapter-9-capstone',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook/discussions',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/your-server',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook',
            },
            {
              label: 'Report Issues',
              href: 'https://github.com/AlishbaFatima12/physical-ai-humanoid-textbook/issues',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },

    // Syntax highlighting
    prism: {
      theme: themes.github,
      darkTheme: themes.dracula,
      additionalLanguages: ['bash', 'python', 'cpp', 'yaml', 'json', 'diff'],
    },

    // Color mode
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    // Table of contents
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 4,
    },
  },
};
