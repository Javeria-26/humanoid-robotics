// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to type-check this file
// and help you improve code quality.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI-Robot Brain Research Module',
  tagline: 'Advanced perception, training, and navigation using NVIDIA Isaac',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'your-project', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Optional: disable the blog plugin
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'ROS 2 Educational Modules',
        logo: {
          alt: 'ROS 2 Educational Modules',
          src: 'img/favicon.ico', // Using favicon as logo
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Module 1',
          },
          {
            type: 'docSidebar',
            sidebarId: 'module2Sidebar',
            position: 'left',
            label: 'Module 2',
          },
          {
            type: 'docSidebar',
            sidebarId: 'aiRobotBrainSidebar',
            position: 'left',
            label: 'Module 3',
          },
          {
            type: 'docSidebar',
            sidebarId: 'vlaModuleSidebar',
            position: 'left',
            label: 'Module 4',
          },
          {
            href: 'https://github.com/your-username/your-repo',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Module 1: ROS 2 Nervous System',
                to: '/docs/modules/ros2-nervous-system/index',
              },
              {
                label: 'Module 2: Digital Twin Simulation',
                to: '/docs/module-2/index',
              },
              {
                label: 'Module 3: AI-Robot Brain',
                to: '/docs/ai-robot-brain/chapter-1-overview',
              },
              {
                label: 'Module 4: VLA Research',
                to: '/docs/module-4-vla/overview',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'NVIDIA Developer Forums',
                href: 'https://forums.developer.nvidia.com/',
              },
              {
                label: 'ROS Discourse',
                href: 'https://discourse.ros.org/',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/facebook/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Educational Modules. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'json'],
      },
    }),
};

module.exports = config;