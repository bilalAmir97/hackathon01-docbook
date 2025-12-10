// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'ROS 2 Robotics Education',
  tagline: 'Practical mastery of ROS 2 middleware to control humanoid robots',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
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
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          ignorePatterns: ['/tags/**'],
        },
      } satisfies Preset.Options,
    ],
  ],

  // Performance optimizations
  stylesheets: [
    {
      href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;700&family=Poppins:wght@400;700&family=JetBrains+Mono:wght@400;700&display=swap',
      type: 'text/css',
      rel: 'stylesheet',
      crossorigin: 'anonymous',
    },
  ],

  themes: [
    // ... your other themes
  ],


  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    metadata: [
      { name: 'viewport', content: 'width=device-width, initial-scale=1.0' },
      { name: 'description', content: 'Master robots that bridge digital intelligence and physical reality. Learn ROS 2, simulation, and humanoid control to build the next generation of intelligent robots.' },
      { name: 'keywords', content: 'ROS 2, robotics, AI, humanoid robots, simulation, Gazebo, NVIDIA Isaac, machine learning' },
      { name: 'theme-color', content: '#00d9ff' },
      // Open Graph tags for social sharing
      { property: 'og:title', content: 'ROS 2 Robotics Education' },
      { property: 'og:description', content: 'Master robots that bridge digital intelligence and physical reality. Learn ROS 2, simulation, and humanoid control to build the next generation of intelligent robots.' },
      { property: 'og:type', content: 'website' },
      { property: 'og:url', content: 'https://your-docusaurus-site.example.com' },
      { property: 'og:image', content: 'img/docusaurus-social-card.jpg' },
      // Twitter card tags
      { name: 'twitter:card', content: 'summary_large_image' },
      { name: 'twitter:title', content: 'ROS 2 Robotics Education' },
      { name: 'twitter:description', content: 'Master robots that bridge digital intelligence and physical reality. Learn ROS 2, simulation, and humanoid control to build the next generation of intelligent robots.' },
      { name: 'twitter:image', content: 'img/docusaurus-social-card.jpg' },
    ],
    headTags: [
      {
        tagName: 'noscript',
        innerHTML: '<div class="noscript-message" style="position: fixed; top: 0; left: 0; width: 100%; background: #ff6b6b; color: white; text-align: center; padding: 10px; z-index: 9999; font-family: sans-serif;">This website requires JavaScript to function properly. Please enable JavaScript to experience the full interactive features.</div>',
      },
      // Preload critical fonts for performance
      {
        tagName: 'link',
        attributes: {
          rel: 'preload',
          href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;700&family=Poppins:wght@400;700&family=JetBrains+Mono:wght@400;700&display=swap',
          as: 'style',
          onload: "this.onload=null;this.rel='stylesheet'"
        },
      },
    ],
    // Custom fonts configuration
    fonts: {
      fontFace: [
        {
          family: 'Inter',
          urls: ['/fonts/Inter-Regular.woff2', '/fonts/Inter-Regular.woff'],
          style: 'normal',
          weight: '400',
        },
        {
          family: 'Inter',
          urls: ['/fonts/Inter-Bold.woff2', '/fonts/Inter-Bold.woff'],
          style: 'normal',
          weight: '700',
        },
        {
          family: 'Poppins',
          urls: ['/fonts/Poppins-Regular.woff2', '/fonts/Poppins-Regular.woff'],
          style: 'normal',
          weight: '400',
        },
        {
          family: 'Poppins',
          urls: ['/fonts/Poppins-Bold.woff2', '/fonts/Poppins-Bold.woff'],
          style: 'normal',
          weight: '700',
        },
        {
          family: 'JetBrains Mono',
          urls: ['/fonts/JetBrainsMono-Regular.woff2', '/fonts/JetBrainsMono-Regular.woff'],
          style: 'normal',
          weight: '400',
        },
      ],
    },
    navbar: {
      title: 'ROS 2 Robotics Education',
      logo: {
        alt: '',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Tutorial',
        },
        {
          href: 'https://github.com/facebook/docusaurus',
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
              label: 'Tutorial',
              to: '/docs/module-01-ros2/chapter-01/lesson-01-introduction-to-ros2-concepts',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            {
              label: 'Discord',
              href: 'https://discordapp.com/invite/docusaurus',
            },
            {
              label: 'Twitter',
              href: 'https://twitter.com/docusaurus',
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
      copyright: `Copyright © ${new Date().getFullYear()} ROS 2 Robotics Education. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;