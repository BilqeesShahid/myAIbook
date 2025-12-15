import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline: 'Your Guide to Advanced Robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://my-ai-book-ten.vercel.app', // your Vercel URL
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/', // root path for Vercel
  


  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'BilqeesShahid', // Usually your GitHub org/user name.
  projectName: 'myAIbook', // Usually your repo name.

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
    localeConfigs: {
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/BilqeesShahid/myAIbook/tree/main/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/BilqeesShahid/myAIbook/tree/main/',
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  // Make environment variables available in client-side code
  themes: [],
  plugins: [
    async function myPlugin() {
      return {
        name: 'inject-environment-variables',
        configureWebpack() {
          return {
            resolve: {
              fallback: {
                process: require.resolve('process/browser'),
              },
            },
            plugins: [
              new (require('webpack')).DefinePlugin({
                'process.env.DOCUSAURUS_SUPABASE_URL': JSON.stringify(process.env.DOCUSAURUS_SUPABASE_URL),
                'process.env.DOCUSAURUS_SUPABASE_ANON_KEY': JSON.stringify(process.env.DOCUSAURUS_SUPABASE_ANON_KEY),
              }),
            ],
          };
        },
      };
    },
  ],

  plugins: [
    // Custom plugin to inject environment variables
    async function envPlugin(context, options) {
      return {
        name: 'env-plugin',
        configureWebpack(config, isServer, utils) {
          return {
            resolve: {
              fallback: {
                process: require.resolve('process/browser'),
              },
            },
            plugins: [
              new (require('webpack').DefinePlugin)({
                'process.env.DOCUSAURUS_SUPABASE_URL': JSON.stringify(process.env.DOCUSAURUS_SUPABASE_URL),
                'process.env.DOCUSAURUS_SUPABASE_ANON_KEY': JSON.stringify(process.env.DOCUSAURUS_SUPABASE_ANON_KEY),
              }),
            ],
          };
        },
      };
    },
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'My AI Book',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'myBookSidebar',
          position: 'left',
          label: 'Learn',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {to: '/signup', label: 'Sign Up', position: 'right', id: 'signup-link'},
        {to: '/signin', label: 'Sign In', position: 'right', id: 'signin-link'},
        {
          href: 'https://github.com/BilqeesShahid/myAIbook',
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
              label: 'MyBook',
              to: '/docs/intro',
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
              label: 'X',
              href: 'https://x.com/docusaurus',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/BilqeesShahid/myAIbook',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book, Inc. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;