import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";
import remarkMath from "remark-math";
import rehypeKatex from "rehype-katex";

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "Physical AI & Humanoid Robotics",
  tagline: "A Comprehensive 14-Module Textbook",
  favicon: "img/favicon.ico",

  stylesheets: [
    {
      href: "https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css",
      type: "text/css",
      integrity:
        "sha384-n8MVd4RsNIU0k7E2iK5nU5L0P0B7xvFeoDyAJ6Dig0KkPxDYGZJ5y+2QF0ZK7Sk",
      crossorigin: "anonymous",
    },
    {
      // Add Academy accent for book mode
      href: "https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.0.0/css/all.min.css",
      type: "text/css",
    },
  ],

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://bismahashmi2.github.io",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/Hackathon-3",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "bismahashmi2", // Usually your GitHub org/user name.
  projectName: "Hackathon-3", // Usually your repo name.

  onBrokenLinks: "warn",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
          // Edit URL for textbook content - points to your repository
          editUrl: "https://github.com/bismahashmi2/Hackathon-3/tree/main/docs",
          routeBasePath: "docs",
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },

        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Textbook social card
    image: "img/textbook-logo.png",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    announcementBar: {
      id: "textbook_release",
      content:
        "📚 This is a comprehensive 14-module textbook. Begin with <strong>Module 01 - Introduction to Physical AI</strong>",
      backgroundColor: "#2563eb",
      textColor: "#fff",
      isCloseable: true,
    },
    navbar: {
      title: "Physical AI & Humanoid Robotics",
      logo: {
        alt: "Textbook Logo",
        src: "img/textbooklogo.png",
      },
      items: [
        {
          to: "/docs/textbook/appendices/module-progression",
          label: "Textbook",
          position: "left",
          activeBaseRegex: "^/docs(/textbook)?(/|$)",
        },
        {
          type: "doc",
          docId: "textbook/modules/introduction-physical-ai/01",
          label: "Start Learning",
          position: "left",
          className: "navbar-button-primary",
        },
        {
          type: "search",
          position: "right",
        },
      ],
    },
    docs: {
      sidebar: {
        hideable: false,
        autoCollapseCategories: false,
      },
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Textbook",
          items: [
            {
              label: "Module Progression",
              to: "/docs/textbook/appendices/module-progression",
            },
            {
              label: "Learning Paths",
              to: "/docs/textbook/appendices/module-progression#learning-paths",
            },
            {
              label: "Case Studies",
              to: "/docs/textbook/case-studies",
            },
          ],
        },
        {
          title: "Modules",
          items: [
            {
              label: "Beginner (1-4)",
              to: "/docs/textbook/modules/introduction-physical-ai/01",
            },
            {
              label: "Intermediate (5-10)",
              to: "/docs/textbook/modules/dynamics-control/05",
            },
            {
              label: "Advanced (11-14)",
              to: "/docs/textbook/modules/learning-based-control/11",
            },
          ],
        },
        {
          title: "Resources",
          items: [
            {
              label: "Code Examples",
              to: "/docs/textbook/assets/code-examples/01/",
            },
            {
              label: "Diagrams",
              to: "/docs/textbook/assets/diagrams/",
            },
            {
              label: "Robot Models",
              to: "/docs/textbook/assets/robot-models/mujoco/",
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 3,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
