// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// https://astro.build/config
export default defineConfig({
  site: 'https://biotrondesignteam.github.io',
  base: '/Exo-Software/',
  integrations: [
    starlight({
      customCss: [
        './src/styles/custom.css',
      ],
      expressiveCode: {
        themes: ['github-light-high-contrast', 'tokyo-night'],
        removeUnusedThemes: true,
        styleOverrides: {
          // You can also override styles
          borderRadius: '0.5rem',
          borderColor: ['gray', 'black'],
          frames: {
            terminalTitlebarBackground: ['#414868', 'lightGray'],
            terminalTitlebarBorderBottomColor: ['gray', 'black'],
            terminalTitlebarDotsOpacity: '0.5',
            terminalTitlebarForeground: ['white', 'black']
          }
        },
      },
      components: {
        ThemeSelect: './src/components/ThemeSelect.astro',
        Pagination: './src/components/Pagination.astro',
      },
      title: 'BioTron',
      sidebar: [
        {
          label: 'Start Here',
          autogenerate: { directory: 'start-here' },
        },
        {
          label: 'Firmware',
          autogenerate: { directory: 'firmware' },
        },
        {
          label: 'ROS simulation',
          autogenerate: { directory: 'ros-simulation' },
        },
        {
          label: 'Control app',
          autogenerate: { directory: 'control-app' },
        },
      ],
    }),
  ],
});
