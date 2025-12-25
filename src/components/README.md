# Custom Components Documentation

This directory contains custom components created for the responsive and visually premium web app.

## Components

### HeroBackground
- **Purpose**: Creates a 3D-style background with gradient mesh, particle effects, and animations
- **Features**:
  - CSS-only animations for performance
  - Responsive design
  - Reduced motion support for accessibility
  - Loading state with spinner
- **Usage**: Import and include in any page that needs a premium background effect

### AIBrainIcon
- **Purpose**: Custom SVG icon representing AI/robotics theme
- **Features**:
  - Accessible with ARIA labels
  - Scalable vector graphics
  - Themable colors
- **Usage**: Use as a logo or icon throughout the application

### ThemeProvider
- **Purpose**: Provides theme context and switching functionality
- **Features**:
  - System preference detection
  - Local storage persistence
  - Theme toggle component
- **Usage**: Wrap your application with ThemeProvider and use useTheme hook or ThemeToggle component

## Styling Guidelines

### CSS Custom Properties (Variables)
The application uses CSS custom properties for consistent theming:
- Color palette: `--ifm-color-primary`, `--ifm-color-accent-purple`, etc.
- Spacing system: `--ifm-spacing`, `--ifm-spacing-double`, etc.
- Typography: `--ifm-h1-font-size`, `--ifm-font-size-base`, etc.
- Z-index levels: `--ifm-z-index-background`, `--ifm-z-index-content`, etc.

### Responsive Breakpoints
- Mobile: ≤640px
- Tablet: 641px-1024px
- Desktop: 1025px+
- Extra Large Desktop: 1400px+

### Accessibility Features
- Reduced motion support with `prefers-reduced-motion` media query
- Proper focus indicators
- Screen reader support
- Touch target minimum 44px