// Shared structural color tokens for hand-built consoles (a machine's own console.js,
// written as literal CSS/SVG strings rather than the kit's CSS variables). kit/style.css
// already drives the generic HUD/director chrome from CSS custom properties and defaults
// dark (`:root` is dark, `:root[data-theme='light']` is the override) — but a console that
// paints its own fixed-layout panels with inline styles bypasses that system entirely and
// typically hardcodes one light-only palette (see the 3 examples' console.js). These tokens
// let such a console support both themes with one switch instead of a rewrite: resolve
// `tokens(cfg.theme)` once inside `createConsole`, keep each machine's own domain/semantic
// colors (the hues that label a *state* — combustion, exhaust, a tracked part — not the
// chrome) constant across themes, and use these for everything structural: text, lines,
// panel fills, and the shades every console needs for "off/neutral" surfaces or subtle
// highlight washes that used to be a bare `#fff`, `#e6e7e9` or `rgba(20,22,26,.05)` literal.
//
//   import {tokens} from '../kit/theme.js';
//   export function createConsole(app, {theme = 'light'} = {}) {
//     const C = {...tokens(theme), red: '#d6182b', blue: '#3d8fe0', ...};
//     ...
//
// `surface` doubles as the text/icon color for anything painted on an `ink`-filled toggle
// (a "this one is active" pill): ink and surface sit at opposite ends of the lightness scale
// in both themes, so swapping fill <-> text between them keeps the inversion working after
// a theme switch. A toggle filled with a fixed accent (red, say) instead of ink should keep
// its own hardcoded light text — the accent's lightness does not change between themes.
//
// Values are chosen to match kit/style.css's own dark :root exactly, so a console and the
// kit's generic chrome never disagree about what "dark" means.
export const LIGHT = {
  bg: '#f3f2ee',
  ink: '#16181b',
  mute: '#8b8f95',
  faint: '#c9ccd0',
  line: '#e1e3e6',
  panel: 'rgba(248, 248, 249, .94)',
  surface: '#ffffff', // a panel's "off" / neutral fill: buttons, unlit gauge shapes
  track: '#e6e7e9', // a bar or slider's unfilled background
  wash: 'rgba(20, 22, 26, .05)', // a subtle highlight wash (e.g. "the active segment") over a panel
};

export const DARK = {
  bg: '#09090a',
  ink: '#ece6d6',
  mute: 'rgba(236, 230, 214, 0.64)',
  faint: 'rgba(236, 230, 214, 0.22)',
  line: 'rgba(236, 230, 214, 0.14)',
  panel: 'rgba(9, 9, 10, 0.9)',
  surface: '#1c1c1e',
  track: 'rgba(236, 230, 214, 0.12)',
  wash: 'rgba(236, 230, 214, .08)',
};

export const tokens = (theme) => (theme === 'dark' ? DARK : LIGHT);
