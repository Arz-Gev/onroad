export const PALETTES = {
  onroad: { name: 'Onroad', accent: '#ededed', cool: '#4cc2ff', hot: '#ff5c35' },
  ice: { name: 'Ice', accent: '#eaf2ff', cool: '#7dd3fc', hot: '#fb7185' },
  neon: { name: 'Neon', accent: '#f5f5f5', cool: '#22d3ee', hot: '#ff2d95' },
  mint: { name: 'Mint', accent: '#ecfdf5', cool: '#34d399', hot: '#f97316' },
  violet: { name: 'Violet', accent: '#f4f0ff', cool: '#a78bfa', hot: '#ff4d6d' },
  mono: { name: 'Mono', accent: '#e5e5e5', cool: '#ffffff', hot: '#ff453a' },
};
export const PALETTE_ORDER = ['onroad', 'ice', 'neon', 'mint', 'violet', 'mono'];

export const theme = { ...PALETTES.onroad, glass: 0.5, v: 0 };

const rgb = hex => [1, 3, 5].map(i => parseInt(hex.slice(i, i + 2), 16));
export const rgba = (hex, a) => `rgba(${rgb(hex).join(', ')}, ${a})`;

const lum = hex => {
  const [r, g, b] = rgb(hex).map(c => { c /= 255; return c <= 0.03928 ? c / 12.92 : ((c + 0.055) / 1.055) ** 2.4; });
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
};
const ink = hex => (lum(hex) > 0.35 ? '#0a0a0a' : '#ffffff');

export function applyTheme({ accent, cool, hot, glass }) {
  Object.assign(theme, { accent, cool, hot, glass, v: theme.v + 1 });
  const s = document.documentElement.style;
  s.setProperty('--accent', accent);
  s.setProperty('--accent-ink', ink(accent));
  s.setProperty('--cool', cool);
  s.setProperty('--cool-ink', ink(cool));
  s.setProperty('--cool-soft', rgba(cool, 0.16));
  s.setProperty('--hot', hot);
  s.setProperty('--hot-soft', rgba(hot, 0.16));
  s.setProperty('--panel', `rgba(0, 0, 0, ${glass})`);
  s.setProperty('--sheet', `rgba(0, 0, 0, ${Math.min(0.94, glass + 0.22).toFixed(2)})`);
}
