// UI layout check (docs/UI.md, "No layout shift"): opens the game, then clicks every segment button and
// switch in Settings and Graphics, and every chip and icon button in the tuning panel. After each click it
// compares the clicked control's box with its box before the click, and the menu header across tab
// switches. It prints whatever moved, plus page errors, and exits 1 if anything did.
//
//   npm run dev                        # in another terminal (port 5175)
//   node tools/uisweep.mjs [url]       # default http://localhost:5175/
// Needs Playwright (npm i --no-save playwright) and a Chromium (PW_CHROMIUM=/path, default /opt/pw-browsers/chromium).
import { chromium } from 'playwright';

const url = process.argv[2] || 'http://localhost:5175/';
const b = await chromium.launch({ executablePath: process.env.PW_CHROMIUM || '/opt/pw-browsers/chromium', args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const p = await b.newPage({ viewport: { width: 1280, height: 800 }, reducedMotion: 'reduce' });
const errs = [];
p.on('pageerror', e => errs.push(e.message));
await p.goto(url);
await p.waitForSelector('#loading.done', { timeout: 200000 });
await p.evaluate(() => { try { localStorage.clear(); } catch { /* fine */ } });

const sweep = (root, sel, skip) => p.evaluate(async ([root, sel, skip]) => {
  const out = [], wait = () => new Promise(r => setTimeout(r, 150));
  const all = () => [...document.querySelector(root).querySelectorAll(sel)].filter(b => b.getClientRects().length && !skip.some(s => b.matches(s)));
  const key = b => [b.closest('[data-key]')?.dataset.key, b.closest('[data-row]')?.dataset.row, b.closest('[data-sec]')?.dataset.sec,
    b.dataset.engine, b.dataset.chip, b.dataset.v, b.dataset.seg, b.dataset.act, b.dataset.live, b.dataset.box, b.dataset.gearbox].filter(x => x != null).join(':');
  const n = all().length;
  for (let i = 0; i < n; i++) {
    const b0 = all()[i];
    if (!b0) break;
    const id = key(b0), r0 = b0.getBoundingClientRect();
    b0.click();
    await wait();
    const b1 = b0.isConnected ? b0 : all().find(x => key(x) === id);
    if (!b1) { out.push(`${id}: gone after the click`); continue; }
    const r1 = b1.getBoundingClientRect();
    const d = Math.max(Math.abs(r1.x - r0.x), Math.abs(r1.y - r0.y), Math.abs(r1.width - r0.width));
    if (d > 0.5) out.push(`${id}: moved ${d.toFixed(1)} px`);
  }
  return out;
}, [root, sel, skip]);

const moved = [];
await p.keyboard.press('Escape');
await p.waitForSelector('#menu:not([hidden])', { timeout: 20000 });
const head = () => p.evaluate(() => [...document.querySelectorAll('.sheet-head > *')].map(e => { const r = e.getBoundingClientRect(); return [r.x, r.width].map(Math.round).join(','); }).join(' | '));
let h0 = null;
for (const t of ['locations', 'settings', 'graphics', 'controls']) {
  await p.click(`.tab[data-tab=${t}]`);
  await p.waitForTimeout(150);
  const h = await head();
  if (h0 && h !== h0) moved.push(`menu header changed on ${t}: ${h0} -> ${h}`);
  h0 = h;
  if (t === 'settings' || t === 'graphics') moved.push(...(await sweep('#menu .pane:not([hidden])', '.seg button, .switch', ['[data-key=fullscreen] .switch', '[data-key=sound] .switch'])).map(x => `${t} ${x}`));
}
await p.click('[data-act=resume]');
await p.keyboard.press('Tab');
await p.waitForSelector('#tuning:not([hidden])', { timeout: 20000 });
await p.evaluate(() => document.querySelectorAll('.tn-sec:not(.open) .tn-st').forEach(b => b.click()));
moved.push(...(await sweep('#tuning', '.tn-chips button, .tn-i, .tn-reset, .switch', ['[data-act=close]', '[data-act=saveas]', '[data-act=export]', '[data-act=import]', '[data-act=boxdel]', '[data-act=boxdup]', '[data-act=dyno]'])).map(x => `tuning ${x}`));
await b.close();

for (const m of moved) console.log('MOVED', m);
for (const e of errs) console.log('ERROR', e);
console.log(moved.length || errs.length ? `${moved.length} moved, ${errs.length} errors` : 'ok: nothing moved, no page errors');
process.exit(moved.length || errs.length ? 1 : 0);
