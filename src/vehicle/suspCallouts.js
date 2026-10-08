import * as THREE from 'three';

// Suspension view cards (ColliderView 'susp'): one card per wheel, on screen next to the wheel and tied to
// its hub by a leader line. A card: travel bar (fill = compression from full droop, top zone = bump stop,
// blue tick = static ride height, thin tick = recent peak), travel used, tyre load against static, and the
// suspension speed (up = bump, down = rebound). The cards sit outward from the car on screen and push each
// other apart so they never overlap. Bars move every frame; the numbers change 10 times a second.
// Styles: tuning.css (sv-).

const NAMES = ['FL', 'FR', 'RL', 'RR'];
const CW = 150, CH = 82, GAP = 8, REACH = 70;   // card size, gap between cards, hub to card centre (px)
const _p = new THREE.Vector3();
const pct = (x, t) => `${Math.max(0, Math.min(100, x / t * 100)).toFixed(1)}%`;

export class SuspCallouts {
  constructor() { this.el = null; this.acc = 1; }

  build() {
    const el = document.createElement('div');
    el.className = 'sv';
    el.hidden = true;
    el.setAttribute('aria-hidden', 'true');
    el.innerHTML = `<svg class="sv-lines">${NAMES.map(() => '<line></line><circle r="4"></circle>').join('')}</svg>` + NAMES.map(n => `
      <div class="sv-card">
        <div class="sv-bar"><i class="sv-stop"></i><i class="sv-fill"></i><i class="sv-static"></i><i class="sv-peak"></i></div>
        <div class="sv-txt">
          <div class="sv-h"><b>${n}</b><span class="sv-st"></span></div>
          <div class="sv-big"><span class="sv-pc"></span><small>travel</small></div>
          <div class="sv-row"><span class="sv-load"></span><span class="sv-dl"></span></div>
          <div class="sv-row sv-spd"><span class="sv-dir"></span><span class="sv-ms"></span></div>
        </div>
      </div>`).join('');
    document.body.appendChild(el);
    this.el = el;
    const lines = el.querySelectorAll('line'), dots = el.querySelectorAll('circle');
    this.cards = [...el.querySelectorAll('.sv-card')].map((c, i) => ({
      el: c, line: lines[i], dot: dots[i], x: 0, y: 0, hx: 0, hy: 0, show: false,
      fill: c.querySelector('.sv-fill'), stop: c.querySelector('.sv-stop'), stat: c.querySelector('.sv-static'), peak: c.querySelector('.sv-peak'),
      st: c.querySelector('.sv-st'), pc: c.querySelector('.sv-pc'), load: c.querySelector('.sv-load'), dl: c.querySelector('.sv-dl'),
      spd: c.querySelector('.sv-spd'), dir: c.querySelector('.sv-dir'), ms: c.querySelector('.sv-ms'), cls: '',
    }));
  }

  setVisible(on) {
    if (typeof document === 'undefined') return;
    if (on && !this.el) this.build();
    if (this.el) this.el.hidden = !on;
    this.acc = 1;
  }

  update(camera, root, v, corners, dt) {
    if (!this.el || corners.length < 4) return;
    const W = window.innerWidth, H = window.innerHeight;
    camera.updateMatrixWorld();
    root.updateMatrixWorld();
    const toScreen = p => { p.project(camera); return p.z < 1 && p.z > -1 ? [(p.x + 1) / 2 * W, (1 - p.y) / 2 * H] : null; };
    const ctr = toScreen(_p.setFromMatrixPosition(root.matrixWorld));
    if (!ctr) { for (const c of this.cards) c.el.style.visibility = c.line.style.visibility = c.dot.style.visibility = 'hidden'; return; }
    // card centres: outward from the car's centre on screen, front cards nudged up, rear down
    this.cards.forEach((c, i) => {
      const hub = toScreen(v.hubLocal(v.wheels[i], _p).applyMatrix4(root.matrixWorld));
      c.show = !!hub;
      if (!hub) return;
      [c.hx, c.hy] = hub;
      let dx = c.hx - ctr[0], dy = c.hy - ctr[1];
      const l = Math.hypot(dx, dy);
      if (l < 1) { dx = v.wheels[i].side; dy = 0; } else { dx /= l; dy /= l; }
      c.x = c.hx + dx * (REACH + CW / 2 * Math.abs(dx));
      c.y = c.hy + dy * REACH * 0.6 + (i < 2 ? -CH * 0.35 : CH * 0.35);
    });
    // push overlapping cards apart (mostly vertically), then keep them on screen
    const vis = this.cards.filter(c => c.show);
    for (let it = 0; it < 8; it++) {
      for (let a = 0; a < vis.length; a++) for (let b = a + 1; b < vis.length; b++) {
        const A = vis[a], B = vis[b];
        const ox = CW + GAP - Math.abs(A.x - B.x), oy = CH + GAP - Math.abs(A.y - B.y);
        if (ox <= 0 || oy <= 0) continue;
        if (oy <= ox) { const s = (A.y < B.y || (A.y === B.y && a < b) ? -1 : 1) * oy / 2; A.y += s; B.y -= s; }
        else { const s = (A.x < B.x ? -1 : 1) * ox / 2; A.x += s; B.x -= s; }
      }
      for (const c of vis) {
        c.x = Math.max(CW / 2 + 8, Math.min(W - CW / 2 - 8, c.x));
        c.y = Math.max(CH / 2 + 52, Math.min(H - CH / 2 - 8, c.y));
      }
    }
    const text = (this.acc += dt) >= 0.1;
    if (text) this.acc = 0;
    this.cards.forEach((c, i) => {
      const vis = c.show ? 'visible' : 'hidden';
      c.el.style.visibility = c.line.style.visibility = c.dot.style.visibility = vis;
      if (!c.show) return;
      const x0 = c.x - CW / 2, y0 = c.y - CH / 2;
      c.el.style.transform = `translate(${x0.toFixed(1)}px, ${y0.toFixed(1)}px)`;
      // leader: hub to the nearest point of the card
      const px = Math.max(x0, Math.min(x0 + CW, c.hx)), py = Math.max(y0, Math.min(y0 + CH, c.hy));
      c.line.setAttribute('x1', c.hx.toFixed(1)); c.line.setAttribute('y1', c.hy.toFixed(1));
      c.line.setAttribute('x2', px.toFixed(1)); c.line.setAttribute('y2', py.toFixed(1));
      c.dot.setAttribute('cx', c.hx.toFixed(1)); c.dot.setAttribute('cy', c.hy.toFixed(1));
      const k = corners[i], t = k.travel;
      c.fill.style.height = pct(k.compNow, t);
      c.stop.style.height = pct(t - k.stop, t);
      c.stat.style.bottom = pct(k.staticComp, t);
      c.peak.style.bottom = pct(k.peak, t);
      const cls = k.onStop ? 'sv-card hit' : k.air ? 'sv-card air' : 'sv-card';
      if (cls !== c.cls) { c.cls = cls; c.el.className = cls; c.line.setAttribute('class', k.onStop ? 'hit' : ''); c.dot.setAttribute('class', k.onStop ? 'hit' : ''); }
      if (!text) return;
      c.st.textContent = k.onStop ? 'Bump stop' : k.air ? 'In the air' : k.topped ? 'Full droop' : '';
      c.pc.textContent = `${Math.round(Math.max(0, Math.min(1, k.comp / t)) * 100)}%`;
      c.load.textContent = k.air ? '0.0 kN' : `${(k.load / 1000).toFixed(1)} kN`;
      const d = Math.round((k.load / Math.max(1, k.staticLoad) - 1) * 100);
      c.dl.textContent = k.air ? '' : d === 0 ? 'static' : `${d > 0 ? '+' : '−'}${Math.abs(d)}%`;
      c.dl.className = 'sv-dl' + (!k.air && d >= 25 ? ' up' : '');
      const r = k.rate, fast = Math.abs(r) >= 0.25;
      c.dir.textContent = Math.abs(r) < 0.02 ? '·' : r > 0 ? '▲' : '▼';
      c.ms.textContent = `${Math.abs(r).toFixed(2)} m/s`;
      c.spd.classList.toggle('fast', fast);
    });
  }

  dispose() { this.el?.remove(); this.el = null; }
}
