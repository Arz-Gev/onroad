// Drive recorder (Menu → Game → Drive recorder). While recording it samples the driver's inputs and the
// car's state at 60 Hz (every 4th physics step) plus the discrete actions (shifts, diff, TC...). Stopping
// copies the whole log to the clipboard as text (a header, CSV rows, the events) and opens a panel with
// it to copy again or clear. Started / stopped with the HUD's Rec button or its key (F8).

const WHEELS = ['fl', 'fr', 'rl', 'rr'];
const COLS = ['t', 'thr', 'brk', 'steerIn', 'clutchIn', 'hb', 'steerDeg', 'kmh', 'vx', 'vy', 'betaDeg', 'yawDps',
  'rollDeg', 'pitchDeg', 'rpm', 'gear', 'clutch', 'x', 'y', 'z', 'headingDeg',
  ...WHEELS.flatMap(w => [`${w}Kmh`, `${w}Slip`, `${w}Fx`, `${w}Fy`, `${w}Fn`, `${w}TravelCm`, `${w}Surf`])];
const EVERY = 4;           // physics steps per sample (240 Hz / 4 = 60 Hz)
const MAX_ROWS = 60 * 600; // 10 minutes

const deg = r => r * 57.2958;
const f = (x, n = 2) => (Number.isFinite(x) ? x.toFixed(n) : '');

export class Recorder {
  constructor({ hudRoot, meta }) {
    this.meta = meta;          // () => { car, map, ... } for the log header
    this.on = false;           // the feature switch (setting)
    this.recording = false;
    this.rows = []; this.events = []; this.t = 0; this.n = 0; this.text = '';

    this.btn = document.createElement('button');
    this.btn.type = 'button'; this.btn.className = 'menu-btn rec-btn'; this.btn.tabIndex = -1; this.btn.hidden = true;
    this.btn.addEventListener('mousedown', e => e.preventDefault());
    this.btn.addEventListener('click', () => this.toggle());
    hudRoot.querySelector('.tl-row')?.insertBefore(this.btn, hudRoot.querySelector('#h-hints'));

    this.panel = document.createElement('div');
    this.panel.className = 'rec-panel'; this.panel.hidden = true;
    this.panel.innerHTML = `
      <div class="rec-h"><span id="rec-title">Drive log</span><span class="rec-acts">
        <button type="button" data-a="copy">Copy</button><button type="button" data-a="reset">Reset</button><button type="button" data-a="close">Close</button>
      </span></div>
      <textarea readonly spellcheck="false"></textarea>`;
    this.area = this.panel.querySelector('textarea');
    this.title = this.panel.querySelector('#rec-title');
    this.titleBase = 'Drive log';
    // the game swallows plain keys; let the text box keep its own (select all, copy, arrows)
    this.panel.addEventListener('keydown', e => e.stopPropagation());
    this.panel.addEventListener('click', e => {
      const a = e.target.closest('button')?.dataset.a;
      if (a === 'copy') this.copy();
      else if (a === 'reset') this.reset();
      else if (a === 'close') this.panel.hidden = true;
    });
    document.body.appendChild(this.panel);
    this.label();
  }

  setEnabled(on) {
    this.on = !!on;
    this.btn.hidden = !this.on;
    if (!this.on) { if (this.recording) this.stop(); this.panel.hidden = true; }
  }

  toggle() {
    if (!this.on) return;
    if (this.recording) this.stop(); else this.start();
  }

  start() {
    this.rows = []; this.events = []; this.t = 0; this.n = 0;
    this.recording = true;
    this.panel.hidden = true;
    this.label();
  }

  stop() {
    this.recording = false;
    this.text = this.build();
    this.area.value = this.text;
    this.setTitle(`Drive log · ${this.t.toFixed(1)} s · ${this.rows.length} rows`);
    this.panel.hidden = false;
    this.label();
    this.copy();
  }

  reset() {
    this.rows = []; this.events = []; this.text = ''; this.area.value = '';
    this.setTitle('Drive log');
    this.label();
  }

  async copy() {
    if (!this.text) return;
    let ok = false;
    try { await navigator.clipboard.writeText(this.text); ok = true; } catch {
      // no clipboard permission (or no focus): fall back to selecting the text box and execCommand
      try { this.area.focus(); this.area.select(); ok = document.execCommand('copy'); this.area.blur(); } catch { ok = false; }
    }
    this.flash(ok ? 'Copied to clipboard' : 'Copy failed: select the text and copy it');
  }

  setTitle(t) {
    clearTimeout(this.flashT);
    this.titleBase = this.title.textContent = t;
  }

  flash(msg) {
    this.title.textContent = `${this.titleBase} — ${msg}`;
    clearTimeout(this.flashT);
    this.flashT = setTimeout(() => { this.title.textContent = this.titleBase; }, 2500);
  }

  label() {
    this.btn.classList.toggle('rec-on', this.recording);
    this.btn.innerHTML = this.recording
      ? `<i class="rec-dot"></i><span>Stop ${this.t.toFixed(0)} s</span><span class="kc"><kbd class="cap">F8</kbd></span>`
      : `<i class="rec-dot"></i><span>Rec</span><span class="kc"><kbd class="cap">F8</kbd></span>`;
  }

  // a discrete action (shift, diff, TC...): logged with the time it happened
  event(id) { if (this.recording) this.events.push(`${f(this.t, 3)} ${id}`); }

  // after every physics step
  step(h, raw, v) {
    if (!this.recording) return;
    this.t += h;
    if (this.n++ % EVERY) return;
    if (this.rows.length >= MAX_ROWS) { this.stop(); return; }
    if (this.rows.length % 60 === 0) this.label();
    const d = v.drivetrain;
    const vx = v.vel.dot(v.fwd), vy = v.vel.dot(v.right), r = v.angVel.dot(v.up);
    const row = [f(this.t, 3), f(raw.throttle), f(raw.brake), f(raw.steer), f(raw.clutch), f(raw.handbrake),
      f(deg(v.steerAngle), 1), f(v.speed * 3.6, 1), f(vx), f(vy), f(deg(Math.atan2(vy, Math.max(Math.abs(vx), 1))), 1), f(deg(r), 1),
      f(deg(Math.asin(Math.max(-1, Math.min(1, v.right.y)))), 1), f(deg(Math.asin(Math.max(-1, Math.min(1, v.fwd.y)))), 1),
      f(d.rpm, 0), d.gearLabel(), f(d.clutchPedal), f(v.pos.x), f(v.pos.y), f(v.pos.z), f(deg(v.yaw()), 1)];
    for (let i = 0; i < 4; i++) {
      const w = v.wheels[i];
      row.push(f(d.w[2 + i] * v.R * 3.6, 1), f(w.slipNorm), f(w.Fx, 0), f(w.Fy, 0), f(w.Fn, 0), f((w.c ?? 0) * 100, 1),
        w.contact ? (w.surf?.name || '?') : 'air');
    }
    this.rows.push(row.join(','));
  }

  build() {
    const m = this.meta?.() || {};
    const head = [
      '# onroad drive log',
      `# recorded ${new Date().toISOString()} · ${this.t.toFixed(2)} s · ${this.rows.length} samples at 60 Hz`,
      ...Object.entries(m).map(([k, val]) => `# ${k}: ${typeof val === 'object' ? JSON.stringify(val) : val}`),
      '# inputs are the raw driver inputs (steerIn -1..1, + = right); steerDeg the road-wheel angle; vy + = sliding right;',
      '# betaDeg body slip angle; slip = tyre combined slip / peak (>1 = past the grip peak); F in N; travel = spring compression',
      '',
      COLS.join(','),
    ];
    return [...head, ...this.rows, '', '# events (t action)', ...(this.events.length ? this.events : ['(none)'])].join('\n');
  }
}
