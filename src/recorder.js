// Drive recorder (Menu → Game → Drive recorder). While recording it samples the driver's inputs and the
// car's state at 60 Hz (every 4th physics step) plus the discrete actions (shifts, diff, TC...). Stopping
// copies the whole log to the clipboard as text (a header, CSV rows, the events) and the button is ready
// for the next run. Started / stopped with the HUD's Rec button or its key (F8).

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
    this.rows = []; this.events = []; this.t = 0; this.n = 0;
    this.note = '';            // shown on the button for a moment after a stop ('Copied', 'Copy failed')

    this.btn = document.createElement('button');
    this.btn.type = 'button'; this.btn.className = 'menu-btn rec-btn'; this.btn.tabIndex = -1; this.btn.hidden = true;
    this.btn.addEventListener('mousedown', e => e.preventDefault());
    this.btn.addEventListener('click', () => this.toggle());
    // last in the top row: its text changes while recording, so nothing after it may shift
    hudRoot.querySelector('.tl-row')?.appendChild(this.btn);

    this.label();
  }

  setEnabled(on) {
    this.on = !!on;
    this.btn.hidden = !this.on;
    if (!this.on && this.recording) this.stop();
  }

  toggle() {
    if (!this.on) return;
    if (this.recording) this.stop(); else this.start();
  }

  start() {
    this.rows = []; this.events = []; this.t = 0; this.n = 0;
    this.recording = true;
    this.note = '';
    this.label();
  }

  stop() {
    this.recording = false;
    const text = this.build();
    this.rows = []; this.events = [];
    this.copy(text);
  }

  async copy(text) {
    let ok = false;
    try { await navigator.clipboard.writeText(text); ok = true; } catch {
      // no clipboard permission (or no focus): a temporary text box and execCommand
      const ta = document.createElement('textarea');
      ta.value = text; ta.style.cssText = 'position:fixed;left:-9999px;top:0';
      document.body.appendChild(ta);
      try { ta.select(); ok = document.execCommand('copy'); } catch { ok = false; }
      ta.remove();
    }
    this.note = ok ? 'Copied' : 'Copy failed';
    this.label();
    clearTimeout(this.noteT);
    this.noteT = setTimeout(() => { this.note = ''; this.label(); }, 2000);
  }

  label() {
    this.btn.classList.toggle('rec-on', this.recording);
    this.btn.innerHTML = this.recording
      ? `<i class="rec-dot"></i><span>Stop ${this.t.toFixed(0)} s</span><span class="kc"><kbd class="cap">F8</kbd></span>`
      : `<i class="rec-dot"></i><span>${this.note || 'Rec'}</span><span class="kc"><kbd class="cap">F8</kbd></span>`;
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
