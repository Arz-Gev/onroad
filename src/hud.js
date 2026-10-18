import { capsHTML } from './input.js';
import { MESSAGES } from './messages.js';
import { theme, rgba } from './theme.js';

const SPEED_UNITS = { kmh: { k: 3.6, label: 'km/h' }, mph: { k: 2.236936, label: 'mph' } };
export const fmtPressure = (psi, unit) => unit === 'bar' ? `${(psi * 0.0689476).toFixed(2)} bar` : `${psi.toFixed(0)} psi`;
export const speedUnit = u => SPEED_UNITS[u] || SPEED_UNITS.kmh;

const SUSP = 216;
const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
const LIFE = { info: 2.8, warn: 3.8 };
const MAX_LINES = 3;

const svg = (p, extra = '') => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"${extra}>${p}</svg>`;
const LAMP_ICONS = {
  eng: svg('<path d="M7 8h3V6h5v2h1.5l2 2.5H20V9h1.5v7H20v-1.5h-1.5L16 17H9l-2-2H5v2.5H3.5v-7H5V12h2z"/>'),
  hb: svg('<circle cx="12" cy="12" r="6.5"/><path d="M10.4 15V9h2.3a1.7 1.7 0 0 1 0 3.4h-2.3M4.2 7.5a9.5 9.5 0 0 0 0 9M19.8 7.5a9.5 9.5 0 0 1 0 9"/>'),
  tc: '<b>TC</b>',
  abs: '<b>ABS</b>',
  low: svg('<path d="M10 6.5C6.5 6.5 4 9 4 12s2.5 5.5 6 5.5zM14 9l6 2M14 12.5l6 2M14 16l6 2"/>'),
  high: svg('<path d="M10 6.5C6.5 6.5 4 9 4 12s2.5 5.5 6 5.5zM14 8.5h6.5M14 12h6.5M14 15.5h6.5"/>'),
  haz: svg('<path d="M12 4.5 20.5 19h-17z"/><path d="M12 10 15.5 16h-7z"/>'),
  bar: svg('<rect x="3" y="9" width="18" height="6" rx="1.5"/><path d="M8 9v6M12 9v6M16 9v6"/>'),
  snd: svg('<path d="M4 9.5h3l4.5-3.5v12L7 14.5H4z"/><path d="M15 9.5c1 1.4 1 3.6 0 5M17.5 7.5c2.2 2.6 2.2 6.4 0 9"/>'),
  mute: svg('<path d="M4 9.5h3l4.5-3.5v12L7 14.5H4z"/><path d="m15.5 9.5 5 5M20.5 9.5l-5 5"/>'),
};
const LAMP_TITLES = { eng: 'Engine', hb: 'Handbrake', tc: 'Traction control', abs: 'ABS', light: 'Headlights', haz: 'Hazard lights', bar: 'Light bar', snd: 'Sound' };

export class HUD {
  constructor(root = document.getElementById('hud')) {
    this.root = root;
    root.innerHTML = `
      <div class="hud-tl">
        <div class="tl-row">
          <button class="menu-btn" id="h-menubtn" tabindex="-1" type="button" aria-label="Open menu">
            <svg viewBox="0 0 16 16" aria-hidden="true"><path d="M2.5 4h11M2.5 8h11M2.5 12h11" stroke="currentColor" stroke-width="1.6" stroke-linecap="round"/></svg>
            <span>Menu</span><span class="kc" id="h-menukey"></span>
          </button>
          <div class="hints" id="h-hints"></div>
        </div>
        <pre class="hud-panel telemetry" id="h-tele" hidden></pre>
      </div>
      <div class="hud-fps" id="h-fps" hidden></div>
      <div class="hud-panel susp" id="h-susp" hidden>
        <div class="panel-h"><span>Suspension and tyres</span><span id="h-suspkey"></span></div>
        <canvas id="h-suspcv"></canvas>
        <div class="panel-f"><span>load</span><span>tyre squash</span><span class="lg-travel">travel</span><span class="lg-slip">slip</span></div>
      </div>
      <div class="dash" id="h-dash">
        <div class="ticker" id="h-ticker" aria-live="polite"></div>
        <div class="cluster" id="h-cluster">
          <div class="rpm"><div class="rpm-track"><i class="rpm-red" id="h-rpmred"></i><i class="rpm-fill" id="h-rpmfill"></i></div><div class="rpm-scale" id="h-rpmscale"></div></div>
          <div class="cl-main">
            <div class="gear" id="h-gear">N</div>
            <div class="spd-box"><span class="spd" id="h-spd">0</span><span class="unit" id="h-unit">km/h</span></div>
            <svg class="car" id="h-car" viewBox="0 0 34 48" aria-hidden="true">
              <path class="axle" d="M7 11H27M7 37H27M17 11V37"/>
              <rect class="w" x="2" y="5" width="5" height="12" rx="1.5"/><rect class="w" x="27" y="5" width="5" height="12" rx="1.5"/>
              <rect class="w" x="2" y="31" width="5" height="12" rx="1.5"/><rect class="w" x="27" y="31" width="5" height="12" rx="1.5"/>
              <circle class="df" cx="17" cy="11" r="2.6"/><circle class="df" cx="17" cy="24" r="2.6"/><circle class="df" cx="17" cy="37" r="2.6"/>
            </svg>
            <div class="pedals" title="Throttle · brake · clutch">
              <div class="pd thr"><i id="p-thr"></i></div><div class="pd brk"><i id="p-brk"></i></div><div class="pd clu" id="p-cluwrap"><i id="p-clu"></i></div>
            </div>
          </div>
          <div class="lamps" id="h-lamps">${['eng', 'hb', 'tc', 'abs', 'light', 'haz', 'bar', 'snd'].map(id => `<span class="lamp" id="l-${id}" data-s="off" title="${LAMP_TITLES[id]}">${LAMP_ICONS[id === 'light' ? 'low' : id]}</span>`).join('')}</div>
          <div class="cl-info"><span id="h-box"></span><span id="h-drive"></span><span id="h-psi"></span><span id="h-surf"></span><span id="h-aids" class="aids" title="Steering help · stability control (Esc → Settings → Driving aids)"></span></div>
        </div>
      </div>`;
    const $ = id => root.querySelector('#' + id);
    this.$ = $;
    this.e = {
      menuBtn: $('h-menubtn'), menuKey: $('h-menukey'), hints: $('h-hints'),
      fps: $('h-fps'), tele: $('h-tele'), susp: $('h-susp'), suspKey: $('h-suspkey'),
      ticker: $('h-ticker'), cluster: $('h-cluster'),
      rpmFill: $('h-rpmfill'), rpmRed: $('h-rpmred'), rpmScale: $('h-rpmscale'),
      gear: $('h-gear'), spd: $('h-spd'), unit: $('h-unit'),
      wheels: [...root.querySelectorAll('#h-car .w')], diffs: [...root.querySelectorAll('#h-car .df')],
      thr: $('p-thr'), brk: $('p-brk'), clu: $('p-clu'), cluWrap: $('p-cluwrap'),
      box: $('h-box'), drive: $('h-drive'), psi: $('h-psi'), surf: $('h-surf'), aids: $('h-aids'),
      lamp: Object.fromEntries(['eng', 'hb', 'tc', 'abs', 'light', 'haz', 'bar', 'snd'].map(id => [id, $('l-' + id)])),
    };
    this.sctx = $('h-suspcv').getContext('2d');
    this.c = {};
    this.opts = { speedUnit: 'kmh', pressureUnit: 'psi', cluster: 'auto', hudScale: 1, hints: true, suspension: false, telemetry: false, fps: false, arcade: false };
    this.device = 'kb';
    this.lines = [];
    this.hintKey = null;
    this.tipState = { rolled: 0, stuck: 0, neutral: 0, engine: 0, clear: 0, acc: 0, id: null };
    this.fpsAcc = 0; this.fpsN = 0; this.suspAcc = 1; this.teleAcc = 1;
    this.soundState = 'locked';
    this.onMenu = null;
    this.e.menuBtn.addEventListener('mousedown', e => e.preventDefault());
    this.e.menuBtn.addEventListener('click', () => this.onMenu && this.onMenu());
    window.addEventListener('resize', () => this.resize());
    this.setDevice('kb', true);
    this.resize();
  }

  configure(opts) {
    Object.assign(this.opts, opts);
    const o = this.opts;
    this.e.susp.hidden = !o.suspension;
    this.e.tele.hidden = !o.telemetry;
    this.e.fps.hidden = !o.fps;
    this.e.hints.hidden = !o.hints;
    this.suspAcc = this.teleAcc = 1;
    if (this.suspPrev) this.suspPrev.fill(-1);
    this.c.spd = this.c.psi = this.c.box = undefined;
    this.e.unit.textContent = speedUnit(o.speedUnit).label;
    this.resize();
  }

  resize() {
    const W = window.innerWidth || 1440, H = window.innerHeight || 900;

    const s = clamp(Math.sqrt(Math.min(W / 1600, H / 1000)), 0.8, 1.4) * (this.opts.hudScale || 1);
    if (s === this.scale && W === this.W && H === this.H) return;
    this.scale = s; this.W = W; this.H = H;
    this.small = H < 560 || W < 760;
    this.root.style.setProperty('--s', s.toFixed(3));
    const dpr = Math.min(window.devicePixelRatio || 1, 3);
    const sp = Math.max(32, Math.round(SUSP * s * dpr));
    const scv = this.sctx.canvas; scv.width = scv.height = sp;
    this.suspAcc = 1; if (this.suspPrev) this.suspPrev.fill(-1);
    this.c.compact = undefined;
  }

  setDevice(d, force) {
    if (d === this.device && !force) return;
    this.device = d;
    const k = id => capsHTML(id, d);
    this.e.menuKey.innerHTML = k('menu');
    this.e.hints.innerHTML = d === 'pad'
      ? `<span>${k('camera')} Camera</span><span>${k('recover')} Recover</span><span>${k('shiftUp')}${k('shiftDown')} Shift</span>`
      : `<span>${k('controls')} Controls</span><span>${k('locations')} Locations</span><span>${k('camera')} Camera</span><span>${k('tuning')} Tuning</span><span>${k('recover')} Recover</span>`;
    this.e.suspKey.innerHTML = capsHTML('suspension', 'kb');

    for (const x of this.lines) this.setLine(x, this.render(x.key, x.params));
  }

  setSound(state) {
    if (state === this.soundState) return;
    const was = this.soundState;
    this.soundState = state;
    if (state === 'error' && was !== 'error') this.notify('soundError');
  }

  render(id, p = {}) {
    const m = MESSAGES[id];
    if (!m) { console.warn('hud: unknown message', id); return escapeHTML(id); }
    const ep = {};
    for (const [key, v] of Object.entries(p)) ep[key] = typeof v === 'string' ? escapeHTML(v) : v;
    return m.text(ep, k => capsHTML(k, this.device));
  }

  notify(id, params) {
    const m = MESSAGES[id];
    const level = m?.level === 'warn' ? 'warn' : 'info';
    this.pushLine(id, params, level, LIFE[level]);
  }

  setHint(id, params) {
    if (this.hintKey && this.hintKey !== id) { const x = this.lines.find(l => l.key === this.hintKey); if (x) this.dropLine(x); }
    this.hintKey = id || null;
    if (id) this.pushLine(id, params, 'hint', Infinity, false);
  }
  pushLine(key, params, level, t, nudge = true) {
    const html = this.render(key, params);
    const same = this.lines.find(x => x.key === key && !x.leaving);
    if (same) {
      same.t = t; same.params = params;
      const changed = this.setLine(same, html);

      if (nudge || changed) same.el.firstChild.animate?.([{ transform: 'translateX(calc(-4 * var(--u)))' }, { transform: 'none' }], { duration: 240, easing: 'ease-out' });
      return;
    }
    const el = document.createElement('div');
    el.className = `tk tk-${level} tk-new`;
    el.innerHTML = `<span class="tk-t">${html}</span>`;
    this.e.ticker.append(el);
    this.lines.push({ key, params, html, level, t, el });
    const live = this.lines.filter(x => !x.leaving);
    if (live.length > MAX_LINES) this.dropLine(live.find(x => x.level !== 'hint') || live[0]);
    this.layoutLines();
    requestAnimationFrame(() => requestAnimationFrame(() => el.classList.remove('tk-new')));
  }
  setLine(x, html) {
    if (x.html === html) return false;
    x.html = html;
    x.el.firstChild.innerHTML = html;
    return true;
  }
  dropLine(x) {
    if (x.leaving) return;
    x.leaving = true;
    x.el.classList.add('tk-out');
    setTimeout(() => { x.el.remove(); this.lines = this.lines.filter(y => y !== x); }, 340);
    this.layoutLines();
  }

  layoutLines() {
    const live = this.lines.filter(x => !x.leaving);
    live.forEach((x, i) => { const k = live.length - 1 - i; x.el.style.setProperty('--k', k); x.el.dataset.k = k; });
  }
  tickTicker(dt) {
    for (const x of this.lines) if (!x.leaving && (x.t -= dt) <= 0) this.dropLine(x);
  }

  update(dt, v, view, ctx = {}) {
    const d = v.drivetrain, e = this.e, c = this.c, o = this.opts;
    if (d.message && d.message.seq !== this._msgSeq) { this._msgSeq = d.message.seq; this.notify(d.message.id, d.message.params); }
    this.tickTicker(dt);

    const compact = o.cluster === 'compact' || (o.cluster === 'auto' && (ctx.cam === 'cockpit' || this.small));
    if (compact !== c.compact) { c.compact = compact; e.cluster.classList.toggle('compact', compact); }

    const E = v.P.engine;
    if (E !== c.E || E.limiterRpm !== c.lim || E.redlineRpm !== c.red) {
      c.E = E; c.lim = E.limiterRpm; c.red = E.redlineRpm;
      c.maxRpm = Math.ceil((E.limiterRpm + 100) / 1000) * 1000;
      e.rpmRed.style.left = (E.redlineRpm / c.maxRpm * 100).toFixed(2) + '%';
      let ticks = '';
      for (let r = 0; r <= c.maxRpm; r += 1000) ticks += `<i style="left:${(r / c.maxRpm * 100).toFixed(2)}%"${r >= E.redlineRpm ? ' class="hot"' : ''}>${r / 1000}</i>`;
      e.rpmScale.innerHTML = ticks;
      c.rpmQ = undefined;
    }
    const rpm = Math.max(0, d.rpm);
    const q = Math.round(clamp(rpm / c.maxRpm, 0, 1) * 200);
    if (q !== c.rpmQ) { c.rpmQ = q; e.rpmFill.style.transform = `scaleX(${q / 200})`; }
    const zone = rpm >= E.redlineRpm ? 'red' : rpm >= E.redlineRpm - 600 ? 'near' : '';
    if (zone !== c.rpmZone) { c.rpmZone = zone; e.rpmFill.className = 'rpm-fill ' + zone; }

    const gear = d.gearLabel();
    if (gear !== c.gear) { c.gear = gear; e.gear.textContent = gear; }
    const shifting = !!d.shift || d.grind > 0;
    if (shifting !== c.shifting) { c.shifting = shifting; e.gear.classList.toggle('shifting', shifting); }
    const su = speedUnit(o.speedUnit);
    const spd = Math.round(Math.abs(v.speed) * su.k);
    if (spd !== c.spd) { c.spd = spd; e.spd.textContent = spd; }

    for (let i = 0; i < 4; i++) {
      const w = v.wheels[i];
      const st = !w.contact ? 'w air' : (w.slipNorm || 0) > 0.95 ? 'w slip' : (w.slipNorm || 0) > 0.7 ? 'w edge' : 'w';
      if (st !== c['w' + i]) { c['w' + i] = st; e.wheels[i].setAttribute('class', st); }
    }

    const locks = (d.frontLock ? 1 : 0) | (d.centerLock ? 2 : 0) | (d.rearLock ? 4 : 0) | (d.fixedRwd ? 8 : 0);
    if (locks !== c.locks) {
      c.locks = locks;
      const cls = (on, show) => !show ? 'df none' : on ? 'df locked' : 'df';
      e.diffs[0].setAttribute('class', cls(d.frontLock, !d.fixedRwd));
      e.diffs[1].setAttribute('class', cls(d.centerLock, !d.fixedRwd));
      e.diffs[2].setAttribute('class', cls(d.rearLock, true));
    }

    const manual = d.mode === 'manual';
    this.bar(e.thr, 'thr', v.ctl.throttle);
    this.bar(e.brk, 'brk', Math.max(v.ctl.brake, v.ctl.handbrake));
    this.bar(e.clu, 'clu', manual ? clamp(d.clutchPedal / 0.8, 0, 1) : 0);
    const pedal = manual && !d.clutchAssist;
    if (pedal !== c.pedal) { c.pedal = pedal; e.cluWrap.classList.toggle('off', !pedal); }

    const L = e.lamp;

    this.lampState(L.eng, 'eng', d.running ? 'off' : d.cranking ? 'blink' : 'warn');
    this.lampState(L.hb, 'hb', v.ctl.handbrake > 0.5 ? 'act' : 'off');
    this.lampState(L.tc, 'tc', !v.tc ? 'off' : v.tcActive > 0 ? 'act' : 'on', v.tc);
    this.lampState(L.abs, 'abs', !v.abs ? 'off' : v.absActive > 0 ? 'act' : 'on', v.abs);
    const head = view.lights.head;
    if (head !== c.headIcon) { const first = c.headIcon === undefined; c.headIcon = head; L.light.innerHTML = LAMP_ICONS[head === 2 ? 'high' : 'low']; if (!first) this.flash(L.light); }
    this.lampState(L.light, 'light', head ? 'on' : 'off');
    this.lampState(L.haz, 'haz', view.lights.hazard ? 'blink' : 'off', view.lights.hazard);
    const hasBar = !!view.features?.lightBar;
    if (hasBar !== c.hasBar) { c.hasBar = hasBar; L.bar.hidden = !hasBar; }
    this.lampState(L.bar, 'bar', view.lights.bar ? 'on' : 'off', view.lights.bar);
    const snd = this.soundState;
    if (snd !== c.sndIcon) { const first = c.sndIcon === undefined; c.sndIcon = snd; L.snd.innerHTML = LAMP_ICONS[snd === 'on' ? 'snd' : 'mute']; if (!first) this.flash(L.snd); }
    this.lampState(L.snd, 'snd', snd === 'on' ? 'off' : snd === 'muted' ? 'on' : snd === 'locked' ? 'pulse' : 'warn');

    this.field(e.box, 'box', manual ? (d.clutchAssist ? 'Manual' : 'Manual · pedal') : (o.arcade ? 'Auto · arcade' : 'Auto'));
    const drive = d.fixedRwd ? { welded: 'Welded diff', lsd: 'LSD', open: 'Open diff' }[d.rearDiff] || 'RWD'
      : `${d.rwd ? '2WD' : '4WD'} · ${d.range === 'low' ? 'Low' : 'High'}`;
    this.field(e.drive, 'drive', drive);
    this.field(e.psi, 'psi', fmtPressure(v.pressure, o.pressureUnit));
    this.field(e.aids, 'aids', o.aids || '');
    let surf = 'Airborne';
    for (const w of v.wheels) if (w.contact) { surf = w.surf?.name || 'Ground'; break; }
    if (surf !== c.surf) { c.surf = surf; e.surf.textContent = surf; }

    if (ctx.paused) this.setHint(null);
    else if ((this.tipState.acc += dt) > 0.25) { this.tips(this.tipState.acc, v, ctx.raw || v.ctl); this.tipState.acc = 0; }

    if (o.suspension && (this.suspAcc += dt) > 0.05) { this.suspAcc = 0; this.drawSuspension(v); }
    if (o.telemetry && (this.teleAcc += dt) > 0.1) { this.teleAcc = 0; e.tele.textContent = this.telemetryText(v, ctx.telemetry ? ctx.telemetry() : ''); }
    this.fpsAcc += dt; this.fpsN++;
    if (this.fpsAcc > 0.5) { if (o.fps) e.fps.textContent = (this.fpsN / this.fpsAcc).toFixed(0) + ' fps'; this.fpsAcc = 0; this.fpsN = 0; }
  }

  bar(el, k, val) {
    const q = Math.round(clamp(val, 0, 1) * 100);
    if (q === this.c[k]) return;
    this.c[k] = q;
    el.style.transform = `scaleY(${q / 100})`;
  }

  lampState(el, k, s, sw) {
    const key = 'L' + k, swKey = 'S' + k;
    if (sw !== undefined && sw !== this.c[swKey]) {
      if (this.c[swKey] !== undefined) this.flash(el);
      this.c[swKey] = sw;
    }
    if (s === this.c[key]) return;
    this.c[key] = s;
    el.dataset.s = s;
  }

  field(el, k, text) {
    const key = 'F' + k;
    if (text === this.c[key]) return;
    const first = this.c[key] === undefined;
    this.c[key] = text;
    el.textContent = text;
    if (!first) this.flash(el);
  }
  flash(el) { el.animate?.([{ background: 'rgba(255,255,255,0.22)', color: '#fff' }, { background: 'rgba(255,255,255,0)' }], { duration: 650, easing: 'ease-out' }); }

  tips(dt, v, raw) {
    const d = v.drivetrain, ts = this.tipState;
    const sp = Math.abs(v.speed);
    const thr = raw.throttle || 0;
    const inGear = d.mode === 'manual' ? d.manualGear !== 0 : (d.selector === 'D' || d.selector === 'R');
    const grow = (name, cond) => { ts[name] = cond ? ts[name] + dt : 0; return ts[name]; };
    const rolled = grow('rolled', v.up.y < 0.35);
    const engine = grow('engine', !d.running && !d.cranking);
    const neutral = grow('neutral', d.running && !inGear && thr > 0.5 && (d.mode === 'manual' || sp > 0.8));
    const stuck = grow('stuck', d.running && inGear && thr > 0.6 && sp < 0.6 && v.up.y > 0.35);
    let id = null, p;
    if (this.soundState === 'locked') id = 'soundLocked';
    else if (rolled > 1.0) id = 'tipRolled';
    else if (engine > 0.6) { id = 'tipEngine'; p = { stalled: d.stalled }; }
    else if (neutral > 1.2) { id = 'tipNeutral'; p = { manual: d.mode === 'manual', sel: d.selector }; }
    else if (stuck > 3) { id = 'tipStuck'; p = { offroad: !d.fixedRwd }; }
    if (!id && ts.id) { if ((ts.clear += dt) < 0.75) return; }
    ts.clear = 0; ts.id = id;
    this.setHint(id, p);
  }

  drawSuspension(v) {
    const q = this.suspKey || (this.suspKey = new Int32Array(27)), prev = this.suspPrev || (this.suspPrev = new Int32Array(27).fill(-1));
    for (let i = 0; i < 4; i++) {
      const w = v.wheels[i], ax = w.axle;
      const comp = v.wheelComp(w);
      const slip = w.slipNorm || 0;
      q[i * 6] = Math.round((w.FnAvg || 0) / 9.81);
      q[i * 6 + 1] = Math.round(Math.max(0, w.pen) * 1000);
      q[i * 6 + 2] = Math.round(clamp(comp / ax.p.travel, 0, 1) * 100);
      q[i * 6 + 3] = !w.contact ? 3 : slip > 0.95 ? 2 : slip > 0.7 ? 1 : 0;
      q[i * 6 + 4] = Math.round(w.steer * 100);
      q[i * 6 + 5] = Math.round(Math.min(1, (w.FnAvg || 0) / 12000) * 20);
    }
    q[24] = Math.round(v.axleRoll(0) * 573); q[25] = Math.round(v.axleRoll(1) * 573);
    q[26] = theme.v;
    if (q.every((x, i) => x === prev[i])) return;
    prev.set(q);
    const g = this.sctx, px = g.canvas.width, k = px / 240;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, px, px);
    g.setTransform(k, 0, 0, k, 0, 0);
    const pos = [[62, 70], [178, 70], [62, 172], [178, 172]];
    g.strokeStyle = 'rgba(255,255,255,0.22)';
    g.lineWidth = 2.5;
    g.beginPath(); g.moveTo(62, 70); g.lineTo(178, 70); g.moveTo(62, 172); g.lineTo(178, 172); g.moveTo(120, 70); g.lineTo(120, 172); g.stroke();
    g.font = '500 12px Geist, ui-sans-serif, system-ui, sans-serif';
    g.textAlign = 'center'; g.textBaseline = 'middle';
    for (let i = 0; i < 4; i++) {
      const w = v.wheels[i], p = w.axle.p;
      const comp = v.wheelComp(w);
      const f = clamp(comp / p.travel, 0, 1);
      const [x, y] = pos[i];
      const slip = Math.min(1, w.slipNorm || 0);
      const load = Math.min(1, (w.FnAvg || 0) / 12000);
      const col = !w.contact ? 'rgba(140,140,140,0.4)' : slip > 0.95 ? theme.cool : slip > 0.7 ? theme.accent : rgba(theme.accent, 0.45);
      g.save();
      g.translate(x, y);
      g.rotate(w.steer);
      g.fillStyle = 'rgba(0,0,0,0.4)';
      g.fillRect(-12, -22, 24, 44);
      g.fillStyle = col;
      g.globalAlpha = 0.35 + 0.65 * load;
      g.fillRect(-10, -20, 20, 40);
      g.globalAlpha = 1;
      g.restore();
      const bx = x + (w.side < 0 ? -28 : 21);
      g.fillStyle = 'rgba(255,255,255,0.14)';
      g.fillRect(bx, y - 22, 7, 44);
      g.fillStyle = f > 0.9 || f < 0.05 ? theme.hot : rgba(theme.accent, 0.7);
      g.fillRect(bx, y + 22 - 44 * f, 7, 44 * f);
      const ty = i < 2 ? y - 46 : y + 36;
      g.fillStyle = 'rgba(255,255,255,0.92)';
      g.fillText(`${((w.FnAvg || 0) / 9.81).toFixed(0)} kg`, x, ty);
      g.fillStyle = 'rgba(255,255,255,0.55)';
      g.fillText(`${Math.max(0, w.pen * 100).toFixed(1)} cm`, x, ty + 14);
    }
    g.fillStyle = 'rgba(255,255,255,0.6)';
    g.font = '500 11px Geist, ui-sans-serif, system-ui, sans-serif';
    g.fillText(`${v.indep ? 'susp roll' : 'axle twist'}  F ${(v.axleRoll(0) * 57.3).toFixed(1)}°  R ${(v.axleRoll(1) * 57.3).toFixed(1)}°`, 120, 121);
  }

  telemetryText(v, extra) {
    const d = v.drivetrain;
    const f = (x, n = 2) => (x >= 0 ? ' ' : '') + x.toFixed(n);
    let s = '';
    s += `rpm ${d.rpm.toFixed(0)}  thr ${d.thr.toFixed(2)}  Tcomb ${d.Tcomb.toFixed(0)}Nm  load ${d.load.toFixed(2)}  clutch ${d.clutchPedal.toFixed(2)} slip ${d.clutchSlip.toFixed(1)}  lockup ${d.lockup.toFixed(2)}\n`;
    s += `ratio ${d.currentRatio().toFixed(2)}  prop F ${d.propTorque[0].toFixed(0)} R ${d.propTorque[1].toFixed(0)} Nm  speed ${(v.speed * 3.6).toFixed(1)} km/h\n`;
    s += `wheel  ω(rad/s)  Fn(N)   Fx     Fy    slip  pen(cm)  surf\n`;
    v.wheels.forEach((w, i) => {
      s += `${['FL', 'FR', 'RL', 'RR'][i]}   ${f(d.w[2 + i], 1).padStart(7)} ${w.FnAvg.toFixed(0).padStart(6)} ${w.Fx.toFixed(0).padStart(6)} ${w.Fy.toFixed(0).padStart(6)}  ${(w.slipNorm || 0).toFixed(2)}  ${(w.pen * 100).toFixed(1).padStart(5)}  ${w.surf.name}\n`;
    });
    if (v.indep) v.wheels.forEach((w, i) => { s += `${['FL', 'FR', 'RL', 'RR'][i]} c ${w.c.toFixed(3)}  spring ${w.S.toFixed(0)} N${i % 2 ? '\n' : '   '}`; });
    else v.axles.forEach((a, i) => { s += `axle ${i ? 'R' : 'F'} c ${a.c.toFixed(3)}  φ ${(a.phi * 57.3).toFixed(1)}°  springs ${a.S[0].toFixed(0)} / ${a.S[1].toFixed(0)} N\n`; });
    if (extra) s += extra;
    return s;
  }
}

export function escapeHTML(s) {
  return String(s).replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
}
