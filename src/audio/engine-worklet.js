class Biquad {
  constructor() { this.x1 = 0; this.x2 = 0; this.y1 = 0; this.y2 = 0; this.b0 = 1; this.b1 = 0; this.b2 = 0; this.a1 = 0; this.a2 = 0; }
  bandpass(f, q, sr) {
    const w = 2 * Math.PI * Math.min(f, sr * 0.45) / sr, a = Math.sin(w) / (2 * q), c = Math.cos(w), n = 1 + a;
    this.b0 = a / n; this.b1 = 0; this.b2 = -a / n; this.a1 = -2 * c / n; this.a2 = (1 - a) / n;
  }
  highpass(f, q, sr) {
    const w = 2 * Math.PI * f / sr, a = Math.sin(w) / (2 * q), c = Math.cos(w), n = 1 + a;
    this.b0 = (1 + c) / 2 / n; this.b1 = -(1 + c) / n; this.b2 = this.b0; this.a1 = -2 * c / n; this.a2 = (1 - a) / n;
  }
  lowpass(f, q, sr) {
    const w = 2 * Math.PI * Math.min(f, sr * 0.45) / sr, a = Math.sin(w) / (2 * q), c = Math.cos(w), n = 1 + a;
    this.b0 = (1 - c) / 2 / n; this.b1 = (1 - c) / n; this.b2 = this.b0; this.a1 = -2 * c / n; this.a2 = (1 - a) / n;
  }
  lowshelf(f, gainDb, sr) {
    const A = Math.pow(10, gainDb / 40), w = 2 * Math.PI * f / sr, c = Math.cos(w), al = Math.sin(w) / 2 * Math.SQRT2, sa = 2 * Math.sqrt(A) * al;
    const n = (A + 1) + (A - 1) * c + sa;
    this.b0 = A * ((A + 1) - (A - 1) * c + sa) / n; this.b1 = 2 * A * ((A - 1) - (A + 1) * c) / n; this.b2 = A * ((A + 1) - (A - 1) * c - sa) / n;
    this.a1 = -2 * ((A - 1) + (A + 1) * c) / n; this.a2 = ((A + 1) + (A - 1) * c - sa) / n;
  }
  run(x) {
    const y = this.b0 * x + this.b1 * this.x1 + this.b2 * this.x2 - this.a1 * this.y1 - this.a2 * this.y2;
    this.x2 = this.x1; this.x1 = x; this.y2 = this.y1; this.y1 = y;
    return y;
  }
}

const STARTER_DEFAULTS = {
  unevenness: 0.65,
  puff: 0.06,
  pops: 3,
  level: 0.015,
  pitch: 1.1,
  mesh: 1.83,
  mesh2: 0,
  q1: 10, q2: 30,
  wobble: 0.045,
  labour: 1,
  noise: 0.012,
  noiseHz: 850,
};

class EngineProcessor extends AudioWorkletProcessor {
  static get parameterDescriptors() {
    return [
      { name: 'rpm', defaultValue: 0, minValue: -2000, maxValue: 9000, automationRate: 'k-rate' },
      { name: 'load', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
      { name: 'throttle', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
      { name: 'starter', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },

      { name: 'running', defaultValue: 0, minValue: 0, maxValue: 1, automationRate: 'k-rate' },
    ];
  }
  constructor() {
    super();
    this.sr = sampleRate;
    this.phase = 0;
    this.bank = [0, 1, 1, 0, 1, 0, 0, 1];
    this.cylGain = [1.0, 0.9, 1.08, 0.95, 1.05, 0.92, 1.06, 0.97];
    this.jit = [0, 0.006, -0.004, 0.003, -0.005, 0.004, 0.002, -0.003];
    this.env = [0, 0];
    this.pres = [0, 0];
    this.rpm = 0; this.load = 0; this.thr = 0;
    this.seed = 22222;
    this.res = [[new Biquad(), new Biquad(), new Biquad()], [new Biquad(), new Biquad(), new Biquad()]];
    this.lp = new Biquad(); this.lp2 = new Biquad();
    this.hp = new Biquad(); this.hp.highpass(34, 0.7, this.sr);
    this.hp2 = new Biquad(); this.hp2.highpass(34, 0.7, this.sr);
    this.intake = new Biquad();
    this.mech = new Biquad();
    this.delay = new Float32Array(256); this.dp = 0;
    this.k = 0;
    this.crackle = 0;
    this.starterBp = new Biquad(); this.starterR1 = new Biquad(); this.starterR2 = new Biquad();
    this.starterR1.bandpass(450, 5, this.sr); this.starterR2.bandpass(900, 7, this.sr); this.starterBp.bandpass(900, 0.9, this.sr);
    this.shelf = new Biquad(); this.shelf.lowshelf(130, 3, this.sr);
    this.wander = 0; this.wanderT = 0;
    this.tick = 0;

    this.st = { ...STARTER_DEFAULTS };
    if (this.port) this.port.onmessage = e => { if (e.data && e.data.starter) Object.assign(this.st, e.data.starter); };
  }
  rand() { this.seed = (this.seed * 1103515245 + 12345) & 0x7fffffff; return this.seed / 0x7fffffff; }

  updateFilters() {
    const sr = this.sr, r = Math.max(this.rpm, 50);

    const t = 1 + this.load * 0.08 + r / 40000;
    const f = [[92 * t, 1.4], [255 * t, 2.6], [820 * t, 2.0]];
    const g = [[104 * t, 1.5], [290 * t, 2.4], [930 * t, 2.2]];
    for (let i = 0; i < 3; i++) { this.res[0][i].bandpass(f[i][0], f[i][1], sr); this.res[1][i].bandpass(g[i][0], g[i][1], sr); }

    const cut = 350 + r * 0.5 + this.load * (400 + r * 0.5) + 200 * this.thr;
    this.lp.lowpass(cut, 0.8, sr);
    this.lp2.lowpass(cut * 1.6, 0.7, sr);
    this.intake.bandpass(1300 + r * 0.25, 1.2, sr);
    this.mech.bandpass(2600 + r * 0.3, 2.5, sr);
    this.starterBp.bandpass(this.st.noiseHz + r * 1.5, 0.9, sr);
  }

  process(inputs, outputs, params) {
    const out = outputs[0];
    const ch0 = out[0], ch1 = out[1] || null;
    const rpmT = params.rpm[0], loadT = params.load[0], thrT = params.throttle[0];
    const starter = params.starter[0], running = params.running[0];
    const sr = this.sr;
    for (let i = 0; i < ch0.length; i++) {
      this.rpm += (rpmT - this.rpm) * 0.0015;
      this.load += (loadT - this.load) * 0.002;
      this.thr += (thrT - this.thr) * 0.002;
      if ((this.k++ & 63) === 0) this.updateFilters();
      if ((this.k & 1023) === 0) this.wanderT = (this.rand() * 2 - 1) * 22 * Math.max(0, 1 - this.load * 3);
      this.wander += (this.wanderT - this.wander) * 0.00004;

      const uneven = (1 - running) * Math.max(0, Math.min(1, 1.5 - this.rpm / 800));
      const cyc = Math.sin(2 * Math.PI * 8 * this.phase);
      const rpm = Math.max(0, (this.rpm + (running > 0.99 ? this.wander : 0)) * (1 + this.st.unevenness * uneven * cyc));
      const prev = this.phase;
      this.phase += rpm / 120 / sr;
      if (this.phase >= 1) this.phase -= 1;

      for (let c = 0; c < 8; c++) {
        let o = c / 8 + this.jit[c];
        if (o < 0) o += 1;
        const crossed = prev <= this.phase ? (prev < o && this.phase >= o) : (prev < o || this.phase >= o);
        if (!crossed) continue;
        let a;
        const fired = running > 0.999 || (running > 0 && this.rand() < running);
        if (fired) {
          a = 0.22 + 0.78 * this.load;

          if (this.load < 0.08 && rpm > 1800) { a = 0.13 + 0.05 * this.rand(); if (this.rand() < 0.02 * Math.min(1, (rpm - 1800) / 2000)) this.crackle = 0.7 + this.rand() * 0.6; }

          a *= this.cylGain[c] * (0.94 + 0.12 * this.rand()) * (0.62 + 0.38 * Math.min(1, rpm / 3200));

          if (running < 0.999) a *= this.st.pops + 0.8 * this.rand();
        } else {
          a = this.st.puff * this.cylGain[c] * Math.min(1, rpm / 150);
        }
        this.env[this.bank[c]] += a;
        if (fired) this.tick += 0.5 + 0.5 * this.rand();
      }

      const degS = 1 / (Math.max(rpm, 200) * 6);
      const tauD = Math.min(0.012, Math.max(0.0011, 42 * degS));
      const tauA = Math.min(0.0028, Math.max(0.00022, 10 * degS));
      const dec = Math.exp(-1 / (tauD * sr)), att = 1 - Math.exp(-1 / (tauA * sr));

      const rr = Math.min(1, rpm / 3200);
      const nAmt = 0.12 + 0.4 * this.load * rr * rr;
      let y = 0;
      for (let b = 0; b < 2; b++) {
        const n = this.rand() * 2 - 1;
        this.pres[b] += (this.env[b] - this.pres[b]) * att;
        this.env[b] *= dec;
        const pr = this.pres[b];
        const x = pr * (1 + nAmt * 1.6 * n);
        const r = this.res[b];
        let s = 1.0 * r[0].run(x) + 0.6 * r[1].run(x) + 0.28 * r[2].run(x) + 0.05 * x;
        if (b === 1) {
          this.delay[this.dp] = s;
          s = this.delay[(this.dp + 256 - 55) & 255];
          this.dp = (this.dp + 1) & 255;
        }
        y += s;
      }
      if (this.crackle > 0.01) { y += this.crackle * (this.rand() * 2 - 1) * 0.6; this.crackle *= 0.985; }
      let s = this.lp.run(y) * 0.8 + this.lp2.run(y) * 0.3;

      const nz = this.rand() * 2 - 1;
      s += this.intake.run(nz) * 0.05 * this.thr * Math.min(1, rpm / 2500) * (0.5 + this.env[0] + this.env[1]);

      this.tick *= 0.9965;
      s += this.mech.run(nz) * (0.006 + 0.02 * this.tick * (1 - rr * 0.6)) * Math.min(1, 0.4 + rpm / 3000);

      if (starter > 0.5) {
        const st = this.st, labour = 0.5 - 0.5 * cyc * uneven;
        if ((this.k & 31) === 0) {
          const fm = Math.max(60, this.rpm) * (1 + st.wobble * uneven * cyc) / 60 * 130 * st.pitch;
          this.starterR1.bandpass(fm, st.q1, sr); this.starterR2.bandpass(2 * fm, st.q2, sr);
        }
        const whirr = st.mesh * this.starterR1.run(nz) + st.mesh2 * this.starterR2.run(nz);
        s += (st.level * whirr + st.noise * this.starterBp.run(nz)) * (1 - 0.5 * st.labour + st.labour * labour);
      }

      s = this.hp2.run(this.hp.run(s));
      s = this.shelf.run(Math.tanh(s * 2.0) * 0.6);
      ch0[i] = s;
      if (ch1) ch1[i] = s;
    }
    return true;
  }
}

registerProcessor('engine-v8', EngineProcessor);
