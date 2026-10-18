// Assetto Corsa's data files: INIReader and Curve (.lut), with the game's own parsing rules (ported from
// rustyAC's data/ini.rs and curve.rs). A car's files come from a `files` object: name -> text (see carFiles.js).
//
// The game's quirks are kept on purpose because car files depend on them: a '[' anywhere on a line (even in a
// comment) starts a section; reopening a section empties it; a repeated key keeps its first non-empty value;
// a comment that starts before '=' hides the key; numbers are read like C's wcstod (the longest prefix).

// C wcstod: skip leading white space, read the longest number prefix. { value, consumed }
export function wcstod(text) {
  const m = /^[ \t\n\v\f\r]*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?|[+-]?(?:inf(?:inity)?|nan))/i.exec(text);
  if (!m) return { value: 0, consumed: 0 };
  const s = m[1].toLowerCase();
  const value = s.includes('inf') ? (s[0] === '-' ? -Infinity : Infinity) : s.includes('nan') ? NaN : parseFloat(s);
  return { value, consumed: m[0].length, outOfRange: Number.isFinite(value) && Math.abs(value) > 3.4028235e38 };
}
function wcstol(text) {
  const m = /^[ \t\n\v\f\r]*([+-]?\d+)/.exec(text);
  if (!m) return { value: 0, consumed: 0 };
  return { value: parseInt(m[1], 10), consumed: m[0].length };
}

// the text as the game reads it: up to a Ctrl-Z, CRLF -> LF
export function textMode(text) {
  const z = text.indexOf('\x1a');
  return (z >= 0 ? text.slice(0, z) : text).replace(/\r\n/g, '\n');
}

export class Curve {
  constructor(pairs = []) {
    this.refs = [];
    this.vals = [];
    this.spline = null;
    for (const [r, v] of pairs) this.addValue(r, v);
  }
  static fromLut(text) {
    const c = new Curve();
    if (text != null) c.loadLines(textMode(text));
    return c;
  }
  loadLines(text) {
    for (const line of text.split('\n')) {
      const pieces = line.split('|').filter(p => p.length);
      if (pieces.length !== 2) continue;
      const v = wcstod(pieces[1]), r = wcstod(pieces[0]);
      if (!v.consumed || !r.consumed) throw new Error(`lut line ${JSON.stringify(line)}: invalid number`);
      this.addValue(r.value, v.value);
    }
  }
  addValue(r, v) { this.refs.push(r); this.vals.push(v); this.spline = null; }
  scale(k) { this.vals = this.vals.map(v => v * k); this.spline = null; }
  get count() { return this.vals.length; }
  pairAt(i) { return i >= 0 && i < this.vals.length ? [this.refs[i], this.vals[i]] : [0, 0]; }
  getValue(x) {
    const r = this.refs, v = this.vals;
    if (!r.length) return 0;
    if (!(x > r[0])) return v[0];
    let i = 1;
    for (;;) {
      if (i === r.length) return v[v.length - 1];
      if (!(x > r[i])) break;
      i++;
    }
    return (v[i] - v[i - 1]) * (x - r[i - 1]) / (r[i] - r[i - 1]) + v[i - 1];
  }
  getCubicSplineValue(x) {
    if (!this.spline) this.spline = splineCoefficients(this.refs, this.vals);
    const e = this.spline;
    if (!e.length) return 0;
    let first = 0, count = e.length;
    while (count > 0) {
      const half = count >> 1;
      if (x > e[first + half].x) { first += half + 1; count -= half + 1; } else count = half;
    }
    if (first) first--;
    const el = e[first], dx = x - el.x, sq = dx * dx;
    return ((dx * el.b + el.a) + sq * el.c) + (sq * dx) * el.d;
  }
}

function splineCoefficients(x, y) {
  if (x.length !== y.length || x.length < 3) return [];
  const size = y.length, n = size - 1;
  const b = new Array(n).fill(0), d = new Array(n).fill(0), a = new Array(n).fill(0);
  const c = new Array(size).fill(0), l = new Array(size).fill(0), u = new Array(size).fill(0), z = new Array(size).fill(0), h = new Array(size).fill(0);
  l[0] = 1; h[0] = x[1] - x[0];
  for (let i = 1; i < n; i++) {
    h[i] = x[i + 1] - x[i];
    l[i] = (x[i + 1] - x[i - 1]) * 2 - h[i - 1] * u[i - 1];
    u[i] = h[i] / l[i];
    a[i] = (y[i + 1] - y[i]) * (3 / h[i]) - (y[i] - y[i - 1]) * (3 / h[i - 1]);
    z[i] = (a[i] - h[i - 1] * z[i - 1]) / l[i];
  }
  l[n] = 1;
  for (let j = n - 1; j >= 0; j--) {
    c[j] = z[j] - u[j] * c[j + 1];
    b[j] = (y[j + 1] - y[j]) / h[j] - ((c[j] * 2 + c[j + 1]) * h[j]) * (1 / 3);
    d[j] = (c[j + 1] - c[j]) / (h[j] * 3);
  }
  const out = [];
  for (let i = 0; i < n; i++) out.push({ x: x[i], a: y[i], b: b[i], c: c[i], d: d[i] });
  return out;
}

// file names are case-insensitive (the game runs on Windows); `files` keys are lower case
export const fileText = (files, name) => files[String(name).toLowerCase().replace(/\\/g, '/')] ?? null;

export class IniReader {
  // files: { name: text } of the car's data folder; name: this file's name in it
  constructor(files, name) {
    this.files = files;
    this.name = name;
    this.sections = new Map();
    const text = fileText(files, name);
    this.ready = text != null;
    if (this.ready) this.parse(textMode(text));
  }
  parse(code) {
    let section = '';
    for (const line of code.split('\n')) {
      const open = line.indexOf('['), close = line.indexOf(']');
      if (open >= 0 && close >= 0) {
        section = close > open ? line.slice(open + 1, close) : line.slice(open + 1);
        this.sections.set(section, new Map());
        continue;
      }
      const eq = line.indexOf('=');
      if (eq < 0) continue;
      const comment = line.indexOf(';');
      if (!section || (comment >= 0 && comment <= eq)) continue;
      const key = line.slice(0, eq);
      const value = (comment >= 0 ? line.slice(eq + 1, comment) : line.slice(eq + 1)).replace(/[ \t\r]+$/, '');
      if (!this.sections.has(section)) this.sections.set(section, new Map());
      const keys = this.sections.get(section);
      if (!keys.get(key)) keys.set(key, value);
    }
  }
  hasSection(s) { return this.sections.has(s); }
  hasKey(s, k) { return this.sections.has(s) && this.sections.get(s).has(k); }
  keys(s) { return this.sections.has(s) ? [...this.sections.get(s).keys()] : []; }
  getString(s, k) { return (this.sections.get(s) && this.sections.get(s).get(k)) || ''; }
  #err(s, k, text, what) { return new Error(`${this.name}: [${s}] ${k}=${text}: ${what}`); }
  getFloat(s, k) {
    const text = this.getString(s, k);
    if (!text || (text.length === 1 && ' \t\n\v\f\r'.includes(text))) return 0;
    const p = wcstod(text);
    if (!p.consumed) throw this.#err(s, k, text, 'invalid stof argument');
    return p.value;
  }
  getInt(s, k) {
    const text = this.getString(s, k);
    if (!text) return 0;
    const p = wcstol(text);
    if (!p.consumed) throw this.#err(s, k, text, 'invalid stoi argument');
    return p.value;
  }
  getFloat3(s, k) {
    const text = this.getString(s, k);
    if (!text) return [0, 0, 0];
    const out = [0, 0, 0];
    let rest = text;
    for (let i = 0; i < 3; i++) {
      const delim = i < 2 ? /[,]/ : /[\n\r]/;
      rest = rest.replace(i < 2 ? /^,+/ : /^[\n\r]+/, '');
      if (!rest) return [0, 0, 0];
      const m = delim.exec(rest);
      const token = m ? rest.slice(0, m.index) : rest;
      rest = m ? rest.slice(m.index + 1) : '';
      const p = wcstod(token);
      if (!p.consumed) throw this.#err(s, k, text, 'invalid stof argument');
      out[i] = p.value;
    }
    return out;
  }
  getFloat4(s, k) {
    const text = this.getString(s, k), out = [0, 0, 0, 0];
    text.split(',').filter(t => t.length).slice(0, 4).forEach((t, i) => {
      const p = wcstod(t);
      if (!p.consumed) throw this.#err(s, k, text, 'invalid stof argument');
      out[i] = p.value;
    });
    return out;
  }
  // an inline curve "(0=1|10=3)" or the name of a .lut next to this file
  getCurve(s, k) {
    const text = this.getString(s, k);
    const open = text.indexOf('('), close = text.indexOf(')');
    if (open < 0 || close < 0) return Curve.fromLut(fileText(this.files, text));
    const inner = close > open ? text.slice(open + 1, close) : text.slice(open + 1);
    const c = new Curve();
    for (const pair of inner.split('|').filter(p => p.length)) {
      const parts = pair.split('=').filter(p => p.length);
      if (parts.length !== 2) throw this.#err(s, k, text, `is not a valid curve, at value:${pair}`);
      const v = wcstod(parts[1]), r = wcstod(parts[0]);
      if (!v.consumed || !r.consumed) throw this.#err(s, k, text, 'invalid stof argument');
      c.addValue(r.value, v.value);
    }
    return c;
  }
}
