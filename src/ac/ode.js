// The part of ODE 0.13.1 (Open Dynamics Engine) that Assetto Corsa's car uses: bodies, the four joint types of
// a car (DBall rods, Ball, Slider, Fixed), contact joints, and dWorldStep (the "big matrix" stepper with
// ODE's Dantzig LCP solver). Ported from rustyAC's crates/rustyac-ode (a 1:1 transcription of acs.exe's ODE),
// in double precision. ODE is copyright (c) 2001-2007 Russell L. Smith, BSD-3-Clause.
//
// Conventions as in ODE: a matrix is 3x4 row-major (R[4 * row + col]), world = R * local; a quaternion is
// (w, x, y, z). Objects sit in linked lists, newest first, because that order decides the row order of the
// constraint matrix.

const INF = Infinity;

export const BODY_FINITE_ROTATION = 1, BODY_FINITE_ROTATION_AXIS = 2, BODY_DISABLED = 4, BODY_NO_GRAVITY = 8;
export const BODY_GYROSCOPIC = 256;
const JOINT_INGROUP = 1, JOINT_REVERSE = 2, JOINT_DISABLED = 8;
export const PARAM_CFM = 8, PARAM_ERP = 0xd;

export const CONTACT_MU2 = 0x001, CONTACT_FDIR1 = 0x002, CONTACT_BOUNCE = 0x004, CONTACT_SOFT_ERP = 0x008;
export const CONTACT_SOFT_CFM = 0x010, CONTACT_MOTION1 = 0x020, CONTACT_MOTION2 = 0x040, CONTACT_MOTION_N = 0x080;
export const CONTACT_SLIP1 = 0x100, CONTACT_SLIP2 = 0x200, CONTACT_APPROX1_1 = 0x1000, CONTACT_APPROX1_2 = 0x2000;

const pad = a => (a > 1 ? ((a - 1) | 3) + 1 : a);

// --- small maths (odemath.cpp, rotation.cpp) ---------------------------------------------------------------

export const dot3 = (a, b, ia = 0, ib = 0) => a[ia] * b[ib] + a[ia + 1] * b[ib + 1] + a[ia + 2] * b[ib + 2];
export const cross3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const length3 = a => Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
// R * v
export const mul0_331 = (R, v) => [
  R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
  R[4] * v[0] + R[5] * v[1] + R[6] * v[2],
  R[8] * v[0] + R[9] * v[1] + R[10] * v[2],
];
// R^T * v
export const mul1_331 = (R, v) => [
  R[0] * v[0] + R[4] * v[1] + R[8] * v[2],
  R[1] * v[0] + R[5] * v[1] + R[9] * v[2],
  R[2] * v[0] + R[6] * v[1] + R[10] * v[2],
];
// res = A * B
function mul0_333(res, A, B) {
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++)
    res[4 * r + c] = A[4 * r] * B[c] + A[4 * r + 1] * B[4 + c] + A[4 * r + 2] * B[8 + c];
}
// res = A * B^T
function mul2_333(res, A, B) {
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++)
    res[4 * r + c] = A[4 * r] * B[4 * c] + A[4 * r + 1] * B[4 * c + 1] + A[4 * r + 2] * B[4 * c + 2];
}
function setCrossPlus(res, o, a, skip) {
  res[o + 1] = -a[2]; res[o + 2] = a[1];
  res[o + skip] = a[2]; res[o + skip + 2] = -a[0];
  res[o + 2 * skip] = -a[1]; res[o + 2 * skip + 1] = a[0];
}
function setCrossMinus(res, o, a, skip) {
  res[o + 1] = a[2]; res[o + 2] = -a[1];
  res[o + skip] = -a[2]; res[o + skip + 2] = a[0];
  res[o + 2 * skip] = a[1]; res[o + 2 * skip + 1] = -a[0];
}
function invertMatrix3(dst, m) {
  const c0 = m[5] * m[10] - m[6] * m[9], c1 = m[4] * m[10] - m[8] * m[6], c2 = m[4] * m[9] - m[8] * m[5];
  const det = (m[0] * c0 - m[1] * c1) + m[2] * c2;
  if (det === 0 || Number.isNaN(det)) return 0;
  const r = 1 / det;
  dst[0] = c0 * r;
  dst[1] = (m[9] * m[2] - m[1] * m[10]) * r;
  dst[2] = (m[1] * m[6] - m[5] * m[2]) * r;
  dst[4] = (m[6] * m[8] - m[4] * m[10]) * r;
  dst[5] = (m[0] * m[10] - m[8] * m[2]) * r;
  dst[6] = (m[4] * m[2] - m[0] * m[6]) * r;
  dst[8] = (m[4] * m[9] - m[8] * m[5]) * r;
  dst[9] = (m[8] * m[1] - m[0] * m[9]) * r;
  dst[10] = (m[0] * m[5] - m[1] * m[4]) * r;
  return det;
}
export function safeNormalize3(a) {
  const x = Math.abs(a[0]), y = Math.abs(a[1]), z = Math.abs(a[2]);
  let m;
  if (y > x) m = z > y ? z : y;
  else if (z > x) m = z;
  else { if (!(x > 0)) { a[0] = 1; a[1] = 0; a[2] = 0; return false; } m = x; }
  const r = 1 / m, t0 = r * a[0], t1 = r * a[1], t2 = r * a[2];
  const l = 1 / Math.sqrt(t0 * t0 + t1 * t1 + t2 * t2);
  a[0] = t0 * l; a[1] = t1 * l; a[2] = t2 * l;
  return true;
}
function safeNormalize4(a) {
  const l = a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3];
  if (l > 0) { const k = 1 / Math.sqrt(l); a[0] *= k; a[1] *= k; a[2] *= k; a[3] *= k; return true; }
  a[0] = 1; a[1] = 0; a[2] = 0; a[3] = 0;
  return false;
}
export function planeSpace(n, p, q) {
  if (Math.abs(n[2]) > Math.SQRT1_2) {
    const a = n[1] * n[1] + n[2] * n[2], k = 1 / Math.sqrt(a);
    p[0] = 0; p[1] = -n[2] * k; p[2] = n[1] * k;
    q[0] = a * k; q[1] = -n[0] * p[2]; q[2] = n[0] * p[1];
  } else {
    const a = n[0] * n[0] + n[1] * n[1], k = 1 / Math.sqrt(a);
    p[0] = -n[1] * k; p[1] = n[0] * k; p[2] = 0;
    q[0] = -n[2] * p[1]; q[1] = n[2] * p[0]; q[2] = a * k;
  }
}
function orthogonalizeR(m) {
  const n0 = m[0] * m[0] + m[1] * m[1] + m[2] * m[2];
  if (n0 !== 1) { const r = [m[0], m[1], m[2]]; safeNormalize3(r); m[0] = r[0]; m[1] = r[1]; m[2] = r[2]; }
  const proj = m[0] * m[4] + m[1] * m[5] + m[2] * m[6];
  if (proj !== 0) { m[4] -= proj * m[0]; m[5] -= proj * m[1]; m[6] -= proj * m[2]; }
  const n1 = m[4] * m[4] + m[5] * m[5] + m[6] * m[6];
  if (n1 !== 1) { const r = [m[4], m[5], m[6]]; safeNormalize3(r); m[4] = r[0]; m[5] = r[1]; m[6] = r[2]; }
  const c = cross3([m[0], m[1], m[2]], [m[4], m[5], m[6]]);
  m[8] = c[0]; m[9] = c[1]; m[10] = c[2];
  m[3] = 0; m[7] = 0; m[11] = 0;
}
export const qMultiply0 = (b, c) => [
  b[0] * c[0] - b[1] * c[1] - b[2] * c[2] - b[3] * c[3],
  b[0] * c[1] + b[1] * c[0] + b[2] * c[3] - b[3] * c[2],
  b[0] * c[2] + b[2] * c[0] + b[3] * c[1] - b[1] * c[3],
  b[0] * c[3] + b[3] * c[0] + b[1] * c[2] - b[2] * c[1],
];
const qMultiply1 = (b, c) => [
  b[0] * c[0] + b[1] * c[1] + b[2] * c[2] + b[3] * c[3],
  b[0] * c[1] - b[1] * c[0] - b[2] * c[3] + b[3] * c[2],
  b[0] * c[2] - b[2] * c[0] - b[3] * c[1] + b[1] * c[3],
  b[0] * c[3] - b[3] * c[0] - b[1] * c[2] + b[2] * c[1],
];
const qMultiply2 = (b, c) => [
  b[0] * c[0] + b[1] * c[1] + b[2] * c[2] + b[3] * c[3],
  -b[0] * c[1] + b[1] * c[0] - b[2] * c[3] + b[3] * c[2],
  -b[0] * c[2] + b[2] * c[0] - b[3] * c[1] + b[1] * c[3],
  -b[0] * c[3] + b[3] * c[0] - b[1] * c[2] + b[2] * c[1],
];
const qMultiply3 = (b, c) => [
  b[0] * c[0] - b[1] * c[1] - b[2] * c[2] - b[3] * c[3],
  (b[2] * c[3] - (b[1] * c[0] + b[0] * c[1])) - b[3] * c[2],
  (b[3] * c[1] - (b[2] * c[0] + b[0] * c[2])) - b[1] * c[3],
  (b[1] * c[2] - (b[3] * c[0] + b[0] * c[3])) - b[2] * c[1],
];
export function rFromQ(r, q) {
  const qq1 = 2 * q[1] * q[1], qq2 = 2 * q[2] * q[2], qq3 = 2 * q[3] * q[3];
  r[0] = 1 - qq2 - qq3; r[1] = 2 * (q[1] * q[2] - q[0] * q[3]); r[2] = 2 * (q[1] * q[3] + q[0] * q[2]); r[3] = 0;
  r[4] = 2 * (q[1] * q[2] + q[0] * q[3]); r[5] = 1 - qq1 - qq3; r[6] = 2 * (q[2] * q[3] - q[0] * q[1]); r[7] = 0;
  r[8] = 2 * (q[1] * q[3] - q[0] * q[2]); r[9] = 2 * (q[2] * q[3] + q[0] * q[1]); r[10] = 1 - qq1 - qq2; r[11] = 0;
}
export function qFromR(r) {
  const m = (i, j) => r[i * 4 + j];
  const q = [0, 0, 0, 0];
  const tr = m(0, 0) + m(1, 1) + m(2, 2);
  if (tr >= 0) {
    let s = Math.sqrt(tr + 1);
    q[0] = 0.5 * s; s = 0.5 / s;
    q[1] = (m(2, 1) - m(1, 2)) * s; q[2] = (m(0, 2) - m(2, 0)) * s; q[3] = (m(1, 0) - m(0, 1)) * s;
    return q;
  }
  const c = m(1, 1) > m(0, 0) ? (m(2, 2) > m(1, 1) ? 2 : 1) : (m(2, 2) > m(0, 0) ? 2 : 0);
  if (c === 0) {
    let s = Math.sqrt((m(0, 0) - (m(1, 1) + m(2, 2))) + 1);
    q[1] = 0.5 * s; s = 0.5 / s;
    q[2] = (m(0, 1) + m(1, 0)) * s; q[3] = (m(2, 0) + m(0, 2)) * s; q[0] = (m(2, 1) - m(1, 2)) * s;
  } else if (c === 1) {
    let s = Math.sqrt((m(1, 1) - (m(2, 2) + m(0, 0))) + 1);
    q[2] = 0.5 * s; s = 0.5 / s;
    q[3] = (m(1, 2) + m(2, 1)) * s; q[1] = (m(0, 1) + m(1, 0)) * s; q[0] = (m(0, 2) - m(2, 0)) * s;
  } else {
    let s = Math.sqrt((m(2, 2) - (m(0, 0) + m(1, 1))) + 1);
    q[3] = 0.5 * s; s = 0.5 / s;
    q[1] = (m(2, 0) + m(0, 2)) * s; q[2] = (m(1, 2) + m(2, 1)) * s; q[0] = (m(1, 0) - m(0, 1)) * s;
  }
  return q;
}
const dqFromW = (w, q) => [
  0.5 * (-w[0] * q[1] - w[1] * q[2] - w[2] * q[3]),
  0.5 * (w[0] * q[0] + w[1] * q[3] - w[2] * q[2]),
  0.5 * (-w[0] * q[3] + w[1] * q[0] + w[2] * q[1]),
  0.5 * (w[0] * q[2] - w[1] * q[1] + w[2] * q[0]),
];

// --- matrix.cpp: L*D*L^T factor and solves -----------------------------------------------------------------

function factorLDLT(A, d, n, nskip) {
  for (let i = 0; i < n; i++) {
    // solve L * x = row i (below the diagonal), then scale by D
    const ri = i * nskip;
    for (let j = 0; j < i; j++) {
      let s = A[ri + j];
      const rj = j * nskip;
      for (let k = 0; k < j; k++) s -= A[rj + k] * A[ri + k];
      A[ri + j] = s;
    }
    let z = A[ri + i];
    for (let k = 0; k < i; k++) {
      const p = A[ri + k], q = p * d[k];
      A[ri + k] = q;
      z -= p * q;
    }
    d[i] = 1 / z;
  }
}
function solveL1(L, b, n, nskip) {
  for (let i = 0; i < n; i++) {
    let s = b[i];
    const r = i * nskip;
    for (let k = 0; k < i; k++) s -= L[r + k] * b[k];
    b[i] = s;
  }
}
function solveL1T(L, b, n, nskip) {
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i];
    for (let k = i + 1; k < n; k++) s -= L[k * nskip + i] * b[k];
    b[i] = s;
  }
}
function solveLDLT(L, d, b, n, nskip) {
  solveL1(L, b, n, nskip);
  for (let i = 0; i < n; i++) b[i] *= d[i];
  solveL1T(L, b, n, nskip);
}
function dotN(a, ia, b, ib, n) {
  let s = 0;
  for (let k = 0; k < n; k++) s += a[ia + k] * b[ib + k];
  return s;
}
// inverse of a symmetric positive definite 3x3 (Cholesky); false if it is not positive definite
function invertPD3(A, out) {
  const n = 3, s = 4, L = A.slice(0, 12), recip = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < i; j++) {
      let sum = L[i * s + j];
      for (let k = 0; k < j; k++) sum -= L[i * s + k] * L[j * s + k];
      L[i * s + j] = sum * recip[j];
    }
    let sum = L[i * s + i];
    for (let k = 0; k < i; k++) sum -= L[i * s + k] * L[i * s + k];
    if (!(sum > 0)) return false;
    L[i * s + i] = Math.sqrt(sum);
    recip[i] = 1 / L[i * s + i];
  }
  out.fill(0);
  for (let c = 0; c < n; c++) {
    const y = [0, 0, 0], b = [0, 0, 0];
    b[c] = 1;
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let k = 0; k < i; k++) sum += L[i * s + k] * y[k];
      y[i] = (b[i] - sum) / L[i * s + i];
    }
    for (let i = n - 1; i >= 0; i--) {
      let sum = 0;
      for (let k = i + 1; k < n; k++) sum += L[k * s + i] * b[k];
      b[i] = (y[i] - sum) / L[i * s + i];
    }
    for (let j = 0; j < n; j++) out[j * s + c] = b[j];
  }
  return true;
}

// ldltAddTL / ldltRemove (matrix.cpp), used by the LCP solver to drop a row from the factor
function ldltAddTL(L, lo, d, dofs, a, n, nskip, w1, w2) {
  if (n < 2) return;
  w1[0] = 0; w2[0] = 0;
  for (let j = 1; j < n; j++) { const v = a[j] * Math.SQRT1_2; w1[j] = v; w2[j] = v; }
  const W11 = (0.5 * a[0] + 1) * Math.SQRT1_2, W21 = (0.5 * a[0] - 1) * Math.SQRT1_2;
  let alpha1 = 1, alpha2 = 1;
  {
    let dee = d[dofs];
    let alphanew = alpha1 + W11 * W11 * dee;
    dee /= alphanew;
    const gamma1 = W11 * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - W21 * W21 * dee;
    dee /= alphanew;
    alpha2 = alphanew;
    const k1 = 1 - W21 * gamma1, k2 = W21 * gamma1 * W11 - W21;
    let ll = lo + nskip;
    for (let p = 1; p < n; p++) {
      const wp = w1[p], ell = L[ll];
      w1[p] = wp - W11 * ell;
      w2[p] = k1 * wp + k2 * ell;
      ll += nskip;
    }
  }
  let ll = lo + nskip + 1;
  for (let j = 1; j < n; j++) {
    const k1 = w1[j], k2 = w2[j];
    let dee = d[dofs + j];
    let alphanew = alpha1 + k1 * k1 * dee;
    dee /= alphanew;
    const gamma1 = k1 * dee;
    dee *= alpha1;
    alpha1 = alphanew;
    alphanew = alpha2 - k2 * k2 * dee;
    dee /= alphanew;
    const gamma2 = k2 * dee;
    dee *= alpha2;
    d[dofs + j] = dee;
    alpha2 = alphanew;
    let lp = ll + nskip;
    for (let p = j + 1; p < n; p++) {
      let ell = L[lp];
      let wp = w1[p] - k1 * ell;
      ell += gamma1 * wp;
      w1[p] = wp;
      wp = w2[p] - k2 * ell;
      ell -= gamma2 * wp;
      w2[p] = wp;
      L[lp] = ell;
      lp += nskip;
    }
    ll += nskip + 1;
  }
}
function removeRowCol(A, n, nskip, r) {
  if (r >= n - 1) return;
  if (r > 0) {
    for (let i = 0; i < r; i++) for (let j = r; j < n - 1; j++) A[i * nskip + j] = A[i * nskip + j + 1];
    for (let i = r; i < n - 1; i++) for (let j = 0; j < r; j++) A[i * nskip + j] = A[(i + 1) * nskip + j];
  }
  for (let i = r; i < n - 1; i++) for (let j = r; j < n - 1; j++) A[i * nskip + j] = A[(i + 1) * nskip + j + 1];
}
function ldltRemove(A, arows, p, L, d, n2, r, nskip) {
  if (r === n2 - 1) return;
  const geta = (i, j) => (i > j ? A[arows[i] * nskip + j] : A[arows[j] * nskip + i]);
  const w1 = new Float64Array(nskip), w2 = new Float64Array(nskip), aa = new Float64Array(n2);
  if (r === 0) {
    const p0 = p[0];
    for (let i = 0; i < n2; i++) aa[i] = -geta(p[i], p0);
    aa[0] += 1;
    ldltAddTL(L, 0, d, 0, aa, n2, nskip, w1, w2);
  } else {
    const t = new Float64Array(r);
    for (let i = 0; i < r; i++) t[i] = L[r * nskip + i] / d[i];
    const pr = p[r];
    for (let i = 0; i < n2 - r; i++) aa[i] = dotN(L, (r + i) * nskip, t, 0, r) - geta(p[r + i], pr);
    aa[0] += 1;
    ldltAddTL(L, r * nskip + r, d, r, aa, n2 - r, nskip, w1, w2);
  }
  removeRowCol(L, n2, nskip, r);
  for (let i = r; i < n2 - 1; i++) d[i] = d[i + 1];
}

// --- lcp.cpp: ODE's Dantzig LCP solver (dSolveLCP with friction indices) ------------------------------------

function solveLCP(n, A, x, b, nub, lo, hi, findex) {
  const nskip = pad(n);
  const stats = { pivots: 0, error: false };
  if (nub >= n) {
    const d = new Float64Array(n);
    factorLDLT(A, d, n, nskip);
    solveLDLT(A, d, b, n, nskip);
    for (let i = 0; i < n; i++) x[i] = b[i];
    return stats;
  }
  const L = new Float64Array(n * nskip), d = new Float64Array(n), w = new Float64Array(n);
  const deltaW = new Float64Array(n), deltaX = new Float64Array(n), dell = new Float64Array(n), ell = new Float64Array(n);
  const arows = new Int32Array(n), P = new Int32Array(n), C = new Int32Array(n), state = new Uint8Array(n);
  let nC = 0, nN = 0;
  const at = (row, col) => A[arows[row] * nskip + col];

  const swapRowsAndCols = (i1, i2, fast) => {
    const r1 = arows[i1] * nskip, r2 = arows[i2] * nskip;
    for (let i = i1 + 1; i < i2; i++) {
      const ri = arows[i] * nskip + i1;
      A[r1 + i] = A[ri];
      A[ri] = A[r2 + i];
    }
    A[r1 + i2] = A[r1 + i1];
    A[r1 + i1] = A[r2 + i1];
    A[r2 + i1] = A[r2 + i2];
    if (fast) { const t = arows[i1]; arows[i1] = arows[i2]; arows[i2] = t; }
    else for (let k = 0; k <= i2; k++) { const t = A[r1 + k]; A[r1 + k] = A[r2 + k]; A[r2 + k] = t; }
    for (let j = i2 + 1; j < n; j++) {
      const rj = arows[j] * nskip, t = A[rj + i1];
      A[rj + i1] = A[rj + i2]; A[rj + i2] = t;
    }
  };
  const sw = (arr, i, j) => { const t = arr[i]; arr[i] = arr[j]; arr[j] = t; };
  const swapProblem = (i1, i2, fast) => {
    if (i1 === i2) return;
    swapRowsAndCols(i1, i2, fast);
    sw(x, i1, i2); sw(b, i1, i2); sw(w, i1, i2); sw(lo, i1, i2); sw(hi, i1, i2); sw(P, i1, i2); sw(state, i1, i2);
    if (findex) sw(findex, i1, i2);
  };
  const loadDell = i => {
    const start = arows[i] * nskip;
    for (let j = 0; j < nub; j++) dell[j] = A[start + j];
    for (let j = nub; j < nC; j++) dell[j] = A[start + C[j]];
  };
  const transferIToC = i => {
    if (nC > 0) {
      const lt = nC * nskip;
      for (let j = 0; j < nC; j++) L[lt + j] = ell[j];
      d[nC] = 1 / (at(i, i) - dotN(ell, 0, dell, 0, nC));
    } else d[0] = 1 / at(i, i);
    swapProblem(nC, i, true);
    C[nC] = nC;
    nC++;
  };
  const transferIFromNToC = i => {
    if (nC > 0) {
      loadDell(i);
      solveL1(L, dell, nC, nskip);
      const lt = nC * nskip;
      for (let j = 0; j < nC; j++) { const v = dell[j] * d[j]; ell[j] = v; L[lt + j] = v; }
      d[nC] = 1 / (at(i, i) - dotN(ell, 0, dell, 0, nC));
    } else d[0] = 1 / at(i, i);
    swapProblem(nC, i, true);
    C[nC] = nC;
    nN--;
    nC++;
  };
  const transferIFromCToN = i => {
    let lastIdx = -1;
    for (let j = 0; j < nC; j++) {
      if (C[j] === nC - 1) lastIdx = j;
      if (C[j] === i) {
        ldltRemove(A, arows, C, L, d, nC, j, nskip);
        let k = lastIdx;
        if (k < 0) { k = j + 1; while (k < nC && C[k] !== nC - 1) k++; }
        C[k] = C[j];
        for (let m = j; m < nC - 1; m++) C[m] = C[m + 1];
        break;
      }
    }
    swapProblem(i, nC - 1, true);
    nN++;
    nC--;
  };
  const solve1 = (a, tmp, i, dir, onlyTransfer) => {
    if (nC > 0) {
      loadDell(i);
      solveL1(L, dell, nC, nskip);
      for (let j = 0; j < nC; j++) ell[j] = dell[j] * d[j];
      if (!onlyTransfer) {
        for (let j = 0; j < nC; j++) tmp[j] = ell[j];
        solveL1T(L, tmp, nC, nskip);
        if (dir > 0) for (let j = 0; j < nC; j++) a[C[j]] = -tmp[j];
        else for (let j = 0; j < nC; j++) a[C[j]] = tmp[j];
      }
    }
  };

  // init (dLCP::dLCP)
  x.fill(0, 0, n);
  for (let k = 0; k < n; k++) { arows[k] = k; P[k] = k; }
  for (let k = nub; k < n; k++) {
    if (findex && findex[k] >= 0) continue;
    if ((lo[k] === -INF || Number.isNaN(lo[k])) && (hi[k] === INF || Number.isNaN(hi[k]))) { swapProblem(nub, k, false); nub++; }
  }
  if (nub > 0) {
    for (let j = 0; j < nub; j++) { const src = arows[j] * nskip; for (let k = 0; k <= j; k++) L[j * nskip + k] = A[src + k]; }
    factorLDLT(L, d, nub, nskip);
    for (let j = 0; j < nub; j++) x[j] = b[j];
    solveLDLT(L, d, x, nub, nskip);
    w.fill(0, 0, nub);
    for (let k = 0; k < nub; k++) C[k] = k;
    nC = nub;
  }
  if (findex) {
    let numAtEnd = 0;
    for (let k = n - 1; k >= nub; k--) if (findex[k] >= 0) { swapProblem(k, n - 1 - numAtEnd, true); numAtEnd++; }
  }
  const adjNub = nub;

  let hitFirstFriction = false;
  for (let i = adjNub; i < n; i++) {
    let sError = false;
    if (!hitFirstFriction && findex && findex[i] >= 0) {
      for (let j = 0; j < n; j++) deltaW[P[j]] = x[j];
      for (let k = i; k < n; k++) {
        const wfk = deltaW[findex[k]];
        if (wfk === 0 || Number.isNaN(wfk)) { hi[k] = 0; lo[k] = 0; }
        else { const h = Math.abs(wfk * hi[k]); hi[k] = h; lo[k] = -h; }
      }
      hitFirstFriction = true;
    }
    w[i] = dotN(A, arows[i] * nskip, x, 0, nC) + dotN(A, arows[i] * nskip + nC, x, nC, nN) - b[i];

    if ((lo[i] === 0 || Number.isNaN(lo[i])) && w[i] >= 0) { nN++; state[i] = 0; }
    else if ((hi[i] === 0 || Number.isNaN(hi[i])) && !(w[i] > 0)) { nN++; state[i] = 1; }
    else if (w[i] === 0 || Number.isNaN(w[i])) { solve1(deltaX, deltaW, i, 0, true); transferIToC(i); }
    else {
      for (;;) {
        stats.pivots++;
        const dir = w[i] <= 0 ? 1 : -1, dirf = dir;
        solve1(deltaX, deltaW, i, dir, false);
        for (let k = 0; k < nN; k++) deltaW[nC + k] = dotN(A, arows[nC + k] * nskip, deltaX, 0, nC);
        {
          const start = arows[i] * nskip + nC;
          if (dir > 0) for (let j = 0; j < nN; j++) deltaW[nC + j] += A[start + j];
          else for (let j = 0; j < nN; j++) deltaW[nC + j] -= A[start + j];
        }
        deltaW[i] = dirf * at(i, i) + dotN(A, arows[i] * nskip, deltaX, 0, nC);

        let cmd = 1, si = 0;
        let s = (-1 / deltaW[i]) * w[i];
        if (dir > 0) {
          if (!(hi[i] >= INF)) { const s2 = (hi[i] - x[i]) * dirf; if (!(s2 >= s)) { s = s2; cmd = 3; } }
        } else if (lo[i] > -INF) { const s2 = (lo[i] - x[i]) * dirf; if (!(s2 >= s)) { s = s2; cmd = 2; } }
        for (let k = 0; k < nN; k++) {
          const ik = k + nC, dw = deltaW[ik];
          const wants = !state[ik] ? !(dw >= 0) : dw > 0;
          if (wants) {
            if ((lo[ik] === 0 || Number.isNaN(lo[ik])) && (hi[ik] === 0 || Number.isNaN(hi[ik]))) continue;
            const s2 = (-1 / dw) * w[ik];
            if (!(s2 >= s)) { s = s2; cmd = 4; si = ik; }
          }
        }
        for (let k = adjNub; k < nC; k++) {
          const dx = deltaX[k];
          if (!(dx >= 0) && lo[k] > -INF) { const s2 = (lo[k] - x[k]) / dx; if (!(s2 >= s)) { s = s2; cmd = 5; si = k; } }
          if (dx > 0 && !(hi[k] >= INF)) { const s2 = (hi[k] - x[k]) / dx; if (!(s2 >= s)) { s = s2; cmd = 6; si = k; } }
        }
        if (!(s > 0)) {
          for (let k = i; k < n; k++) { x[k] = 0; w[k] = 0; }
          sError = true;
          break;
        }
        for (let k = 0; k < nC; k++) x[k] += s * deltaX[k];
        x[i] += s * dirf;
        for (let k = 0; k < nN; k++) w[nC + k] += s * deltaW[nC + k];
        w[i] += s * deltaW[i];
        if (cmd === 1) { w[i] = 0; transferIToC(i); }
        else if (cmd === 2) { x[i] = lo[i]; state[i] = 0; nN++; }
        else if (cmd === 3) { x[i] = hi[i]; state[i] = 1; nN++; }
        else if (cmd === 4) { w[si] = 0; transferIFromNToC(si); }
        else if (cmd === 5) { x[si] = lo[si]; state[si] = 0; transferIFromCToN(si); }
        else { x[si] = hi[si]; state[si] = 1; transferIFromCToN(si); }
        if (cmd <= 3) break;
      }
    }
    if (sError) { stats.error = true; break; }
  }
  for (let j = 0; j < n; j++) deltaW[j] = x[j];
  for (let j = 0; j < n; j++) x[P[j]] = deltaW[j];
  return stats;
}

// --- bodies and joints -------------------------------------------------------------------------------------

class Body {
  constructor(id, next) {
    this.id = id;
    this.next = next;
    this.tag = 0;
    this.firstJoint = null; // { joint, node }
    this.flags = BODY_GYROSCOPIC;
    this.mass = { mass: 1, c: [0, 0, 0], I: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0] };
    this.invI = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
    this.invMass = 1;
    this.pos = [0, 0, 0];
    this.R = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
    this.q = [1, 0, 0, 0];
    this.lvel = [0, 0, 0];
    this.avel = [0, 0, 0];
    this.facc = [0, 0, 0];
    this.tacc = [0, 0, 0];
    this.finiteRotAxis = [0, 0, 0];
  }
}

const relPointPos = (b, p) => { const r = mul0_331(b.R, p); return [r[0] + b.pos[0], r[1] + b.pos[1], r[2] + b.pos[2]]; };
const posRelPoint = (b, p) => mul1_331(b.R, [p[0] - b.pos[0], p[1] - b.pos[1], p[2] - b.pos[2]]);
const pointVel = (b, p) => {
  const c = cross3(b.avel, [p[0] - b.pos[0], p[1] - b.pos[1], p[2] - b.pos[2]]);
  return [c[0] + b.lvel[0], c[1] + b.lvel[1], c[2] + b.lvel[2]];
};

// one joint: kind is 'ball' | 'dball' | 'fixed' | 'slider' | 'contact'
class Joint {
  constructor(id, next, kind, world) {
    this.id = id;
    this.next = next;
    this.tag = 0;
    this.flags = 0;
    this.node = [{ body: null, next: null }, { body: null, next: null }];
    this.feedback = null;
    this.kind = kind;
    this.alive = true;
    this.erp = world.erp;
    this.cfm = world.cfm;
    this.anchor1 = [0, 0, 0];
    this.anchor2 = [0, 0, 0];
    this.targetDistance = 0;
    this.qrel = [0, 0, 0, 0];
    this.offset = [0, 0, 0];
    this.axis1 = [1, 0, 0];
    this.contact = null;
    this.theM = 0;
  }
  isEnabled() {
    return (this.flags & JOINT_DISABLED) === 0 &&
      ((this.node[0].body && !(0 >= this.node[0].body.invMass)) || (this.node[1].body && !(0 >= this.node[1].body.invMass)));
  }
  info1() {
    switch (this.kind) {
      case 'ball': return [3, 3];
      case 'fixed': return [6, 6];
      case 'dball': return [1, 1];
      case 'slider': return [5, 5];
      case 'contact': return contactInfo1(this);
    }
    return [0, 0];
  }
}

function contactInfo1(j) {
  const s = j.contact.surface;
  let m = 1, nub = 0;
  if (0 > s.mu) s.mu = 0;
  if (s.mode & CONTACT_MU2) {
    if (0 > s.mu2) s.mu2 = 0;
    if (s.mu > 0) m = 2;
    if (s.mu2 > 0) m++;
    if (s.mu === INF) nub = 1;
    if (s.mu2 === INF) nub++;
  } else {
    if (s.mu > 0) m = 3;
    if (s.mu === INF) nub = 2;
  }
  j.theM = m;
  return [m, nub];
}

// Info2: rows of J for both bodies (8 values per row: linear 0..2, angular 4..6), c, cfm, lo, hi, findex
function setFixedOrientation(b0, b1, fps, erp, J, o1, o2, c, co, qrel, startRow) {
  const si = startRow * 8;
  J[o1 + 4 + si] = 1; J[o1 + 4 + si + 9] = 1; J[o1 + 4 + si + 18] = 1;
  if (b1) { J[o2 + 4 + si] = -1; J[o2 + 4 + si + 9] = -1; J[o2 + 4 + si + 18] = -1; }
  const qerr = b1 ? qMultiply2(qMultiply1(b0.q, b1.q), qrel) : qMultiply3(b0.q, qrel);
  if (!(qerr[0] >= 0)) { qerr[1] = -qerr[1]; qerr[2] = -qerr[2]; qerr[3] = -qerr[3]; }
  const e = mul0_331(b0.R, [qerr[1], qerr[2], qerr[3]]);
  const k = fps * erp;
  c[co + startRow] = 2 * k * e[0];
  c[co + startRow + 1] = 2 * k * e[1];
  c[co + startRow + 2] = 2 * k * e[2];
}

function jointInfo2(jt, fps, worldErp, maxVel, minDepth, J, o1, o2, c, cfm, lo, hi, findex, ro) {
  const b0 = jt.node[0].body, b1 = jt.node[1].body;
  switch (jt.kind) {
    case 'ball': {
      cfm[ro] = jt.cfm; cfm[ro + 1] = jt.cfm; cfm[ro + 2] = jt.cfm;
      J[o1] = 1; J[o1 + 9] = 1; J[o1 + 18] = 1;
      const a1 = mul0_331(b0.R, jt.anchor1);
      setCrossMinus(J, o1 + 4, a1, 8);
      let a2 = [0, 0, 0];
      if (b1) {
        J[o2] = -1; J[o2 + 9] = -1; J[o2 + 18] = -1;
        a2 = mul0_331(b1.R, jt.anchor2);
        setCrossPlus(J, o2 + 4, a2, 8);
      }
      const k = fps * jt.erp;
      for (let j = 0; j < 3; j++)
        c[ro + j] = b1 ? k * (a2[j] + b1.pos[j] - a1[j] - b0.pos[j]) : k * (jt.anchor2[j] - a1[j] - b0.pos[j]);
      break;
    }
    case 'dball': {
      cfm[ro] = jt.cfm;
      const ga1 = relPointPos(b0, jt.anchor1);
      const ga2 = b1 ? relPointPos(b1, jt.anchor2) : jt.anchor2.slice();
      let q = [ga1[0] - ga2[0], ga1[1] - ga2[1], ga1[2] - ga2[2]];
      if (!(length3(q) >= 1e-7)) {
        const v1 = pointVel(b0, ga1), v2 = b1 ? pointVel(b1, ga2) : [0, 0, 0];
        q = [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]];
        if (!(length3(q) >= 1e-7)) q = [1, 0, 0];
      }
      safeNormalize3(q);
      J[o1] = q[0]; J[o1 + 1] = q[1]; J[o1 + 2] = q[2];
      const ra1 = mul0_331(b0.R, jt.anchor1);
      // (a1 x)^T-minus times q: [a1]x^- as a matrix, then transposed product
      const m1 = new Array(12).fill(0);
      setCrossMinus(m1, 0, ra1, 4);
      const j1a = mul1_331(m1, q);
      J[o1 + 4] = j1a[0]; J[o1 + 5] = j1a[1]; J[o1 + 6] = j1a[2];
      if (b1) {
        J[o2] = -q[0]; J[o2 + 1] = -q[1]; J[o2 + 2] = -q[2];
        const ra2 = mul0_331(b1.R, jt.anchor2);
        const m2 = new Array(12).fill(0);
        setCrossPlus(m2, 0, ra2, 4);
        const j2a = mul1_331(m2, q);
        J[o2 + 4] = j2a[0]; J[o2 + 5] = j2a[1]; J[o2 + 6] = j2a[2];
      }
      const k = fps * jt.erp;
      c[ro] = k * (jt.targetDistance - length3([ga1[0] - ga2[0], ga1[1] - ga2[1], ga1[2] - ga2[2]]));
      break;
    }
    case 'fixed': {
      setFixedOrientation(b0, b1, fps, worldErp, J, o1, o2, c, ro, jt.qrel, 3);
      J[o1] = 1; J[o1 + 9] = 1; J[o1 + 18] = 1;
      cfm[ro] = jt.cfm; cfm[ro + 1] = jt.cfm; cfm[ro + 2] = jt.cfm;
      const ofs = mul0_331(b0.R, jt.offset);
      if (b1) {
        setCrossPlus(J, o1 + 4, ofs, 8);
        J[o2] = -1; J[o2 + 9] = -1; J[o2 + 18] = -1;
      }
      const k = fps * jt.erp;
      for (let j = 0; j < 3; j++) c[ro + j] = b1 ? k * (b1.pos[j] - b0.pos[j] + ofs[j]) : k * (jt.offset[j] - b0.pos[j]);
      break;
    }
    case 'slider': {
      const cc = [0, 0, 0];
      if (b1) for (let i = 0; i < 3; i++) cc[i] = b1.pos[i] - b0.pos[i];
      setFixedOrientation(b0, b1, fps, worldErp, J, o1, o2, c, ro, jt.qrel, 0);
      const ax1 = mul0_331(b0.R, jt.axis1);
      const p = [0, 0, 0], q = [0, 0, 0];
      planeSpace(ax1, p, q);
      const s3 = 24, s4 = 32;
      if (b1) {
        let t = cross3(cc, p);
        for (let i = 0; i < 3; i++) { J[o1 + s3 + 4 + i] = 0.5 * t[i]; J[o2 + s3 + 4 + i] = 0.5 * t[i]; }
        t = cross3(cc, q);
        for (let i = 0; i < 3; i++) { J[o1 + s4 + 4 + i] = 0.5 * t[i]; J[o2 + s4 + 4 + i] = 0.5 * t[i]; }
        for (let i = 0; i < 3; i++) { J[o2 + s3 + i] = -p[i]; J[o2 + s4 + i] = -q[i]; }
      }
      for (let i = 0; i < 3; i++) { J[o1 + s3 + i] = p[i]; J[o1 + s4 + i] = q[i]; }
      const k = fps * worldErp;
      if (b1) {
        const ofs = mul0_331(b1.R, jt.offset);
        for (let i = 0; i < 3; i++) cc[i] += ofs[i];
        c[ro + 3] = k * dot3(p, cc);
        c[ro + 4] = k * dot3(q, cc);
      } else {
        const ofs = [jt.offset[0] - b0.pos[0], jt.offset[1] - b0.pos[1], jt.offset[2] - b0.pos[2]];
        c[ro + 3] = k * dot3(p, ofs);
        c[ro + 4] = k * dot3(q, ofs);
      }
      break;
    }
    case 'contact': contactInfo2(jt, b0, b1, fps, worldErp, maxVel, minDepth, J, o1, o2, c, cfm, lo, hi, findex, ro); break;
  }
}

function contactInfo2(jt, b0, b1, fps, worldErp, maxVel, minDepth, J, o1, o2, c, cfm, lo, hi, findex, ro) {
  const { surface: s, geom } = jt.contact;
  const mode = s.mode;
  const normal = jt.flags & JOINT_REVERSE ? [-geom.normal[0], -geom.normal[1], -geom.normal[2]] : geom.normal.slice();
  const c1 = [geom.pos[0] - b0.pos[0], geom.pos[1] - b0.pos[1], geom.pos[2] - b0.pos[2]];
  const c2 = b1 ? [geom.pos[0] - b1.pos[0], geom.pos[1] - b1.pos[1], geom.pos[2] - b1.pos[2]] : [0, 0, 0];
  const setDir = (row, t) => {
    const r = o1 + row * 8, r2 = o2 + row * 8;
    J[r] = t[0]; J[r + 1] = t[1]; J[r + 2] = t[2];
    const x = cross3(c1, t);
    J[r + 4] = x[0]; J[r + 5] = x[1]; J[r + 6] = x[2];
    if (b1) {
      J[r2] = -t[0]; J[r2 + 1] = -t[1]; J[r2 + 2] = -t[2];
      const y = cross3(c2, t);
      J[r2 + 4] = -y[0]; J[r2 + 5] = -y[1]; J[r2 + 6] = -y[2];
    }
  };
  setDir(0, normal);
  const erp = mode & CONTACT_SOFT_ERP ? s.softErp : worldErp;
  const k = erp * fps;
  let depth = geom.depth - minDepth;
  if (!(depth >= 0)) depth = 0;
  if (mode & CONTACT_SOFT_CFM) cfm[ro] = s.softCfm;
  const motionN = mode & CONTACT_MOTION_N ? s.motionN : 0;
  c[ro] = depth * k + motionN;
  if (!(maxVel >= c[ro])) c[ro] = maxVel;
  if (mode & CONTACT_BOUNCE) {
    let outgoing = (J[o1 + 4] * b0.avel[0] + J[o1 + 5] * b0.avel[1] + J[o1 + 6] * b0.avel[2]) +
      (J[o1] * b0.lvel[0] + J[o1 + 1] * b0.lvel[1] + J[o1 + 2] * b0.lvel[2]);
    if (b1) outgoing += (J[o2 + 4] * b1.avel[0] + J[o2 + 5] * b1.avel[1] + J[o2 + 6] * b1.avel[2]) +
      (J[o2] * b1.lvel[0] + J[o2 + 1] * b1.lvel[1] + J[o2 + 2] * b1.lvel[2]);
    outgoing -= motionN;
    if (s.bounceVel >= 0 && -outgoing > s.bounceVel) {
      const newc = motionN - outgoing * s.bounce;
      if (newc > c[ro]) c[ro] = newc;
    }
  }
  lo[ro] = 0; hi[ro] = INF;
  if (jt.theM === 1) return;
  let t1 = [0, 0, 0], t2 = [0, 0, 0];
  if (mode & CONTACT_FDIR1) { t1 = jt.contact.fdir1.slice(); t2 = cross3(normal, t1); }
  else planeSpace(normal, t1, t2);
  let second = 2, rowF2 = 2;
  if (!(0 >= s.mu)) {
    setDir(1, t1);
    if (mode & CONTACT_MOTION1) c[ro + 1] = s.motion1;
    lo[ro + 1] = -s.mu; hi[ro + 1] = s.mu;
    if (mode & CONTACT_APPROX1_1) findex[ro + 1] = 0;
    if (mode & CONTACT_SLIP1) cfm[ro + 1] = s.slip1;
  } else { second = 1; rowF2 = 1; }
  const mu2 = mode & CONTACT_MU2 ? s.mu2 : s.mu;
  if (mu2 > 0) {
    setDir(second, t2);
    if (mode & CONTACT_MOTION2) c[ro + rowF2] = s.motion2;
    lo[ro + rowF2] = -mu2; hi[ro + rowF2] = mu2;
    if (mode & CONTACT_APPROX1_2) findex[ro + rowF2] = 0;
    if (mode & CONTACT_SLIP2) cfm[ro + rowF2] = s.slip2;
  }
}

export function surfaceParameters(o = {}) {
  return { mode: 0, mu: 0, mu2: 0, bounce: 0, bounceVel: 0, softErp: 0, softCfm: 0, motion1: 0, motion2: 0, motionN: 0, slip1: 0, slip2: 0, ...o };
}

// --- the world ---------------------------------------------------------------------------------------------

export class World {
  constructor() {
    this.bodies = [];
    this.joints = [];
    this.freeJoints = [];
    this.groups = [];
    this.firstBody = null;
    this.firstJoint = null;
    this.gravity = [0, 0, 0];
    this.erp = 0.2;
    this.cfm = 1e-5;
    this.contactMaxVel = INF;
    this.contactMinDepth = 0;
    this.stats = { islands: 0, rows: 0, pivots: 0, lcpErrors: 0 };
  }

  // the world PhysicsCore::PhysicsCore gives AC: gravity -9.806, ERP 0.3, CFM 1e-7, contacts at most 3 m/s apart
  static assettoCorsa() {
    const w = new World();
    w.gravity = [0, -9.806, 0];
    w.erp = 0.3;
    w.cfm = 1e-7;
    w.contactMaxVel = 3;
    w.contactMinDepth = 0;
    return w;
  }

  bodyCreate() {
    const b = new Body(this.bodies.length, this.firstBody);
    this.bodies.push(b);
    this.firstBody = b;
    return b;
  }
  bodySetPosition(b, x, y, z) { b.pos[0] = x; b.pos[1] = y; b.pos[2] = z; }
  bodySetRotation(b, R) {
    for (let i = 0; i < 12; i++) b.R[i] = R[i];
    orthogonalizeR(b.R);
    b.q = qFromR(R);
    safeNormalize4(b.q);
  }
  bodySetMass(b, m) {
    b.mass = { mass: m.mass, c: m.c.slice(), I: m.I.slice() };
    if (!invertPD3(b.mass.I, b.invI)) b.invI = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
    b.invMass = 1 / m.mass;
  }
  bodySetFiniteRotationMode(b, on) {
    b.flags &= ~(BODY_FINITE_ROTATION | BODY_FINITE_ROTATION_AXIS);
    if (on) {
      b.flags |= BODY_FINITE_ROTATION;
      if (b.finiteRotAxis[0] !== 0 || b.finiteRotAxis[1] !== 0 || b.finiteRotAxis[2] !== 0) b.flags |= BODY_FINITE_ROTATION_AXIS;
    }
  }
  bodySetFiniteRotationAxis(b, x, y, z) {
    b.finiteRotAxis = [x, y, z];
    if (x !== 0 || y !== 0 || z !== 0) { safeNormalize3(b.finiteRotAxis); b.flags |= BODY_FINITE_ROTATION_AXIS; }
    else b.flags &= ~BODY_FINITE_ROTATION_AXIS;
  }
  bodyEnable(b) { b.flags &= ~BODY_DISABLED; }
  bodyDisable(b) { b.flags |= BODY_DISABLED; }

  addForce(b, f) { for (let i = 0; i < 3; i++) b.facc[i] = f[i] + b.facc[i]; }
  addTorque(b, t) { for (let i = 0; i < 3; i++) b.tacc[i] = t[i] + b.tacc[i]; }
  addRelTorque(b, t) { this.addTorque(b, mul0_331(b.R, t)); }
  #arm(b, f, q) {
    for (let i = 0; i < 3; i++) b.facc[i] = f[i] + b.facc[i];
    const c = cross3(q, f);
    for (let i = 0; i < 3; i++) b.tacc[i] = c[i] + b.tacc[i];
  }
  addForceAtPos(b, f, p) { this.#arm(b, f, [p[0] - b.pos[0], p[1] - b.pos[1], p[2] - b.pos[2]]); }
  addForceAtRelPos(b, f, p) { this.#arm(b, f, mul0_331(b.R, p)); }
  addRelForceAtPos(b, f, p) { this.#arm(b, mul0_331(b.R, f), [p[0] - b.pos[0], p[1] - b.pos[1], p[2] - b.pos[2]]); }
  addRelForceAtRelPos(b, f, p) { this.#arm(b, mul0_331(b.R, f), mul0_331(b.R, p)); }

  getRelPointPos(b, p) { return relPointPos(b, p); }
  getPosRelPoint(b, p) { return posRelPoint(b, p); }
  vectorToWorld(b, v) { return mul0_331(b.R, v); }
  vectorFromWorld(b, v) { return mul1_331(b.R, v); }
  getPointVel(b, p) { return pointVel(b, p); }
  getRelPointVel(b, p) {
    const c = cross3(b.avel, mul0_331(b.R, p));
    return [c[0] + b.lvel[0], c[1] + b.lvel[1], c[2] + b.lvel[2]];
  }

  #jointCreate(kind) {
    let id;
    if (this.freeJoints.length) id = this.freeJoints.pop();
    else { id = this.joints.length; this.joints.push(null); }
    const j = new Joint(id, this.firstJoint, kind, this);
    this.joints[id] = j;
    this.firstJoint = j;
    return j;
  }
  jointCreateBall() { return this.#jointCreate('ball'); }
  jointCreateDBall() { return this.#jointCreate('dball'); }
  jointCreateFixed() { return this.#jointCreate('fixed'); }
  jointCreateSlider() { return this.#jointCreate('slider'); }
  jointGroupCreate() { this.groups.push([]); return this.groups.length - 1; }
  jointCreateContact(group, contact) {
    const j = this.#jointCreate('contact');
    j.contact = { surface: { ...contact.surface }, geom: contact.geom, fdir1: contact.fdir1 || [0, 0, 0] };
    if (group != null) { j.flags |= JOINT_INGROUP; j.group = group; this.groups[group].push(j); }
    return j;
  }
  jointGroupEmpty(group) {
    const list = this.groups[group];
    for (let i = list.length - 1; i >= 0; i--) {
      const j = list[i];
      this.#removeJointReferences(j);
      if (this.firstJoint === j) this.firstJoint = j.next;
      else for (let c = this.firstJoint; c; c = c.next) if (c.next === j) { c.next = j.next; break; }
      j.next = null;
      j.alive = false;
      this.freeJoints.push(j.id);
    }
    list.length = 0;
  }
  #removeJointReferences(j) {
    for (let i = 0; i < 2; i++) {
      const body = j.node[i].body;
      if (!body) continue;
      let prev = null;
      for (let n = body.firstJoint; n; ) {
        const next = n.joint.node[n.node].next;
        if (n.joint === j) {
          if (prev) prev.joint.node[prev.node].next = next;
          else body.firstJoint = next;
          break;
        }
        prev = n;
        n = next;
      }
    }
    j.node[0].body = null; j.node[0].next = null;
    j.node[1].body = null; j.node[1].next = null;
  }
  jointAttach(j, body1, body2) {
    if (j.node[0].body || j.node[1].body) this.#removeJointReferences(j);
    if (!body1) { j.flags |= JOINT_REVERSE; body1 = body2; body2 = null; }
    else j.flags &= ~JOINT_REVERSE;
    j.node[0].body = body1;
    j.node[1].body = body2;
    if (body1) { j.node[1].next = body1.firstJoint; body1.firstJoint = { joint: j, node: 1 }; }
    else j.node[1].next = null;
    if (body2) { j.node[0].next = body2.firstJoint; body2.firstJoint = { joint: j, node: 0 }; }
    else j.node[0].next = null;
    if (body1 || body2) this.#setRelativeValues(j);
  }
  #setRelativeValues(j) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    if (j.kind === 'dball') this.#dballUpdateTarget(j);
    else if (j.kind === 'ball') {
      let a = [0, 0, 0];
      if (j.flags & JOINT_REVERSE) a = b1 ? relPointPos(b1, j.anchor2) : j.anchor2.slice();
      else if (b0) a = relPointPos(b0, j.anchor1);
      this.jointSetBallAnchor(j, a[0], a[1], a[2]);
    } else if (j.kind === 'slider') { this.#sliderComputeOffset(j); this.#initialRelativeRotation(j); }
  }
  #dballUpdateTarget(j) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    const p1 = b0 ? relPointPos(b0, j.anchor1) : j.anchor1.slice();
    const p2 = b1 ? relPointPos(b1, j.anchor2) : j.anchor2.slice();
    j.targetDistance = length3([p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2]]);
  }
  #dballSetAnchor(j, node, p) {
    const body = j.node[node].body;
    const local = body ? posRelPoint(body, p) : p.slice();
    if (node === 0) j.anchor1 = local; else j.anchor2 = local;
  }
  jointSetDBallAnchor1(j, x, y, z) { this.#dballSetAnchor(j, j.flags & JOINT_REVERSE ? 1 : 0, [x, y, z]); this.#dballUpdateTarget(j); }
  jointSetDBallAnchor2(j, x, y, z) { this.#dballSetAnchor(j, j.flags & JOINT_REVERSE ? 0 : 1, [x, y, z]); this.#dballUpdateTarget(j); }
  jointSetDBallDistance(j, d) { j.targetDistance = d; }
  jointGetDBallDistance(j) { return j.targetDistance; }
  jointSetBallAnchor(j, x, y, z) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    if (b0) {
      j.anchor1 = posRelPoint(b0, [x, y, z]);
      j.anchor2 = b1 ? posRelPoint(b1, [x, y, z]) : [x, y, z];
    }
  }
  jointSetFixed(j) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    if (b0) j.offset = b1 ? mul1_331(b0.R, [b0.pos[0] - b1.pos[0], b0.pos[1] - b1.pos[1], b0.pos[2] - b1.pos[2]]) : b0.pos.slice();
    this.#initialRelativeRotation(j);
  }
  #initialRelativeRotation(j) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    if (!b0) return;
    j.qrel = b1 ? qMultiply1(b0.q, b1.q) : [b0.q[0], -b0.q[1], -b0.q[2], -b0.q[3]];
  }
  #sliderComputeOffset(j) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    if (b1) j.offset = mul1_331(b1.R, [b0.pos[0] - b1.pos[0], b0.pos[1] - b1.pos[1], b0.pos[2] - b1.pos[2]]);
    else if (b0) j.offset = b0.pos.slice();
  }
  jointSetSliderAxis(j, x, y, z) {
    const b0 = j.node[0].body;
    if (b0) { const q = [x, y, z]; safeNormalize3(q); j.axis1 = mul1_331(b0.R, q); }
    this.#sliderComputeOffset(j);
    this.#initialRelativeRotation(j);
  }
  jointGetSliderPosition(j) {
    const b0 = j.node[0].body, b1 = j.node[1].body;
    if (j.kind !== 'slider' || !b0) return 0;
    let ax1 = mul0_331(b0.R, j.axis1), q;
    if (b1) { const w = mul0_331(b1.R, j.offset); q = [b0.pos[0] - w[0] - b1.pos[0], b0.pos[1] - w[1] - b1.pos[1], b0.pos[2] - w[2] - b1.pos[2]]; }
    else { q = [b0.pos[0] - j.offset[0], b0.pos[1] - j.offset[1], b0.pos[2] - j.offset[2]]; if (j.flags & JOINT_REVERSE) ax1 = ax1.map(v => -v); }
    return dot3(ax1, q);
  }
  jointSetParam(j, param, value) {
    if (param === PARAM_CFM) j.cfm = value;
    else if (param === PARAM_ERP) j.erp = value;
  }
  jointSetFeedback(j, on) { j.feedback = on ? { f1: [0, 0, 0], t1: [0, 0, 0], f2: [0, 0, 0], t2: [0, 0, 0] } : null; }
  jointGetBody(j, i) { return j.flags & JOINT_REVERSE ? j.node[1 - i].body : j.node[i].body; }

  // dWorldStep: islands, then each island through the big-matrix stepper
  step(h) {
    for (let b = this.firstBody; b; b = b.next) b.tag = 0;
    for (let j = this.firstJoint; j; j = j.next) j.tag = 0;
    const st = this.stats;
    st.islands = 0; st.rows = 0; st.pivots = 0; st.lcpErrors = 0;
    for (let bb = this.firstBody; bb; bb = bb.next) {
      if (bb.tag !== 0) continue;
      if (bb.flags & BODY_DISABLED) { bb.tag = -1; continue; }
      bb.tag = 1;
      const bodies = [bb], joints = [], stack = [];
      let b = bb;
      for (;;) {
        for (let n = b.firstJoint; n; ) {
          const jt = n.joint, jn = jt.node[n.node];
          if (jt.tag === 0) {
            if (jt.isEnabled()) {
              jt.tag = 1;
              joints.push(jt);
              const nb = jn.body;
              if (nb && nb.tag <= 0) { nb.tag = 1; nb.flags &= ~BODY_DISABLED; stack.push(nb); }
            } else jt.tag = -1;
          }
          n = jn.next;
        }
        if (!stack.length) break;
        b = stack.pop();
        bodies.push(b);
      }
      this.#stepIsland(bodies, joints, h);
      st.islands++;
    }
  }

  #stepIsland(bodies, ijoints, h) {
    const nb = bodies.length;
    for (let i = 0; i < nb; i++) bodies[i].tag = i;
    for (let ax = 0; ax < 3; ax++) {
      const g = this.gravity[ax];
      if (g === 0 || Number.isNaN(g)) continue;
      for (const b of bodies) if (!(b.flags & BODY_NO_GRAVITY)) b.facc[ax] = b.mass.mass * g + b.facc[ax];
    }
    const invI = new Float64Array(12 * nb), tmp = new Array(12).fill(0);
    for (let i = 0; i < nb; i++) {
      const b = bodies[i], row = new Array(12).fill(0);
      mul2_333(tmp, b.invI, b.R);
      mul0_333(row, b.R, tmp);
      for (let k = 0; k < 12; k++) invI[12 * i + k] = row[k];
      if (b.flags & BODY_GYROSCOPIC && !(0 >= b.invMass)) gyroscopicTorque(b, h);
    }

    // joint order: fully unbounded joints in reverse island order, then the bounded (contacts) in island order
    const unb = [], lcpJ = [];
    let m = 0;
    for (const jt of ijoints) {
      const [jm, jnub] = jt.info1();
      if (jm === 0) { jt.tag = -1; continue; }
      jt.m = jm;
      m += jm;
      if (jnub === jm) unb.unshift(jt);
      else if (jnub === 0) lcpJ.push(jt);
      else throw new Error('ode: a joint with both unbounded and bounded rows');
    }
    const joints = unb.concat(lcpJ), nj = joints.length, nub = unb.length;
    joints.forEach((jt, i) => { jt.tag = i; });
    this.stats.rows = Math.max(this.stats.rows, m);

    const rhsTmp = new Float64Array(8 * nb);
    let lambda = null, J = null;
    const mindex = new Int32Array(nj + 1);
    if (m > 0) {
      const recip = 1 / h, mskip = pad(m);
      for (let i = 0; i < nj; i++) mindex[i + 1] = mindex[i] + joints[i].m;
      J = new Float64Array(16 * m);
      const rhs = new Float64Array(m), cfm = new Float64Array(m).fill(this.cfm);
      const lo = new Float64Array(m).fill(-INF), hi = new Float64Array(m).fill(INF), findex = new Int32Array(m).fill(-1);
      for (let ji = 0; ji < nj; ji++) {
        const ofs = mindex[ji], im = mindex[ji + 1] - ofs;
        // J2 lives right after J1 in the same array: rows of body 1, then rows of body 2
        const base = 16 * ofs;
        jointInfo2(joints[ji], recip, this.erp, this.contactMaxVel, this.contactMinDepth, J, base, base + 8 * im, rhs, cfm, lo, hi, findex, ofs);
        for (let k = 0; k < im; k++) {
          rhs[ofs + k] *= recip;
          if (findex[ofs + k] !== -1) findex[ofs + k] += ofs;
        }
      }
      const A = new Float64Array(m * mskip);
      for (let r = 0; r < m; r++) A[r * mskip + r] = cfm[r] * recip;
      const JinvM = new Float64Array(16 * m);
      for (let ji = 0; ji < nj; ji++) {
        const ofs = mindex[ji], im = mindex[ji + 1] - ofs, jt = joints[ji];
        let src = 16 * ofs;
        for (let node = 0; node < 2; node++) {
          const b = jt.node[node].body;
          if (!b) { src += 8 * im; continue; }
          const t = b.tag * 12;
          for (let r = 0; r < im; r++) {
            for (let k = 0; k < 3; k++) JinvM[src + k] = J[src + k] * b.invMass;
            const a0 = J[src + 4], a1 = J[src + 5], a2 = J[src + 6];
            JinvM[src + 4] = a0 * invI[t] + a1 * invI[t + 4] + a2 * invI[t + 8];
            JinvM[src + 5] = a0 * invI[t + 1] + a1 * invI[t + 5] + a2 * invI[t + 9];
            JinvM[src + 6] = a0 * invI[t + 2] + a1 * invI[t + 6] + a2 * invI[t + 10];
            src += 8;
          }
        }
      }
      for (let bi = 0; bi < nb; bi++) {
        const b = bodies[bi], t = 12 * bi, o = 8 * bi;
        for (let j = 0; j < 3; j++) rhsTmp[o + j] = b.facc[j] * b.invMass + b.lvel[j] * recip;
        for (let k = 0; k < 3; k++)
          rhsTmp[o + 4 + k] = invI[t + 4 * k] * b.tacc[0] + invI[t + 4 * k + 1] * b.tacc[1] + invI[t + 4 * k + 2] * b.tacc[2] + b.avel[k] * recip;
      }
      const mulAdd = (aOff, B, bOff, C, cOff, p, r) => {
        for (let i = 0; i < p; i++) {
          const bb = bOff + 8 * i;
          for (let j = 0; j < r; j++) {
            const cc = cOff + 8 * j;
            A[aOff + i * mskip + j] += B[bb] * C[cc] + B[bb + 1] * C[cc + 1] + B[bb + 2] * C[cc + 2] +
              B[bb + 4] * C[cc + 4] + B[bb + 5] * C[cc + 5] + B[bb + 6] * C[cc + 6];
          }
        }
      };
      for (let ji = 0; ji < nj; ji++) {
        const ofs = mindex[ji], im = mindex[ji + 1] - ofs, jt = joints[ji];
        for (let node = 0; node < 2; node++) {
          const jb = jt.node[node].body;
          if (!jb) continue;
          const row = 16 * ofs + node * 8 * im;
          mulAdd(ofs * mskip + ofs, JinvM, row, J, row, im, im);
          if (ji === 0) continue;
          for (let n = jb.firstJoint; n; ) {
            const other = n.joint;
            n = other.node[n.node].next;
            const j0 = other.tag;
            if (j0 !== -1 && j0 < ji && other.alive) {
              const oofs = mindex[j0], oim = mindex[j0 + 1] - oofs;
              const ofsOther = other.node[1].body === jb ? 8 * oim : 0;
              mulAdd(ofs * mskip + oofs, JinvM, row, J, 16 * oofs + ofsOther, im, oim);
            }
          }
        }
      }
      for (let ji = 0; ji < nj; ji++) {
        const ofs = mindex[ji], im = mindex[ji + 1] - ofs, jt = joints[ji];
        for (let node = 0; node < 2; node++) {
          const jb = jt.node[node].body;
          if (!jb) continue;
          const c = 8 * jb.tag;
          let bb = 16 * ofs + node * 8 * im;
          for (let i = 0; i < im; i++) {
            rhs[ofs + i] -= J[bb] * rhsTmp[c] + J[bb + 1] * rhsTmp[c + 1] + J[bb + 2] * rhsTmp[c + 2] +
              J[bb + 4] * rhsTmp[c + 4] + J[bb + 5] * rhsTmp[c + 5] + J[bb + 6] * rhsTmp[c + 6];
            bb += 8;
          }
        }
      }
      lambda = new Float64Array(m);
      if (lcpJ.length === 0) {
        const d = new Float64Array(m);
        factorLDLT(A, d, m, mskip);
        solveLDLT(A, d, rhs, m, mskip);
        lambda.set(rhs);
      } else {
        const s = solveLCP(m, A, lambda, rhs, nub, lo, hi, findex);
        this.stats.pivots += s.pivots;
        if (s.error) this.stats.lcpErrors++;
      }
    }

    const cforce = new Float64Array(8 * nb);
    if (m > 0) {
      for (let ji = 0; ji < nj; ji++) {
        const ofs = mindex[ji], im = mindex[ji + 1] - ofs, jt = joints[ji];
        for (let node = 0; node < 2; node++) {
          const b = jt.node[node].body;
          if (!b) continue;
          const jj = 16 * ofs + node * 8 * im;
          let s0 = 0, s1 = 0, s2 = 0, s4 = 0, s5 = 0, s6 = 0;
          for (let k = 0; k < im; k++) {
            const r = jj + 8 * k, l = lambda[ofs + k];
            s0 += J[r] * l; s1 += J[r + 1] * l; s2 += J[r + 2] * l;
            s4 += J[r + 4] * l; s5 += J[r + 5] * l; s6 += J[r + 6] * l;
          }
          const cf = 8 * b.tag;
          cforce[cf] += s0; cforce[cf + 1] += s1; cforce[cf + 2] += s2;
          cforce[cf + 4] += s4; cforce[cf + 5] += s5; cforce[cf + 6] += s6;
          if (jt.feedback) {
            if (node === 0) { jt.feedback.f1 = [s0, s1, s2]; jt.feedback.t1 = [s4, s5, s6]; }
            else { jt.feedback.f2 = [s0, s1, s2]; jt.feedback.t2 = [s4, s5, s6]; }
          }
        }
      }
    }

    for (let bi = 0; bi < nb; bi++) {
      const b = bodies[bi], t = 12 * bi, cf = 8 * bi;
      const k = h * b.invMass;
      for (let j = 0; j < 3; j++) b.lvel[j] = (cforce[cf + j] + b.facc[j]) * k + b.lvel[j];
      const d0 = (cforce[cf + 4] + b.tacc[0]) * h, d1 = (cforce[cf + 5] + b.tacc[1]) * h, d2 = (cforce[cf + 6] + b.tacc[2]) * h;
      for (let r = 0; r < 3; r++) b.avel[r] = invI[t + 4 * r] * d0 + invI[t + 4 * r + 1] * d1 + invI[t + 4 * r + 2] * d2 + b.avel[r];
    }
    for (const b of bodies) stepBody(b, h);
    for (const b of bodies) { b.facc[0] = b.facc[1] = b.facc[2] = 0; b.tacc[0] = b.tacc[1] = b.tacc[2] = 0; }
  }
}

function gyroscopicTorque(b, h) {
  const tmp = new Array(12).fill(0), I = new Array(12).fill(0);
  mul2_333(tmp, b.mass.I, b.R);
  mul0_333(I, b.R, tmp);
  const L = mul0_331(I, b.avel);
  const itild = new Array(12).fill(0);
  setCrossMinus(itild, 0, L, 4);
  for (let i = 0; i < 12; i++) itild[i] = itild[i] * h + I[i];
  const r = 1 / h;
  L[0] *= r; L[1] *= r; L[2] *= r;
  const itInv = new Array(12).fill(0);
  if (invertMatrix3(itInv, itild) !== 0) {
    mul0_333(itild, I, itInv);
    itild[0] -= 1; itild[5] -= 1; itild[10] -= 1;
    const tau = mul0_331(itild, L);
    for (let i = 0; i < 3; i++) b.tacc[i] = tau[i] + b.tacc[i];
  }
}

const sinc = x => (Math.abs(x) < 1e-4 ? 1 - x * x * (1 / 6) : Math.sin(x) / x);

function stepBody(b, h) {
  for (let j = 0; j < 3; j++) b.pos[j] = h * b.lvel[j] + b.pos[j];
  if (b.flags & BODY_FINITE_ROTATION) {
    let irv = [0, 0, 0], q;
    if (b.flags & BODY_FINITE_ROTATION_AXIS) {
      const k = dot3(b.finiteRotAxis, b.avel);
      const frv = [b.finiteRotAxis[0] * k, b.finiteRotAxis[1] * k, b.finiteRotAxis[2] * k];
      irv = [b.avel[0] - frv[0], b.avel[1] - frv[1], b.avel[2] - frv[2]];
      h *= 0.5;
      const theta = k * h, s = sinc(theta) * h;
      q = [Math.cos(theta), frv[0] * s, frv[1] * s, frv[2] * s];
    } else {
      const wlen = Math.sqrt(b.avel[0] * b.avel[0] + b.avel[1] * b.avel[1] + b.avel[2] * b.avel[2]);
      h *= 0.5;
      const theta = wlen * h, s = sinc(theta) * h;
      q = [Math.cos(theta), b.avel[0] * s, b.avel[1] * s, b.avel[2] * s];
    }
    b.q = qMultiply0(q, b.q);
    if (b.flags & BODY_FINITE_ROTATION_AXIS) {
      const dq = dqFromW(irv, b.q);
      for (let j = 0; j < 4; j++) b.q[j] += h * dq[j];
    }
  } else {
    const dq = dqFromW(b.avel, b.q);
    for (let j = 0; j < 4; j++) b.q[j] += h * dq[j];
  }
  safeNormalize4(b.q);
  rFromQ(b.R, b.q);
}
