// DDS (DXT1/3/5, uncompressed 32-bit) decode → RGBA, bilinear resize, PNG encode.
// Used for textures WebGL can't take as compressed data (sides not a multiple of 4 at every mip level).
import { deflateSync } from 'node:zlib';

export function ddsInfo(d) {
  const dv = new DataView(d.buffer, d.byteOffset, d.byteLength);
  if (dv.getUint32(0, true) !== 0x20534444) return null;
  const height = dv.getUint32(12, true), width = dv.getUint32(16, true);
  const mips = Math.max(1, dv.getUint32(28, true));
  const pfFlags = dv.getUint32(80, true);
  const fourcc = String.fromCharCode(d[84], d[85], d[86], d[87]);
  const bpp = dv.getUint32(88, true);
  return { width, height, mips, format: pfFlags & 4 ? fourcc : `RGB${bpp}`, masks: [92, 96, 100, 104].map((o) => dv.getUint32(o, true)) };
}

const isPot = (n) => (n & (n - 1)) === 0;
export function needsTranscode(info) {
  return !isPot(info.width) || !isPot(info.height) || !/^DXT[135]$/.test(info.format);
}

function rgb565(c, out, o) {
  out[o] = ((c >> 11) & 31) * 255 / 31; out[o + 1] = ((c >> 5) & 63) * 255 / 63; out[o + 2] = (c & 31) * 255 / 31;
}

export function decodeDds(d) {
  const info = ddsInfo(d);
  const { width: w, height: h, format } = info;
  const out = new Uint8Array(w * h * 4);
  const dv = new DataView(d.buffer, d.byteOffset, d.byteLength);
  let p = 128;
  if (/^RGB(32|24)$/.test(format)) {
    const bytes = format === 'RGB32' ? 4 : 3;
    const [rm, gm, bm, am] = info.masks;
    const sh = (m) => (m ? 31 - Math.clz32(m & -m) : 0);
    for (let i = 0; i < w * h; i++, p += bytes) {
      const v = bytes === 4 ? dv.getUint32(p, true) : d[p] | (d[p + 1] << 8) | (d[p + 2] << 16);
      out[i * 4] = (v & rm) >>> sh(rm); out[i * 4 + 1] = (v & gm) >>> sh(gm); out[i * 4 + 2] = (v & bm) >>> sh(bm);
      out[i * 4 + 3] = am ? (v & am) >>> sh(am) : 255;
    }
    return { width: w, height: h, data: out };
  }
  if (!/^DXT[135]$/.test(format)) throw new Error(`unsupported DDS format ${format}`);
  const bw = Math.ceil(w / 4), bh = Math.ceil(h / 4);
  const pal = new Uint8Array(16), alpha = new Uint8Array(16), a8 = new Uint8Array(8);
  for (let by = 0; by < bh; by++) for (let bx = 0; bx < bw; bx++) {
    alpha.fill(255);
    if (format === 'DXT3') {
      for (let i = 0; i < 8; i++) { const b = d[p + i]; alpha[i * 2] = (b & 15) * 17; alpha[i * 2 + 1] = (b >> 4) * 17; }
      p += 8;
    } else if (format === 'DXT5') {
      const a0 = d[p], a1 = d[p + 1];
      a8[0] = a0; a8[1] = a1;
      if (a0 > a1) for (let i = 1; i < 7; i++) a8[i + 1] = ((7 - i) * a0 + i * a1) / 7;
      else { for (let i = 1; i < 5; i++) a8[i + 1] = ((5 - i) * a0 + i * a1) / 5; a8[6] = 0; a8[7] = 255; }
      let bits = 0n;
      for (let i = 0; i < 6; i++) bits |= BigInt(d[p + 2 + i]) << BigInt(8 * i);
      for (let i = 0; i < 16; i++) alpha[i] = a8[Number((bits >> BigInt(3 * i)) & 7n)];
      p += 8;
    }
    const c0 = dv.getUint16(p, true), c1 = dv.getUint16(p + 2, true);
    rgb565(c0, pal, 0); rgb565(c1, pal, 4); pal[3] = pal[7] = 255;
    if (c0 > c1 || format !== 'DXT1') {
      for (let k = 0; k < 3; k++) { pal[8 + k] = (2 * pal[k] + pal[4 + k]) / 3; pal[12 + k] = (pal[k] + 2 * pal[4 + k]) / 3; }
      pal[11] = pal[15] = 255;
    } else {
      for (let k = 0; k < 3; k++) { pal[8 + k] = (pal[k] + pal[4 + k]) / 2; pal[12 + k] = 0; }
      pal[11] = 255; pal[15] = 0;
    }
    const idx = dv.getUint32(p + 4, true);
    p += 8;
    for (let i = 0; i < 16; i++) {
      const x = bx * 4 + (i & 3), y = by * 4 + (i >> 2);
      if (x >= w || y >= h) continue;
      const c = ((idx >>> (2 * i)) & 3) * 4, o = (y * w + x) * 4;
      out[o] = pal[c]; out[o + 1] = pal[c + 1]; out[o + 2] = pal[c + 2];
      out[o + 3] = format === 'DXT1' ? pal[c + 3] : alpha[i];
    }
  }
  return { width: w, height: h, data: out };
}

// Box-filtered down / bilinear up resize of RGBA8.
export function resize(img, W, H) {
  const { width: w, height: h, data } = img;
  if (w === W && h === H) return img;
  const out = new Uint8Array(W * H * 4);
  const sx = w / W, sy = h / H;
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const x0 = x * sx, x1 = (x + 1) * sx, y0 = y * sy, y1 = (y + 1) * sy;
    if (sx <= 1 && sy <= 1) {
      const fx = Math.min(w - 1, Math.max(0, (x + 0.5) * sx - 0.5)), fy = Math.min(h - 1, Math.max(0, (y + 0.5) * sy - 0.5));
      const ix = Math.floor(fx), iy = Math.floor(fy), tx = fx - ix, ty = fy - iy;
      const jx = Math.min(w - 1, ix + 1), jy = Math.min(h - 1, iy + 1);
      for (let c = 0; c < 4; c++) {
        const a = data[(iy * w + ix) * 4 + c], b = data[(iy * w + jx) * 4 + c], cc = data[(jy * w + ix) * 4 + c], dd = data[(jy * w + jx) * 4 + c];
        out[(y * W + x) * 4 + c] = (a * (1 - tx) + b * tx) * (1 - ty) + (cc * (1 - tx) + dd * tx) * ty;
      }
      continue;
    }
    const acc = [0, 0, 0, 0]; let n = 0;
    for (let yy = Math.floor(y0); yy < Math.min(h, Math.ceil(y1)); yy++) for (let xx = Math.floor(x0); xx < Math.min(w, Math.ceil(x1)); xx++) {
      const o = (yy * w + xx) * 4; acc[0] += data[o]; acc[1] += data[o + 1]; acc[2] += data[o + 2]; acc[3] += data[o + 3]; n++;
    }
    for (let c = 0; c < 4; c++) out[(y * W + x) * 4 + c] = acc[c] / n;
  }
  return { width: W, height: H, data: out };
}

export const nearestPot = (n, max = 2048) => Math.min(max, 2 ** Math.max(0, Math.round(Math.log2(n))));

let crcTable;
function crc32(b) {
  if (!crcTable) {
    crcTable = new Uint32Array(256);
    for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; crcTable[n] = c >>> 0; }
  }
  let c = 0xffffffff;
  for (let i = 0; i < b.length; i++) c = crcTable[(c ^ b[i]) & 255] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}
export function encodePng({ width, height, data }) {
  const raw = Buffer.alloc((width * 4 + 1) * height);
  for (let y = 0; y < height; y++) Buffer.from(data.buffer, data.byteOffset + y * width * 4, width * 4).copy(raw, y * (width * 4 + 1) + 1);
  const chunk = (type, body) => {
    const len = Buffer.alloc(4); len.writeUInt32BE(body.length);
    const tb = Buffer.concat([Buffer.from(type), body]);
    const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(tb));
    return Buffer.concat([len, tb, crc]);
  };
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(width, 0); ihdr.writeUInt32BE(height, 4); ihdr[8] = 8; ihdr[9] = 6;
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', deflateSync(raw, { level: 6 })), chunk('IEND', Buffer.alloc(0))]);
}
