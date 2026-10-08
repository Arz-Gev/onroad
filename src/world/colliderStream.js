import { MAP_SIZE } from './terrain.js';

export class ColliderStream {
  constructor(RAPIER, world, colliderSurface, { chunk = 32, ring = 2 } = {}) {
    this.RAPIER = RAPIER; this.world = world; this.surfaces = colliderSurface;
    this.chunk = chunk; this.ring = ring;
    this.cn = Math.ceil(MAP_SIZE / chunk);
    this.items = Array.from({ length: this.cn * this.cn }, () => []);
    this.active = new Map();
    this.cur = -1;
    this.total = 0;
  }

  chunkOf(x, z) {
    const h = MAP_SIZE / 2, n = this.cn;
    return Math.min(n - 1, Math.max(0, Math.floor((z + h) / this.chunk))) * n + Math.min(n - 1, Math.max(0, Math.floor((x + h) / this.chunk)));
  }

  add(x, z, desc, surface) {
    this.items[this.chunkOf(x, z)].push({ desc, surface });
    this.total++;
    this.cur = -1;
  }

  get count() { let n = 0; for (const l of this.active.values()) n += l.length; return n; }

  update(x, z) {
    const ci = this.chunkOf(x, z);
    if (ci === this.cur) return;
    this.cur = ci;
    const n = this.cn, cx = ci % n, cz = Math.floor(ci / n), R = this.ring;
    const want = new Set();
    for (let a = -R; a <= R; a++) for (let b = -R; b <= R; b++) {
      const x2 = cx + a, z2 = cz + b;
      if (x2 >= 0 && z2 >= 0 && x2 < n && z2 < n) want.add(z2 * n + x2);
    }
    for (const k of [...this.active.keys()]) {
      if (want.has(k)) continue;
      for (const c of this.active.get(k)) { this.surfaces.delete(c.handle); this.world.removeCollider(c, false); }
      this.active.delete(k);
    }
    for (const k of want) {
      if (this.active.has(k)) continue;
      const list = [];
      for (const it of this.items[k]) {
        const c = this.world.createCollider(it.desc);
        if (it.surface) this.surfaces.set(c.handle, it.surface);
        list.push(c);
      }
      this.active.set(k, list);
    }
  }
}
