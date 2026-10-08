export const D = {
  W: 0.90,
  WG: 0.868,
  TUMBLE: 0.12,
  SILL: 0.66,
  WAIST: 1.385,
  EAVE: 1.935,
  ROOF: 2.0,
  FRONT: -2.0,
  REAR: 2.33,
  SCREEN: -0.70,
  RAKE: 0.17,
  ARCH_F: -1.397, ARCH_R: 1.397,
  ARCH_TOP: 1.03,
  FLOOR: 0.78,
  DX: -0.42,
};
D.HEADER = D.SCREEN + D.RAKE;

D.pillarZ = (y) => D.SCREEN + (y - D.WAIST) * (D.RAKE / (D.EAVE - D.WAIST));

D.tumble = (y) => 1 - D.TUMBLE * Math.max(0, y - D.WAIST);
