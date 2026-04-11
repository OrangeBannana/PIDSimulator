export const DEFAULT_CONTROLLER = {
  p: 0.014,
  i: 0,
  d: 0.00082,
  f: 0.1,
  dt: 0.026,
  variation: 0
};

export const DEFAULT_SIM = {
  target: 100,
  simTime: 3,
  simDt: 1,
  mass: 1.2,
  ratio: 0.2,
  spoolR: 0.02,
  fricCoeff: 0.5,
  cascadeRigging: false,
  stages: 2,
  counterSpring: 0,
  maxV: 12,
  freeRPM: 5900,
  stallTorque: 0.19,
  stallCurrent: 9.2,
  motorCount: 2
};
