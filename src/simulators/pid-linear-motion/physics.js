export function createMotor({ maxV, freeRPM, stallCurrent, stallTorque, motorCount }) {
  const kt = maxV / ((freeRPM * 2 * Math.PI) / 60);
  const resistance = maxV / stallCurrent;
  const count = motorCount;

  return {
    maxV,
    kt,
    resistance,
    count,
    stallTorque,
    getTorque(v, omega) {
      return (kt * (v - kt * omega) / resistance) * count;
    },
    getCurrent(v, omega) {
      return ((v - kt * omega) / resistance) * count;
    }
  };
}

export function createMechanism({ mass, ratio, spoolR, fricCoeff, cascadeRigging, stages, counterSpring }) {
  const g = -9.81;
  const kineticFric = fricCoeff * Math.abs(g) * mass * spoolR;

  return {
    mass,
    ratio,
    spoolR,
    g,
    kineticFric,
    cascadeRigging,
    stages,
    counterSpring,
    getEffectiveRatio() {
      return cascadeRigging ? ratio / stages : ratio;
    },
    getDistanceMultiplier() {
      return cascadeRigging ? stages : 1;
    },
    getInertia() {
      return mass * spoolR * spoolR;
    },
    getGravityTorque() {
      return g * mass * spoolR;
    },
    getCounterSpringTorque() {
      return counterSpring * spoolR;
    },
    getLinearPos(angPos) {
      const effectiveRatio = this.getEffectiveRatio();
      const distanceMultiplier = this.getDistanceMultiplier();
      return angPos * effectiveRatio * distanceMultiplier * this.spoolR * 100;
    }
  };
}

export function pidCalc(ctrl, state, mech, targetCm, dt, maxV) {
  const ticksPerRev = 28;
  const targetTicks = ticksPerRev * ((targetCm / (2 * Math.PI * mech.spoolR * 100)) / mech.ratio);
  const currTicks = (state.pos / (2 * Math.PI)) * ticksPerRev;
  const err = targetTicks - currTicks;
  const deriv = (err - ctrl.prevErr) / dt;
  ctrl.integral += err * dt;

  let v = (err * ctrl.p + ctrl.integral * ctrl.i + deriv * ctrl.d + ctrl.f) * maxV;
  v = Math.min(Math.max(v, -maxV), maxV);
  ctrl.prevErr = err;
  return v;
}

export function initSim({ simDt, motor, mech, target, totalTime, controllers }) {
  const states = controllers.map(() => ({ pos: 0, vel: 0, t: 0 }));
  controllers.forEach(ctrl => {
    ctrl.integral = 0;
    ctrl.prevErr = 0;
  });

  return { simDt, motor, mech, target, totalTime, states, done: false };
}

export function normalRandom(mean = 0, std = 1) {
  const u1 = Math.random();
  const u2 = Math.random();
  const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  return z0 * std + mean;
}

export function stepBatch(simState, controllers, batchSize = 80) {
  if (!simState || simState.done) {
    return { done: true, points: null, progress: 100 };
  }

  const { simDt, motor, mech, target } = simState;
  const points = {
    position: controllers.map(() => []),
    velocity: controllers.map(() => []),
    accel: controllers.map(() => []),
    current: controllers.map(() => []),
    voltage: controllers.map(() => [])
  };

  for (let step = 0; step < batchSize; step++) {
    let allDone = true;

    controllers.forEach((ctrl, index) => {
      const state = simState.states[index];
      if (state.t >= simState.totalTime) return;

      allDone = false;

      let stepDt = ctrl.dt;
      if (ctrl.variation > 0) {
        const randomVariation = normalRandom(0, ctrl.variation / 2000);
        const maxVar = (3 * ctrl.variation) / 1000;
        const clampedVariation = Math.max(-maxVar, Math.min(maxVar, randomVariation));
        stepDt = Math.max(0.001, ctrl.dt + clampedVariation);
      }

      const v = pidCalc(ctrl, state, mech, target, stepDt, motor.maxV);

      const numSubSteps = Math.ceil(stepDt / simDt);
      const actualSimDt = stepDt / numSubSteps;
      let totalCurrent = 0;
      const prevVel = state.vel;

      for (let subStep = 0; subStep < numSubSteps; subStep++) {
        const torqueMotor = motor.getTorque(v, state.vel);
        const effectiveRatio = mech.getEffectiveRatio();
        const gravityTorque = mech.getGravityTorque();
        const counterSpringTorque = mech.getCounterSpringTorque();
        const torqueLoad = torqueMotor / effectiveRatio + gravityTorque + counterSpringTorque;
        const netT = torqueLoad - Math.sign(state.vel) * mech.kineticFric;
        const acc = netT / mech.getInertia();

        totalCurrent += motor.getCurrent(v, state.vel);
        state.pos += actualSimDt * state.vel;
        state.vel += acc * actualSimDt;
      }

      state.t += stepDt;

      const avgCurrent = totalCurrent / numSubSteps;
      const acceleration = (state.vel - prevVel) / stepDt;
      const position = +mech.getLinearPos(state.pos).toFixed(3);

      points.position[index].push({ x: +state.t.toFixed(4), y: position });
      points.velocity[index].push({ x: +state.t.toFixed(4), y: +state.vel.toFixed(3) });
      points.accel[index].push({ x: +state.t.toFixed(4), y: +acceleration.toFixed(3) });
      points.current[index].push({ x: +state.t.toFixed(4), y: +avgCurrent.toFixed(3) });
      points.voltage[index].push({ x: +state.t.toFixed(4), y: +v.toFixed(3) });
    });

    if (allDone) {
      simState.done = true;
      break;
    }
  }

  const currentTime = Math.max(...simState.states.map(s => s.t));
  const progress = Math.round((currentTime / simState.totalTime) * 100);

  return { done: simState.done, points, progress };
}
