// Simulator loader/entry point
// Loads the selected simulator module (default: linear)

function loadSimulator(simName) {
  if (simName === 'linear') {
    import('./simulators/linear.js')
      .then(mod => {
        if (mod && typeof mod.initLinearSimulator === 'function') {
          mod.initLinearSimulator();
        }
      })
      .catch(err => {
        document.body.innerHTML = '<div style="color:red;padding:2rem;">Failed to load simulator: ' + err + '</div>';
      });
  }
}

// Default: load linear simulator
window.addEventListener('DOMContentLoaded', () => {
  loadSimulator('linear');
});
<<<<<<< Updated upstream
=======
      targetDataset.data.push({ x: 0, y: target });
    
  

    for (let step = 0; step < batchSize; step++) {
    let allDone = true;

    controllers.forEach((ctrl, index) => {
      const state = simState.states[index];
      if (state.t >= simState.totalTime) return;

      allDone = false;

      // Apply random timing variation if ctrl.variation > 0
      let stepDt = ctrl.dt;
      if (ctrl.variation > 0) {
        const randomVariation = normalRandom(0, ctrl.variation / 2000); // variation is in ms, so /1000 /2
        // Clamp to ±3x the variation
        const clampedVariation = Math.max(-3 * ctrl.variation / 1000, Math.min(3 * ctrl.variation / 1000, randomVariation));
>>>>>>> Stashed changes
        stepDt = Math.max(0.001, ctrl.dt + clampedVariation);
      }

      const v = pidCalc(ctrl, state, mech, target, stepDt);

      // Debug: log key parameters for first controller, first step

      // Simulate physics in smaller timesteps
      const numSubSteps = Math.ceil(stepDt / simDt);
      const actualSimDt = stepDt / numSubSteps;
      let totalCurrent = 0;
      let prevVel = state.vel;

      for (let subStep = 0; subStep < numSubSteps; subStep++) {
        const torqueMotor = motor.getTorque(v, state.vel);
        const effectiveRatio = mech.getEffectiveRatio();
        const gravityTorque = mech.getGravityTorque();
        const counterSpringTorque = mech.getCounterSpringTorque();
        const torqueLoad = torqueMotor / effectiveRatio + gravityTorque + counterSpringTorque;
        const netT = torqueLoad - Math.sign(state.vel) * mech.kineticFric;
        const acc = netT / mech.getInertia();

        // Debug: log torques and acc for first controller, first step, first 5 substeps

        // Accumulate current
        totalCurrent += motor.getCurrent(v, state.vel);

        state.pos += actualSimDt * state.vel;
        state.vel += acc * actualSimDt;

        // Debug: log first controller's values
      }

      state.t += stepDt;

      // Calculate average current and acceleration
      const avgCurrent = totalCurrent / numSubSteps;
      const acceleration = (state.vel - prevVel) / stepDt;

      const position = +mech.getLinearPos(state.pos).toFixed(3);

      charts.positionChart.data.datasets[index].data.push({ x: +state.t.toFixed(4), y: position });
      charts.velocityChart.data.datasets[index].data.push({ x: +state.t.toFixed(4), y: +state.vel.toFixed(3) });
      charts.accelChart.data.datasets[index].data.push({ x: +state.t.toFixed(4), y: +acceleration.toFixed(3) });
      charts.currentChart.data.datasets[index].data.push({ x: +state.t.toFixed(4), y: +avgCurrent.toFixed(3) });
      charts.voltageChart.data.datasets[index].data.push({ x: +state.t.toFixed(4), y: +v.toFixed(3) });
    });

    const currentTime = Math.max(...simState.states.map(s => s.t));
    const targetDataset = charts.positionChart.data.datasets[controllers.length];
    if (targetDataset && targetDataset.data.length === 0 || currentTime - targetDataset.data[targetDataset.data.length - 1].x > (controllers[0]?.dt || 0.026) * 5) {
      targetDataset.data.push({ x: +currentTime.toFixed(4), y: target });
    }

    if (allDone) {
      simState.done = true;
      break;
    }
  }

  // Update all charts
  Object.values(charts).forEach(chart => {
    if (chart) {
      autoScaleY(chart);
      chart.update('none');
    }
  });

  const progress = Math.round((Math.max(...simState.states.map(s => s.t)) / simState.totalTime) * 100);
  getEl('status').textContent = simState.done ? 'Simulation complete.' : `Simulating... ${progress}%`;

  if (!simState.done) {
    animFrame = requestAnimationFrame(() => stepBatch(300));
  } else {
    stopRun();
  }


function autoScaleY(chart) {
  if (!chart) return;
  const target = getVal('targetInput');
  const allValues = [target];
  chart.data.datasets.slice(0, controllers.length).forEach(dataset => {
    dataset.data.forEach(point => allValues.push(point.y));
  });
  const minY = Math.min(...allValues);
  const maxY = Math.max(...allValues);
  const padding = Math.max((maxY - minY) * 0.15, 2);
  chart.options.scales.y.min = +(minY - padding).toFixed(1);
  chart.options.scales.y.max = +(maxY + padding).toFixed(1);
}

function updateTargetLine() {
  if (!charts.positionChart) return;
  const target = getVal('targetInput');
  const total = getVal('simTime');
  const targetDataset = charts.positionChart.data.datasets[controllers.length];
  if (!targetDataset) return;
  targetDataset.data = [{ x: 0, y: target }, { x: total, y: target }];
  autoScaleY(charts.positionChart);
  charts.positionChart.update('none');
}

function toggleRun() {
  if (isRunning) {
    stopRun();
    return;
  }

  if (controllers.length === 0) {
    getEl('status').textContent = 'Add at least one PID controller first.';
    return;
  }

  isRunning = true;
  getEl('runBtn').textContent = '⏹ Stop';
  getEl('runBtn').classList.add('running');

  // Always rebuild chart to match controllers and clear all data
  rebuildChart();
  // Clear all chart data
  Object.values(charts).forEach(chart => {
    if (!chart) return;
    chart.data.datasets.forEach(ds => ds.data = []);
    chart.update('none');
  });

  simState = initSim();

  const target = getVal('targetInput');
  const total = getVal('simTime');
  const targetDataset = charts.positionChart.data.datasets[controllers.length];
  if (targetDataset) {
    targetDataset.data = [{ x: 0, y: target }, { x: total, y: target }];
  }

  // Hide placeholder, show results
  getEl('placeholder').style.display = 'none';
  getEl('results').style.display = 'flex';

  animFrame = requestAnimationFrame(() => stepBatch(300));
}

function stopRun() {
  isRunning = false;
  if (animFrame) {
    cancelAnimationFrame(animFrame);
  }
  getEl('runBtn').textContent = '▶ Run Simulation';
  getEl('runBtn').classList.remove('running');
}

function toggleSection(sectionId) {
  const body = getEl(sectionId + '-body');
  const arrow = getEl(sectionId + '-arrow');
  if (body && arrow) {
    const isOpen = body.classList.contains('collapsed');
    if (isOpen) {
      body.classList.remove('collapsed');
      arrow.classList.add('open');
      arrow.textContent = '▶';
    } else {
      body.classList.add('collapsed');
      arrow.classList.remove('open');
      arrow.textContent = '▶';
    }
  }
}

function toggleDarkMode() {
  document.body.classList.toggle('light-mode');
  updateDarkModeIcon();
}

function setupEvents() {
  getEl('addCtrlButton').addEventListener('click', () => addController());
  getEl('runBtn').addEventListener('click', () => toggleRun());
  getEl('targetInput').addEventListener('input', updateTargetLine);
  getEl('simTime').addEventListener('input', updateTargetLine);
  
  // Cascade rigging checkbox
  getEl('cascadeRigging').addEventListener('change', (e) => {
    const stagesField = getEl('stagesField');
    if (e.target.checked) {
      stagesField.style.display = 'block';
    } else {
      stagesField.style.display = 'none';
    }
  });
  
  // Track mouse position for crosshair
  getEl('cascadeRigging').addEventListener('change', (e) => {
    const stagesField = getEl('stagesField');
    if (e.target.checked) {
      stagesField.style.display = 'block';
    } else {
      stagesField.style.display = 'none';
    }
  });
  
  // Track mouse position for crosshair
  document.addEventListener('mousemove', (e) => {
    mousePos = { x: e.clientX, y: e.clientY };
  });
}

function updateDarkModeIcon() {
  const icon = getEl('darkModeToggle');
  if (icon) {
    icon.textContent = document.body.classList.contains('light-mode') ? '☀️' : '🌙';
  }
}

window.addEventListener('DOMContentLoaded', () => {
  setupEvents();
  updateDarkModeIcon();
  addController(0.014, 0, 0.00082, 0.1, 'PID 1');
  rebuildChart();
});