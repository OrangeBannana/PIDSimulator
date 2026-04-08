const COLORS = ['#3B82F6', '#EF4444', '#10B981', '#F59E0B', '#8B5CF6', '#EC4899', '#14B8A6', '#F97316'];
let controllers = [];
let charts = {};
let animFrame = null;
let isRunning = false;
let simState = null;
let mousePos = { x: null, y: null };

const getEl = id => document.getElementById(id);
const getVal = id => parseFloat(getEl(id).value) || 0;

let pidCounter = 0;

let globalInstanceUID = 1;
function addController(p = 0.014, i = 0, d = 0.00082, f = 0.1, name = '') {
  pidCounter++;
  const baseName = name || `PID ${pidCounter}`;
  const baseDt = 0.026; // 26ms default
  const idx = controllers.filter(c => c.baseName === baseName).length;
  const color = COLORS[idx % COLORS.length];
  const borderDash = idx === 0 ? [] : [5 + idx * 2, 5];
  const label = `${baseName}.1`;
  controllers.push({
    id: Date.now(),
    uid: globalInstanceUID++,
    color,
    borderDash,
    name: label,
    p, i, d, f, dt: baseDt, integral: 0, prevErr: 0, baseName, variation: 0
  });
  renderControllers();
}

function removeController(identifier) {
  controllers = controllers.filter(c => c.id !== identifier && c.baseName !== identifier);
  renderControllers();
}

function adjustInstances(baseName, newVal) {
    // Prompt user to re-run simulation for new instance count
    getEl('status').textContent = 'Instance count changed. Re-run simulation to update results.';
  const instances = controllers.filter(c => c.baseName === baseName);
  const currentCount = instances.length;
  const firstCtrl = instances[0];

  if (newVal > currentCount) {
    // Add new instances with persistent uid, color, dash
    for (let i = currentCount; i < newVal; i++) {
      const idx = i;
      const newCtrl = {
        id: Date.now() + i,
        uid: globalInstanceUID++,
        color: COLORS[idx % COLORS.length],
        borderDash: idx === 0 ? [] : [5 + idx * 2, 5],
        name: `${baseName}.${i + 1}`,
        p: firstCtrl.p,
        i: firstCtrl.i,
        d: firstCtrl.d,
        f: firstCtrl.f,
        dt: firstCtrl.dt,
        integral: 0,
        prevErr: 0,
        baseName,
        variation: 0
      };
      controllers.push(newCtrl);
    }
  } else if (newVal < currentCount) {
    // Remove extra instances
    const toRemove = instances.slice(newVal);
    controllers = controllers.filter(c => !toRemove.includes(c));
  }

  // Only update the name for each instance, not color or borderDash
  const updatedInstances = controllers.filter(c => c.baseName === baseName);
  updatedInstances.forEach((ctrl, idx) => {
    ctrl.name = `${baseName}.${idx + 1}`;
  });

  renderControllers();
  // Update chart datasets to match controllers for each chart, preserving data where possible
  Object.entries(charts).forEach(([chartId, chart]) => {
    if (!chart) return;
    const isPositionChart = chartId === 'positionChart';
    const datasets = chart.data.datasets;
    if (isPositionChart) {
      // Always keep the target line as the last dataset
      const targetLine = datasets[datasets.length - 1];
      // Save existing controller data and style by uid
      const existingData = {};
      const existingStyle = {};
      for (let i = 0; i < datasets.length - 1; i++) {
        const ds = datasets[i];
        if (ds && ds._uid != null) {
          existingData[ds._uid] = ds.data;
          existingStyle[ds._uid] = {
            borderColor: ds.borderColor,
            borderDash: ds.borderDash
          };
        }
      }
      // Remove all controller datasets (everything except last)
      datasets.length = 1;
      datasets[0] = targetLine;
      // Insert controller datasets before the target line, preserving data and style by uid
      for (let i = 0; i < controllers.length; i++) {
        const ctrl = controllers[i];
        // First instance always solid, others dashed
        const borderDash = i === 0 ? [] : (existingStyle[ctrl.uid]?.borderDash || [5 + i * 2, 5]);
        datasets.splice(i, 0, {
          label: ctrl.name,
          _uid: ctrl.uid,
          data: existingData[ctrl.uid] || [],
          borderColor: existingStyle[ctrl.uid]?.borderColor || ctrl.color,
          backgroundColor: 'transparent',
          borderWidth: 2,
          borderDash,
          pointRadius: 0,
          tension: 0.2
        });
      }
      // Update controller dataset styles
      for (let i = 0; i < controllers.length; i++) {
        const ctrl = controllers[i];
        datasets[i].label = ctrl.name;
        datasets[i].borderColor = datasets[i].borderColor || ctrl.color;
        datasets[i].borderDash = i === 0 ? [] : (datasets[i].borderDash || [5 + i * 2, 5]);
        datasets[i]._uid = ctrl.uid;
      }
    } else {
      // For other charts, just match controller count
      while (datasets.length > controllers.length) {
        datasets.pop();
      }
      for (let i = 0; i < controllers.length; i++) {
        if (!datasets[i]) {
          datasets[i] = {
            label: controllers[i].name,
            data: [],
            borderColor: controllers[i].color,
            backgroundColor: 'transparent',
            borderWidth: 2,
            borderDash: controllers[i].borderDash,
            pointRadius: 0,
            tension: 0.2
          };
        } else {
          datasets[i].label = controllers[i].name;
          datasets[i].borderColor = controllers[i].color;
          datasets[i].borderDash = controllers[i].borderDash;
        }
      }
    }
    chart.update('none');
  });
}

function renderControllers() {
  const list = getEl('ctrlList');
  list.innerHTML = '';

  // Group controllers by baseName
  const pidGroups = {};
  controllers.forEach(ctrl => {
    const baseName = ctrl.baseName;
    if (!pidGroups[baseName]) pidGroups[baseName] = [];
    pidGroups[baseName].push(ctrl);
  });

  Object.keys(pidGroups).forEach(baseName => {
    const instances = pidGroups[baseName];
    const firstCtrl = instances[0];

    const card = document.createElement('div');
    card.className = 'ctrl-card';

    const header = document.createElement('div');
    header.className = 'ctrl-header';

    const dot = document.createElement('div');
    dot.className = 'ctrl-dot';
    dot.style.background = firstCtrl.color;

    const nameInput = document.createElement('input');
    nameInput.className = 'ctrl-name-input';
    nameInput.value = baseName;
    nameInput.addEventListener('change', () => {
      const newBase = nameInput.value || baseName;
      instances.forEach((ctrl, idx) => {
        ctrl.baseName = newBase;
        ctrl.name = `${newBase}.${idx + 1}`;
      });
      // No rebuildChart here; chart datasets will be updated by adjustInstances if needed
    });

    header.append(dot, nameInput);
    card.appendChild(header);

    const gains = document.createElement('div');
    gains.className = 'ctrl-gains';

    const fields = [
      { label: 'Kp, V/N', key: 'p', step: 0.001 },
      { label: 'Ki, V/(N • s)', key: 'i', step: 0.0001 },
      { label: 'Kd, V • s/N', key: 'd', step: 0.0001 },
      { label: 'Kf, % max V', key: 'f', step: 0.01 }
    ];

    fields.forEach(field => {
      const container = document.createElement('div');
      container.className = 'gain-field';

      const label = document.createElement('label');
      label.textContent = field.label;

      const input = document.createElement('input');
      input.type = 'number';
      input.step = field.step;
      input.value = firstCtrl[field.key];
      input.addEventListener('change', () => {
        instances.forEach(ctrl => ctrl[field.key] = parseFloat(input.value) || 0);
      });

      container.append(label, input);
      gains.appendChild(container);
    });

    const dtContainer = document.createElement('div');
    dtContainer.className = 'gain-field';
    dtContainer.style.display = 'flex';
    dtContainer.style.alignItems = 'center';
    dtContainer.style.gap = '8px';
    dtContainer.innerHTML = `
      <label style="width: 140px;">Looptime (ms):</label>
      <input type="number" step="0.1" value="${(firstCtrl.dt * 1000).toFixed(1)}" style="flex: 1;" />
    `;
    const dtInput = dtContainer.querySelector('input');
    dtInput.addEventListener('change', () => {
      const value = parseFloat(dtInput.value) / 1000;
      instances.forEach(ctrl => ctrl.dt = value);
    });
    gains.appendChild(dtContainer);

    const variationContainer = document.createElement('div');
    variationContainer.className = 'gain-field';
    variationContainer.style.display = 'flex';
    variationContainer.style.alignItems = 'center';
    variationContainer.style.gap = '8px';
    variationContainer.innerHTML = `
      <label style="width: 140px;">Variation (ms):</label>
      <input type="number" step="1" value="${firstCtrl.variation || 0}" style="flex: 1;" />
    `;
    const variationInput = variationContainer.querySelector('input');
    variationInput.addEventListener('change', () => {
      const value = parseFloat(variationInput.value) || 0;
      instances.forEach(ctrl => ctrl.variation = value);
    });
    gains.appendChild(variationContainer);

    const sliderRow = document.createElement('div');
    sliderRow.style.display = 'flex';
    sliderRow.style.alignItems = 'center';
    sliderRow.style.gap = '8px';
    sliderRow.style.width = '100%';
    sliderRow.style.boxSizing = 'border-box';

    const sliderLabel = document.createElement('label');
    sliderLabel.textContent = `Instances: `;
    sliderLabel.style.flexShrink = '0';
    sliderLabel.style.minWidth = '85px';
    
    const instancesValue = document.createElement('span');
    instancesValue.className = 'instances-value';
    instancesValue.textContent = instances.length;
    sliderLabel.appendChild(instancesValue);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'instances-slider';
    slider.min = '1';
    slider.max = '10';
    slider.value = `${instances.length}`;
    slider.step = '1';
    slider.style.flex = '1';
    slider.style.minWidth = '100px';
    slider.dataset.basename = baseName;
    slider.addEventListener('input', (e) => {
      const newVal = parseInt(e.target.value, 10);
      instancesValue.textContent = newVal;
      adjustInstances(baseName, newVal);
    });

    const removeBtn = document.createElement('button');
    removeBtn.className = 'remove-btn';
    removeBtn.type = 'button';
    removeBtn.textContent = '✕';
    removeBtn.style.flexShrink = '0';
    removeBtn.addEventListener('click', () => removeController(baseName));

    sliderRow.append(sliderLabel, slider, removeBtn);
    gains.appendChild(sliderRow);

    card.appendChild(header);
    card.appendChild(gains);
    list.appendChild(card);
  });
}

function rebuildChart() {
  // Destroy existing charts
  Object.values(charts).forEach(chart => {
    if (chart) chart.destroy();
  });
  charts = {};

  const chartConfigs = [
    { id: 'positionChart', title: 'Position vs Time', color: '#00d4ff', unit: 'cm' },
    { id: 'velocityChart', title: 'Velocity vs Time', color: '#7fff6b', unit: 'cm/s' },
    { id: 'accelChart', title: 'Acceleration vs Time', color: '#ff6b35', unit: 'cm/s²' },
    { id: 'currentChart', title: 'Current vs Time', color: '#ffcc00', unit: 'A' },
    { id: 'voltageChart', title: 'Applied Voltage vs Time', color: '#c084fc', unit: 'V' }
  ];

  chartConfigs.forEach(config => {
    const ctx = getEl(config.id);
    if (!ctx) return;

    const datasets = controllers.map((ctrl, index) => ({
      label: ctrl.name,
      data: [],
      borderColor: ctrl.color,
      backgroundColor: 'transparent',
      borderWidth: 2,
      borderDash: ctrl.borderDash,
      pointRadius: 0,
      tension: 0.2
    }));

    // Add target line for position chart
    if (config.id === 'positionChart') {
      datasets.push({
        label: 'target',
        data: [],
        borderColor: '#888',
        borderDash: [5, 4],
        borderWidth: 1.5,
        pointRadius: 0,
        tension: 0
      });
    }

    charts[config.id] = new Chart(ctx, {
      type: 'line',
      data: { datasets },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: {
          mode: 'index',
          intersect: false
        },
        plugins: {
          legend: { display: false },
          tooltip: {
            enabled: true,
            mode: 'index',
            intersect: false,
            backgroundColor: 'rgba(0, 0, 0, 0.8)',
            padding: 12,
            titleFont: { size: 12, weight: 'bold' },
            bodyFont: { size: 11 },
            borderColor: 'rgba(255, 255, 255, 0.2)',
            borderWidth: 1,
            displayColors: true,
            callbacks: {
              title: (context) => {
                if (context.length > 0) {
                  return `Time: ${context[0].raw.x.toFixed(3)}s`;
                }
                return '';
              },
              label: (context) => {
                return `${context.dataset.label}: ${context.raw.y.toFixed(2)} ${config.unit}`;
              }
            }
          }
        },
        scales: {
          x: {
            type: 'linear',
            title: { display: true, text: 'time (s)', font: { size: 11 }, color: '#888' },
            ticks: { font: { size: 11 }, color: '#888', maxTicksLimit: 10, callback: v => v.toFixed(2) },
            grid: { color: 'rgba(128,128,128,0.12)' }
          },
          y: {
            title: { display: true, text: `${config.title.split(' ')[0]} (${config.unit})`, font: { size: 11 }, color: '#888' },
            ticks: { font: { size: 11 }, color: '#888', callback: v => v.toFixed(1) },
            grid: { color: 'rgba(128,128,128,0.12)' }
          }
        }
      }
    });
  });
}

function getMotor() {
  const maxV = getVal('maxV');
  const freeRPM = getVal('freeRPM');
  const stallCurrent = getVal('stallCurrent');
  const stallTorque = getVal('stallTorque');
  const kt = maxV / ((freeRPM * 2 * Math.PI) / 60);
  const R = maxV / stallCurrent;
  const count = getVal('motorCount');
  return {
    kt,
    R,
    count,
    stallTorque,
    getTorque(v, omega) {
      return (kt * (v - kt * omega) / R) * count;
    },
    getCurrent(v, omega) {
      return ((v - kt * omega) / R) * count;
    }
  };
}

function getMech() {
  const mass = getVal('mass');
  const ratio = getVal('ratio');
  const spoolR = getVal('spoolR');
  const fricCoeff = getVal('fricCoeff');
  const cascadeRigging = getEl('cascadeRigging').checked;
  const stages = cascadeRigging ? getVal('stages') : 1;
  const counterSpring = getVal('counterSpring');
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
      // Counterspring force acts vertically, so torque = force * radius
      return counterSpring * spoolR;
    },
    getLinearPos(angPos) {
      const effectiveRatio = this.getEffectiveRatio();
      const distanceMultiplier = this.getDistanceMultiplier();
      // Linear position in cm: angPos [rad] * ratio * stages * spoolR [m] * 100
      return angPos * effectiveRatio * distanceMultiplier * this.spoolR * 100;
    }
  };
}

function pidCalc(ctrl, state, mech, targetCm, dt) {
  const TICKS_PER_REV = 28;
  const targetTicks = TICKS_PER_REV * ((targetCm / (2 * Math.PI * mech.spoolR * 100)) / mech.ratio);
  const currTicks = (state.pos / (2 * Math.PI)) * TICKS_PER_REV;
  const err = targetTicks - currTicks;
  const deriv = (err - ctrl.prevErr) / dt;
  ctrl.integral += err * dt;
  let v = (err * ctrl.p + ctrl.integral * ctrl.ki + deriv * ctrl.d + ctrl.f) * 12;
  v = Math.min(Math.max(v, -12), 12);
  ctrl.prevErr = err;
  return v;
}

function initSim() {
  const simDt = getVal('simDt') / 1000;
  const motor = getMotor();
  const mech = getMech();
  const target = getVal('targetInput');
  const totalTime = getVal('simTime');
  const states = controllers.map(() => ({ pos: 0, vel: 0, t: 0 }));
  controllers.forEach(ctrl => {
    ctrl.integral = 0;
    ctrl.prevErr = 0;
    ctrl.ki = ctrl.i;
  });
  return { simDt, motor, mech, target, totalTime, states, done: false };
}

function normalRandom(mean = 0, std = 1) {
  // Box-Muller transform for normal distribution
  const u1 = Math.random();
  const u2 = Math.random();
  const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  return z0 * std + mean;
}

function stepBatch(batchSize = 80) {
  if (!simState || simState.done) return;

  const { simDt, motor, mech, target } = simState;

  // Add initial data points at t=0 if this is the first call
  if (simState.states[0].t === 0 && charts.positionChart?.data.datasets[0].data.length === 0) {
    controllers.forEach((ctrl, index) => {
      charts.positionChart.data.datasets[index].data.push({ x: 0, y: 0 });
      charts.velocityChart.data.datasets[index].data.push({ x: 0, y: 0 });
      charts.accelChart.data.datasets[index].data.push({ x: 0, y: 0 });
      charts.currentChart.data.datasets[index].data.push({ x: 0, y: 0 });
      charts.voltageChart.data.datasets[index].data.push({ x: 0, y: 0 });
    });
    const targetDataset = charts.positionChart.data.datasets[controllers.length];
    if (targetDataset.data.length === 0) {
      targetDataset.data.push({ x: 0, y: target });
    }
  }

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