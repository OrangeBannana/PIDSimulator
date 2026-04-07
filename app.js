const COLORS = ['#3B82F6', '#EF4444', '#10B981', '#F59E0B', '#8B5CF6', '#EC4899', '#14B8A6', '#F97316'];
let controllers = [];
let chart = null;
let animFrame = null;
let isRunning = false;
let simState = null;
let mousePos = { x: null, y: null };

const getEl = id => document.getElementById(id);
const getVal = id => parseFloat(getEl(id).value) || 0;

function addController(p = 0.014, i = 0, d = 0.00082, f = 0.1, name = '') {
  const id = Date.now();
  const color = COLORS[controllers.length % COLORS.length];
  const label = name || `PID ${controllers.length + 1}`;
  controllers.push({ id, color, name: label, p, i, d, f, integral: 0, prevErr: 0 });
  renderControllers();
  rebuildChart();
}

function removeController(id) {
  controllers = controllers.filter(c => c.id !== id);
  renderControllers();
  rebuildChart();
}

function renderControllers() {
  const list = getEl('ctrlList');
  list.innerHTML = '';

  controllers.forEach((ctrl, index) => {
    ctrl.color = COLORS[index % COLORS.length];

    const card = document.createElement('div');
    card.className = 'ctrl-card';

    const header = document.createElement('div');
    header.className = 'ctrl-header';

    const dot = document.createElement('div');
    dot.className = 'ctrl-dot';
    dot.style.background = ctrl.color;

    const nameInput = document.createElement('input');
    nameInput.className = 'ctrl-name-input';
    nameInput.value = ctrl.name;
    nameInput.addEventListener('change', () => {
      ctrl.name = nameInput.value || ctrl.name;
      rebuildChart();
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
      input.value = ctrl[field.key];
      input.addEventListener('change', () => {
        ctrl[field.key] = parseFloat(input.value) || 0;
      });

      container.append(label, input);
      gains.appendChild(container);
    });

    const removeBtn = document.createElement('button');
    removeBtn.className = 'remove-btn';
    removeBtn.type = 'button';
    removeBtn.textContent = '✕';
    removeBtn.addEventListener('click', () => removeController(ctrl.id));

    card.append(gains, removeBtn);
    list.appendChild(card);
  });

  renderLegend();
}

function renderLegend() {
  const legend = getEl('legend');
  legend.innerHTML = '';

  controllers.forEach(ctrl => {
    const item = document.createElement('div');
    item.className = 'legend-item';

    const dot = document.createElement('div');
    dot.className = 'legend-dot';
    dot.style.background = ctrl.color;

    const label = document.createElement('span');
    label.textContent = ctrl.name;

    item.append(dot, label);
    legend.appendChild(item);
  });

  const targetItem = document.createElement('div');
  targetItem.className = 'legend-item';
  targetItem.innerHTML = '<div class="legend-dot" style="background:#888;opacity:0.6;border-top:2px dashed #888;height:0;margin-top:1px"></div><span>target</span>';
  legend.appendChild(targetItem);
}

function rebuildChart() {
  const ctx = getEl('chart');
  if (chart) {
    chart.destroy();
    chart = null;
  }

  const datasets = controllers.map(ctrl => ({
    label: ctrl.name,
    data: [],
    borderColor: ctrl.color,
    backgroundColor: 'transparent',
    borderWidth: 2,
    pointRadius: 0,
    tension: 0.2
  }));

  datasets.push({
    label: 'target',
    data: [],
    borderColor: '#888',
    borderDash: [5, 4],
    borderWidth: 1.5,
    pointRadius: 0,
    tension: 0
  });

  chart = new Chart(ctx, {
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
              return `${context.dataset.label}: ${context.raw.y.toFixed(2)} cm`;
            }
          }
        },
        verticalLineCursor: {
          id: 'verticalLineCursor'
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
          title: { display: true, text: 'position (cm)', font: { size: 11 }, color: '#888' },
          ticks: { font: { size: 11 }, color: '#888', callback: v => v.toFixed(1) },
          grid: { color: 'rgba(128,128,128,0.12)' }
        }
      }
    },
    plugins: [{
      id: 'verticalLineCursor',
      afterDatasetsDraw(chart) {
        if (!chart.tooltip?.active?.length) return;
        
        const { ctx, chartArea: { left, top, width, height }, scales: { x } } = chart;
        const canvasRect = ctx.canvas.getBoundingClientRect();
        
        const isInChart = mousePos.x >= left + canvasRect.left && 
                         mousePos.x <= left + width + canvasRect.left &&
                         mousePos.y >= top + canvasRect.top && 
                         mousePos.y <= top + height + canvasRect.top;
        
        if (isInChart && chart.tooltip.active[0]) {
          const xValue = chart.tooltip.active[0].raw.x;
          const xPixel = left + ((xValue - x.min) / (x.max - x.min)) * width;
          
          ctx.save();
          ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
          ctx.setLineDash([5, 5]);
          ctx.lineWidth = 1.5;
          ctx.beginPath();
          ctx.moveTo(xPixel, top);
          ctx.lineTo(xPixel, top + height);
          ctx.stroke();
          ctx.restore();
        }
      }
    }]
  });

  renderLegend();
}

function getMotor() {
  const maxV = getVal('maxV');
  const freeRPM = getVal('freeRPM');
  const stallCurrent = getVal('stallCurrent');
  const kt = maxV / ((freeRPM * 2 * Math.PI) / 60);
  const R = maxV / stallCurrent;
  const count = getVal('motorCount');
  return {
    kt,
    R,
    count,
    getTorque(v, omega) {
      return (kt * (v - kt * omega) / R) * count;
    }
  };
}

function getMech() {
  const mass = getVal('mass');
  const ratio = getVal('ratio');
  const spoolR = getVal('spoolR');
  const fricCoeff = getVal('fricCoeff');
  const g = -9.81;
  const kineticFric = fricCoeff * Math.abs(g) * mass * spoolR;
  return {
    mass,
    ratio,
    spoolR,
    g,
    kineticFric,
    getInertia() {
      return mass * spoolR * spoolR;
    },
    getGravityTorque() {
      return g * mass * spoolR;
    },
    getLinearPos(angPos) {
      return ((angPos / (2 * Math.PI)) * ratio) * 2 * Math.PI * spoolR * 100;
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
  const dt = getVal('dt') / 1000;
  const motor = getMotor();
  const mech = getMech();
  const target = getVal('targetInput');
  const totalTime = getVal('simTime');
  const loopVariance = getVal('loopVariance') / 1000; // Convert ms to seconds
  const states = controllers.map(() => ({ pos: 0, vel: 0, t: 0 }));
  controllers.forEach(ctrl => {
    ctrl.integral = 0;
    ctrl.prevErr = 0;
    ctrl.ki = ctrl.i;
  });
  return { dt, motor, mech, target, totalTime, states, done: false, loopVariance };
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

  const { dt, motor, mech, target, loopVariance } = simState;

  // Add initial data points at t=0 if this is the first call
  if (simState.states[0].t === 0 && chart.data.datasets[0].data.length === 0) {
    controllers.forEach((ctrl, index) => {
      chart.data.datasets[index].data.push({ x: 0, y: 0 });
    });
    const targetDataset = chart.data.datasets[controllers.length];
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

      // Apply random timing variation if loopVariance > 0
      let stepDt = dt;
      if (loopVariance > 0) {
        const randomVariation = normalRandom(0, loopVariance / 2);
        // Clamp to ±2x the loopVariance
        const clampedVariation = Math.max(-2 * loopVariance, Math.min(2 * loopVariance, randomVariation));
        stepDt = Math.max(0.001, dt + clampedVariation); // Ensure dt doesn't go negative or too small
      }

      const v = pidCalc(ctrl, state, mech, target, stepDt);
      const torqueMotor = motor.getTorque(v, state.vel);
      const torqueLoad = torqueMotor / mech.ratio + mech.getGravityTorque();
      const netT = torqueLoad - Math.sign(state.vel) * mech.kineticFric;
      const acc = netT / mech.getInertia();

      state.pos += stepDt * state.vel;
      state.vel += acc * stepDt;
      state.t += stepDt;

      chart.data.datasets[index].data.push({ x: +state.t.toFixed(4), y: +mech.getLinearPos(state.pos).toFixed(3) });
    });

    const currentTime = Math.max(...simState.states.map(s => s.t));
    const targetDataset = chart.data.datasets[controllers.length].data;
    if (targetDataset.length === 0 || currentTime - targetDataset[targetDataset.length - 1].x > dt * 5) {
      targetDataset.push({ x: +currentTime.toFixed(4), y: target });
    }

    if (allDone) {
      simState.done = true;
      break;
    }
  }

  autoScaleY();
  chart.update('none');
  const progress = Math.round((Math.max(...simState.states.map(s => s.t)) / simState.totalTime) * 100);
  getEl('status').textContent = simState.done ? 'Simulation complete.' : `Simulating... ${progress}%`;

  if (!simState.done) {
    animFrame = requestAnimationFrame(() => stepBatch(batchSize));
  } else {
    stopRun();
  }
}

function autoScaleY() {
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
  if (!chart) return;
  const target = getVal('targetInput');
  const total = getVal('simTime');
  const targetDataset = chart.data.datasets[controllers.length];
  if (!targetDataset) return;
  targetDataset.data = [{ x: 0, y: target }, { x: total, y: target }];
  chart.options.scales.x.min = 0;
  chart.options.scales.x.max = total;
  autoScaleY();
  chart.update('none');
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
  getEl('runBtn').textContent = 'Stop';
  getEl('runBtn').classList.add('running');

  rebuildChart();
  simState = initSim();

  const target = getVal('targetInput');
  const total = getVal('simTime');
  chart.data.datasets[controllers.length].data = [{ x: 0, y: target }, { x: total, y: target }];
  chart.options.scales.x.min = 0;
  chart.options.scales.x.max = total;
  animFrame = requestAnimationFrame(() => stepBatch(80));
}

function stopRun() {
  isRunning = false;
  if (animFrame) {
    cancelAnimationFrame(animFrame);
  }
  getEl('runBtn').textContent = 'Run simulation';
  getEl('runBtn').classList.remove('running');
}

function setupEvents() {
  getEl('addCtrlButton').addEventListener('click', () => addController());
  getEl('runBtn').addEventListener('click', () => toggleRun());
  getEl('targetInput').addEventListener('input', updateTargetLine);
  getEl('simTime').addEventListener('input', updateTargetLine);
  
  // Dark mode toggle
  const darkModeToggle = getEl('darkModeToggle');
  if (darkModeToggle) {
    darkModeToggle.addEventListener('click', () => {
      document.body.classList.toggle('dark-mode');
      localStorage.setItem('darkMode', document.body.classList.contains('dark-mode'));
      updateDarkModeIcon();
    });
  }
  
  // Collapse buttons
  document.querySelectorAll('.collapse-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
      e.preventDefault();
      const sectionId = btn.getAttribute('data-section');
      const content = getEl(sectionId);
      if (content) {
        content.classList.toggle('collapsed');
        btn.textContent = content.classList.contains('collapsed') ? '+' : '−';
      }
    });
  });
  
  // Track mouse position for crosshair
  document.addEventListener('mousemove', (e) => {
    mousePos = { x: e.clientX, y: e.clientY };
  });
}

function updateDarkModeIcon() {
  const icon = getEl('darkModeToggle');
  if (icon) {
    icon.textContent = document.body.classList.contains('dark-mode') ? '☀️' : '🌙';
  }
}

window.addEventListener('DOMContentLoaded', () => {
  // Restore dark mode preference
  const darkModePref = localStorage.getItem('darkMode');
  if (darkModePref !== null) {
    if (darkModePref === 'true') {
      document.body.classList.add('dark-mode');
    } else {
      document.body.classList.remove('dark-mode');
    }
  }
  
  updateDarkModeIcon();
  setupEvents();
  addController(0.014, 0, 0.00082, 0.1, 'PID 1');
  rebuildChart();
});