const COLORS = ['#3B82F6', '#EF4444', '#10B981', '#F59E0B', '#8B5CF6', '#EC4899', '#14B8A6', '#F97316'];
let controllers = [];
let chart = null;
let animFrame = null;
let isRunning = false;
let simState = null;

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
      { label: 'Kp', key: 'p', step: 0.001 },
      { label: 'Ki', key: 'i', step: 0.0001 },
      { label: 'Kd', key: 'd', step: 0.0001 },
      { label: 'Kf', key: 'f', step: 0.01 }
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
      plugins: { legend: { display: false } },
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
    }
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

function pidCalc(ctrl, state, mech, targetCm) {
  const dt = getVal('dt') / 1000;
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
  const states = controllers.map(() => ({ pos: 0, vel: 0, t: 0 }));
  controllers.forEach(ctrl => {
    ctrl.integral = 0;
    ctrl.prevErr = 0;
    ctrl.ki = ctrl.i;
  });
  return { dt, motor, mech, target, totalTime, states, done: false };
}

function stepBatch(batchSize = 80) {
  if (!simState || simState.done) return;

  const { dt, motor, mech, target } = simState;

  for (let step = 0; step < batchSize; step++) {
    let allDone = true;

    controllers.forEach((ctrl, index) => {
      const state = simState.states[index];
      if (state.t >= simState.totalTime) return;

      allDone = false;
      const v = pidCalc(ctrl, state, mech, target);
      const torqueMotor = motor.getTorque(v, state.vel);
      const torqueLoad = torqueMotor / mech.ratio + mech.getGravityTorque();
      const netT = torqueLoad - Math.sign(state.vel) * mech.kineticFric;
      const acc = netT / mech.getInertia();

      state.pos += dt * state.vel;
      state.vel += acc * dt;
      state.t += dt;

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
  addController(0.014, 0, 0.00082, 0.1, 'Tuned PID');
  rebuildChart();
});