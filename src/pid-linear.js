(() => {
  const COLORS = ['#3B82F6', '#EF4444', '#10B981', '#F59E0B', '#8B5CF6', '#EC4899', '#14B8A6', '#F97316'];
  const DEFAULT_CONTROLLER = {
    p: 0.014,
    i: 0,
    d: 0.00082,
    f: 0.1,
    dt: 0.026,
    variation: 0
  };

  class SiteHeader extends HTMLElement {
    connectedCallback() {
      if (this._initialized) return;
      this._initialized = true;

      const title = this.getAttribute('title') || 'PID Simulator';
      const subtitle = this.getAttribute('subtitle') || '';

      const actions = document.createElement('div');
      actions.className = 'header-actions';
      while (this.firstChild) actions.appendChild(this.firstChild);

      const header = document.createElement('header');
      header.className = 'site-header';

      const logo = document.createElement('a');
      logo.className = 'logo logo-link';
      logo.href = 'index.html';
      logo.innerHTML = `
        <div class="logo-badge">FTS</div>
        <div>
          <div class="logo-text">${title}</div>
          <div class="logo-sub">${subtitle}</div>
        </div>
      `;

      header.appendChild(logo);
      header.appendChild(actions);
      this.appendChild(header);

      if (!actions.querySelector('[data-theme-toggle]')) {
        const toggle = document.createElement('button');
        toggle.type = 'button';
        toggle.className = 'btn';
        toggle.setAttribute('data-theme-toggle', 'true');
        actions.appendChild(toggle);
      }

      const toggleButton = actions.querySelector('[data-theme-toggle]');
      if (toggleButton) {
        toggleButton.addEventListener('click', () => this.toggleTheme(toggleButton));
      }

      this.applySavedTheme();
      this.syncToggle(toggleButton);
    }

    applySavedTheme() {
      const stored = localStorage.getItem('pid-theme');
      if (stored === 'light') {
        document.documentElement.classList.add('theme-light');
      } else {
        document.documentElement.classList.remove('theme-light');
      }
    }

    toggleTheme(button) {
      document.documentElement.classList.toggle('theme-light');
      localStorage.setItem(
        'pid-theme',
        document.documentElement.classList.contains('theme-light') ? 'light' : 'dark'
      );
      this.syncToggle(button);
    }

    syncToggle(button) {
      if (!button) return;
      const isLight = document.documentElement.classList.contains('theme-light');
      button.textContent = isLight ? 'Theme: Light' : 'Theme: Dark';
    }
  }

  class SectionPanel extends HTMLElement {
    connectedCallback() {
      if (this._initialized) return;
      this._initialized = true;

      const label = this.getAttribute('label') || 'Section';
      const tag = this.getAttribute('tag') || 'TAG';
      const color = this.getAttribute('color') || '';
      const isOpen = this.hasAttribute('open');

      const body = document.createElement('div');
      body.className = 'section-body';
      if (!isOpen) body.classList.add('collapsed');

      while (this.firstChild) body.appendChild(this.firstChild);

      const header = document.createElement('button');
      header.type = 'button';
      header.className = 'section-header';

      const tagEl = document.createElement('div');
      tagEl.className = `section-tag${color ? ' ' + color : ''}`;
      tagEl.textContent = tag;

      const titleEl = document.createElement('div');
      titleEl.className = 'section-title';
      titleEl.textContent = label;

      const arrow = document.createElement('div');
      arrow.className = `section-arrow${isOpen ? ' open' : ''}`;
      arrow.textContent = '>';

      header.appendChild(tagEl);
      header.appendChild(titleEl);
      header.appendChild(arrow);

      header.addEventListener('click', () => {
        body.classList.toggle('collapsed');
        arrow.classList.toggle('open');
      });

      this.appendChild(header);
      this.appendChild(body);
    }
  }

  if (!customElements.get('site-header')) {
    customElements.define('site-header', SiteHeader);
  }
  if (!customElements.get('section-panel')) {
    customElements.define('section-panel', SectionPanel);
  }

  let controllers = [];
  let charts = {};
  let animFrame = null;
  let isRunning = false;
  let simState = null;
  let pidCounter = 0;
  let globalInstanceUID = 1;

  const getEl = id => document.getElementById(id);
  const getVal = id => parseFloat(getEl(id).value) || 0;

  function createMotor({ maxV, freeRPM, stallCurrent, stallTorque, motorCount }) {
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

  function createMechanism({ mass, ratio, spoolR, fricCoeff, cascadeRigging, stages, counterSpring }) {
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

  function pidCalc(ctrl, state, mech, targetCm, dt, maxV) {
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

  function initSim({ simDt, motor, mech, target, totalTime, controllers }) {
    const states = controllers.map(() => ({ pos: 0, vel: 0, t: 0 }));
    controllers.forEach(ctrl => {
      ctrl.integral = 0;
      ctrl.prevErr = 0;
    });

    return { simDt, motor, mech, target, totalTime, states, done: false };
  }

  function normalRandom(mean = 0, std = 1) {
    const u1 = Math.random();
    const u2 = Math.random();
    const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    return z0 * std + mean;
  }

  function stepBatch(simState, controllers, batchSize = 80) {
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

  function addController(p = DEFAULT_CONTROLLER.p, i = DEFAULT_CONTROLLER.i, d = DEFAULT_CONTROLLER.d, f = DEFAULT_CONTROLLER.f, name = '') {
    pidCounter += 1;
    const baseName = name || `PID ${pidCounter}`;
    const baseDt = DEFAULT_CONTROLLER.dt;
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
      p,
      i,
      d,
      f,
      dt: baseDt,
      integral: 0,
      prevErr: 0,
      baseName,
      variation: 0
    });
    renderControllers();
  }

  function removeController(identifier) {
    controllers = controllers.filter(c => c.id !== identifier && c.baseName !== identifier);
    renderControllers();
  }

  function adjustInstances(baseName, newVal) {
    getEl('status').textContent = 'Instance count changed. Re-run simulation to update results.';
    const instances = controllers.filter(c => c.baseName === baseName);
    const currentCount = instances.length;
    const firstCtrl = instances[0];

    if (newVal > currentCount) {
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
          variation: firstCtrl.variation || 0
        };
        controllers.push(newCtrl);
      }
    } else if (newVal < currentCount) {
      const toRemove = instances.slice(newVal);
      controllers = controllers.filter(c => !toRemove.includes(c));
    }

    const updatedInstances = controllers.filter(c => c.baseName === baseName);
    updatedInstances.forEach((ctrl, idx) => {
      ctrl.name = `${baseName}.${idx + 1}`;
    });

    renderControllers();

    Object.entries(charts).forEach(([chartId, chart]) => {
      if (!chart) return;
      const isPositionChart = chartId === 'positionChart';
      const datasets = chart.data.datasets;
      if (isPositionChart) {
        const targetLine = datasets[datasets.length - 1];
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
        datasets.length = 1;
        datasets[0] = targetLine;
        for (let i = 0; i < controllers.length; i++) {
          const ctrl = controllers[i];
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
        for (let i = 0; i < controllers.length; i++) {
          const ctrl = controllers[i];
          datasets[i].label = ctrl.name;
          datasets[i].borderColor = datasets[i].borderColor || ctrl.color;
          datasets[i].borderDash = i === 0 ? [] : (datasets[i].borderDash || [5 + i * 2, 5]);
          datasets[i]._uid = ctrl.uid;
        }
      } else {
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
      });

      header.append(dot, nameInput);
      card.appendChild(header);

      const gains = document.createElement('div');
      gains.className = 'ctrl-gains';

      const fields = [
        { label: 'Kp, V/N', key: 'p', step: 0.001 },
        { label: 'Ki, V/(N * s)', key: 'i', step: 0.0001 },
        { label: 'Kd, V * s/N', key: 'd', step: 0.0001 },
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
          instances.forEach(ctrl => {
            ctrl[field.key] = parseFloat(input.value) || 0;
          });
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
        instances.forEach(ctrl => {
          ctrl.dt = value;
        });
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
        instances.forEach(ctrl => {
          ctrl.variation = value;
        });
      });
      gains.appendChild(variationContainer);

      const sliderRow = document.createElement('div');
      sliderRow.style.display = 'flex';
      sliderRow.style.alignItems = 'center';
      sliderRow.style.gap = '8px';
      sliderRow.style.width = '100%';
      sliderRow.style.boxSizing = 'border-box';

      const sliderLabel = document.createElement('label');
      sliderLabel.textContent = 'Instances: ';
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
      slider.addEventListener('input', e => {
        const newVal = parseInt(e.target.value, 10);
        instancesValue.textContent = newVal;
        adjustInstances(baseName, newVal);
      });

      const removeBtn = document.createElement('button');
      removeBtn.className = 'remove-btn';
      removeBtn.type = 'button';
      removeBtn.textContent = 'X';
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
    Object.values(charts).forEach(chart => {
      if (chart) chart.destroy();
    });
    charts = {};

    const chartConfigs = [
      { id: 'positionChart', title: 'Position vs Time', color: '#00d4ff', unit: 'cm' },
      { id: 'velocityChart', title: 'Velocity vs Time', color: '#7fff6b', unit: 'cm/s' },
      { id: 'accelChart', title: 'Acceleration vs Time', color: '#ff6b35', unit: 'cm/s^2' },
      { id: 'currentChart', title: 'Current vs Time', color: '#ffcc00', unit: 'A' },
      { id: 'voltageChart', title: 'Applied Voltage vs Time', color: '#c084fc', unit: 'V' }
    ];

    chartConfigs.forEach(config => {
      const ctx = getEl(config.id);
      if (!ctx) return;

      const datasets = controllers.map(ctrl => ({
        label: ctrl.name,
        data: [],
        borderColor: ctrl.color,
        backgroundColor: 'transparent',
        borderWidth: 2,
        borderDash: ctrl.borderDash,
        pointRadius: 0,
        tension: 0.2
      }));

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
                title: context => {
                  if (context.length > 0) {
                    return `Time: ${context[0].raw.x.toFixed(3)}s`;
                  }
                  return '';
                },
                label: context => `${context.dataset.label}: ${context.raw.y.toFixed(2)} ${config.unit}`
              }
            }
          },
          scales: {
            x: {
              type: 'linear',
              title: { display: true, text: 'time (s)', font: { size: 11 }, color: '#888' },
              ticks: { font: { size: 11 }, color: '#888', maxTicksLimit: 10, callback: v => v.toFixed(2) },
              grid: { color: 'rgba(128, 128, 128, 0.12)' }
            },
            y: {
              title: { display: true, text: `${config.title.split(' ')[0]} (${config.unit})`, font: { size: 11 }, color: '#888' },
              ticks: { font: { size: 11 }, color: '#888', callback: v => v.toFixed(1) },
              grid: { color: 'rgba(128, 128, 128, 0.12)' }
            }
          }
        }
      });
    });
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
    getEl('runBtn').textContent = 'Stop';
    getEl('runBtn').classList.add('running');

    rebuildChart();
    Object.values(charts).forEach(chart => {
      if (!chart) return;
      chart.data.datasets.forEach(ds => {
        ds.data = [];
      });
      chart.update('none');
    });

    const simDt = getVal('simDt') / 1000;
    const motor = createMotor({
      maxV: getVal('maxV'),
      freeRPM: getVal('freeRPM'),
      stallCurrent: getVal('stallCurrent'),
      stallTorque: getVal('stallTorque'),
      motorCount: getVal('motorCount')
    });
    const mech = createMechanism({
      mass: getVal('mass'),
      ratio: getVal('ratio'),
      spoolR: getVal('spoolR'),
      fricCoeff: getVal('fricCoeff'),
      cascadeRigging: getEl('cascadeRigging').checked,
      stages: getEl('cascadeRigging').checked ? getVal('stages') : 1,
      counterSpring: getVal('counterSpring')
    });
    const target = getVal('targetInput');
    const totalTime = getVal('simTime');

    simState = initSim({ simDt, motor, mech, target, totalTime, controllers });

    const targetDataset = charts.positionChart.data.datasets[controllers.length];
    if (targetDataset) {
      targetDataset.data = [{ x: 0, y: target }, { x: totalTime, y: target }];
    }

    getEl('placeholder').style.display = 'none';
    getEl('results').style.display = 'flex';

    animFrame = requestAnimationFrame(() => stepLoop());
  }

  function stepLoop() {
    const result = stepBatch(simState, controllers, 300);

    if (result.points) {
      controllers.forEach((ctrl, index) => {
        charts.positionChart.data.datasets[index].data.push(...result.points.position[index]);
        charts.velocityChart.data.datasets[index].data.push(...result.points.velocity[index]);
        charts.accelChart.data.datasets[index].data.push(...result.points.accel[index]);
        charts.currentChart.data.datasets[index].data.push(...result.points.current[index]);
        charts.voltageChart.data.datasets[index].data.push(...result.points.voltage[index]);
      });
    }

    Object.values(charts).forEach(chart => {
      if (chart) {
        autoScaleY(chart);
        chart.update('none');
      }
    });

    getEl('status').textContent = result.done ? 'Simulation complete.' : `Simulating... ${result.progress}%`;

    if (!result.done) {
      animFrame = requestAnimationFrame(() => stepLoop());
    } else {
      stopRun();
    }
  }

  function stopRun() {
    isRunning = false;
    if (animFrame) cancelAnimationFrame(animFrame);
    getEl('runBtn').textContent = 'Run Simulation';
    getEl('runBtn').classList.remove('running');
  }

  function setupEvents() {
    getEl('addCtrlButton').addEventListener('click', () => addController());
    getEl('runBtn').addEventListener('click', () => toggleRun());
    getEl('targetInput').addEventListener('input', updateTargetLine);
    getEl('simTime').addEventListener('input', updateTargetLine);

    getEl('cascadeRigging').addEventListener('change', e => {
      const stagesField = getEl('stagesField');
      stagesField.style.display = e.target.checked ? 'block' : 'none';
    });
  }

  window.addEventListener('DOMContentLoaded', () => {
    setupEvents();
    addController(DEFAULT_CONTROLLER.p, DEFAULT_CONTROLLER.i, DEFAULT_CONTROLLER.d, DEFAULT_CONTROLLER.f, 'PID 1');
    rebuildChart();
    updateTargetLine();
  });
})();
