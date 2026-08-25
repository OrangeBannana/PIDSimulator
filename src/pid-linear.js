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
  let lastTime = 0;
  let lastSeen = {
    position: [],
    velocity: [],
    accel: [],
    current: [],
    voltage: []
  };

  function interpolateSeries(series, timeValue) {
    if (!series || series.length === 0) return null;
    let left = null;
    let right = null;

    for (let i = 0; i < series.length; i++) {
      const point = series[i];
      if (point.x === timeValue) return point.y;
      if (point.x < timeValue) {
        left = point;
      } else if (point.x > timeValue) {
        right = point;
        break;
      }
    }

    if (!left && !right) return null;
    if (!left) return right.y;
    if (!right) return left.y;

    const span = right.x - left.x;
    if (span === 0) return left.y;
    const ratio = (timeValue - left.x) / span;
    return left.y + (right.y - left.y) * ratio;
  }

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
    rebuildChart();
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
    lastTime = 0;
    rebuildChart();
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
        rebuildChart();
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

  function buildSeries(includeTarget) {
    const series = [{ label: 'time' }];

    controllers.forEach((ctrl, idx) => {
      series.push({
        label: ctrl.name,
        stroke: ctrl.color,
        width: 2,
        dash: idx === 0 ? [] : [5 + idx * 2, 5],
        points: { show: false }
      });
    });

    if (includeTarget) {
      series.push({
        label: 'target',
        stroke: '#888',
        width: 1.5,
        dash: [5, 4],
        points: { show: false }
      });
    }

    return series;
  }

  function buildChartOptions(config, series) {
    const styles = getComputedStyle(document.documentElement);
    const axisColor = styles.getPropertyValue('--text-dim').trim() || '#888';
    const gridColor = 'rgba(128, 128, 128, 0.12)';
    const selectFill = 'rgba(0, 212, 255, 0.12)';
    const selectStroke = 'rgba(0, 212, 255, 0.5)';

    return {
      width: 0,
      height: 0,
      series,
      legend: { show: false },
      select: {
        show: true,
        over: true,
        fill: selectFill,
        stroke: selectStroke
      },
      scales: {
        x: { time: false },
        y: { auto: true }
      },
      axes: [
        {
          label: 'time (s)',
          stroke: axisColor,
          grid: { stroke: gridColor },
          values: (u, vals) => vals.map(v => v.toFixed(2))
        },
        {
          label: `${config.label} (${config.unit})`,
          stroke: axisColor,
          grid: { stroke: gridColor },
          values: (u, vals) => vals.map(v => v.toFixed(1))
        }
      ]
    };
  }

  function createChart(config) {
    const host = getEl(config.id);
    if (!host) return null;

    const includeTarget = config.id === 'positionChart';
    const series = buildSeries(includeTarget);
    const data = [[], ...series.slice(1).map(() => [])];
    const opts = buildChartOptions(config, series);
    const plot = new uPlot(opts, data, host);

    const resize = () => {
      const width = host.clientWidth || 0;
      const height = host.clientHeight || 0;
      if (width && height) {
        plot.setSize({ width, height });
      }
    };

    resize();
    const observer = new ResizeObserver(resize);
    observer.observe(host);

    return {
      plot,
      data,
      seriesCount: series.length - 1,
      includeTarget,
      observer
    };
  }

  function rebuildChart() {
    Object.values(charts).forEach(chart => {
      if (!chart) return;
      chart.observer.disconnect();
      chart.plot.destroy();
    });

    charts = {};

    const chartConfigs = [
      { id: 'positionChart', label: 'Position', unit: 'cm' },
      { id: 'velocityChart', label: 'Velocity', unit: 'cm/s' },
      { id: 'accelChart', label: 'Acceleration', unit: 'cm/s^2' },
      { id: 'currentChart', label: 'Current', unit: 'A' },
      { id: 'voltageChart', label: 'Applied Voltage', unit: 'V' }
    ];

    chartConfigs.forEach(config => {
      charts[config.id] = createChart(config);
    });
    lastTime = 0;
    lastSeen = {
      position: controllers.map(() => 0),
      velocity: controllers.map(() => 0),
      accel: controllers.map(() => 0),
      current: controllers.map(() => 0),
      voltage: controllers.map(() => 0)
    };
    updateTargetLine();
  }

  function updateTargetLine() {
    const chart = charts.positionChart;
    if (!chart) return;
    const target = getVal('targetInput');
    const targetSeriesIndex = chart.includeTarget ? chart.data.length - 1 : null;
    if (targetSeriesIndex == null) return;
    const targetSeries = chart.data[targetSeriesIndex];
    for (let i = 0; i < chart.data[0].length; i++) {
      targetSeries[i] = target;
    }
    chart.plot.setData(chart.data);
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

    lastTime = 0;
    lastSeen = {
      position: controllers.map(() => 0),
      velocity: controllers.map(() => 0),
      accel: controllers.map(() => 0),
      current: controllers.map(() => 0),
      voltage: controllers.map(() => 0)
    };

    rebuildChart();

    Object.values(charts).forEach(chart => {
      if (!chart) return;
      chart.data[0].length = 0;
      for (let i = 1; i < chart.data.length; i++) {
        chart.data[i].length = 0;
      }
      chart.plot.setData(chart.data);
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

    updateTargetLine();

    getEl('placeholder').style.display = 'none';
    getEl('results').style.display = 'flex';

    animFrame = requestAnimationFrame(() => stepLoop());
  }

  function appendBatch(points, targetValue) {
    const times = new Set();
    points.position.forEach(series => {
      series.forEach(point => {
        if (point.x > lastTime) times.add(point.x);
      });
    });

    if (times.size === 0) return;
    const timeList = Array.from(times).sort((a, b) => a - b);
    lastTime = timeList[timeList.length - 1];

    const seriesByKey = {
      position: points.position,
      velocity: points.velocity,
      accel: points.accel,
      current: points.current,
      voltage: points.voltage
    };

    Object.entries(charts).forEach(([chartId, chart]) => {
      if (!chart) return;
      timeList.forEach(timeValue => {
        chart.data[0].push(timeValue);

        if (chartId === 'positionChart') {
          controllers.forEach((_, idx) => {
            const value = interpolateSeries(seriesByKey.position[idx], timeValue);
            if (value != null) lastSeen.position[idx] = value;
            chart.data[idx + 1].push(lastSeen.position[idx] ?? null);
          });
          chart.data[chart.data.length - 1].push(targetValue);
        } else if (chartId === 'velocityChart') {
          controllers.forEach((_, idx) => {
            const value = interpolateSeries(seriesByKey.velocity[idx], timeValue);
            if (value != null) lastSeen.velocity[idx] = value;
            chart.data[idx + 1].push(lastSeen.velocity[idx] ?? null);
          });
        } else if (chartId === 'accelChart') {
          controllers.forEach((_, idx) => {
            const value = interpolateSeries(seriesByKey.accel[idx], timeValue);
            if (value != null) lastSeen.accel[idx] = value;
            chart.data[idx + 1].push(lastSeen.accel[idx] ?? null);
          });
        } else if (chartId === 'currentChart') {
          controllers.forEach((_, idx) => {
            const value = interpolateSeries(seriesByKey.current[idx], timeValue);
            if (value != null) lastSeen.current[idx] = value;
            chart.data[idx + 1].push(lastSeen.current[idx] ?? null);
          });
        } else if (chartId === 'voltageChart') {
          controllers.forEach((_, idx) => {
            const value = interpolateSeries(seriesByKey.voltage[idx], timeValue);
            if (value != null) lastSeen.voltage[idx] = value;
            chart.data[idx + 1].push(lastSeen.voltage[idx] ?? null);
          });
        }
      });
    });
  }

  function stepLoop() {
    const result = stepBatch(simState, controllers, 300);

    if (result.points) {
      const targetValue = getVal('targetInput');
      appendBatch(result.points, targetValue);
    }

    Object.values(charts).forEach(chart => {
      if (chart) {
        chart.plot.setData(chart.data);
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

    const chartGrid = getEl('chartsGrid');
    const expandButtons = document.querySelectorAll('.chart-expand-btn');
    const cards = document.querySelectorAll('.chart-card');

    cards.forEach(card => {
      if (card.dataset.defaultLarge === 'true') {
        card.classList.add('is-large');
      }
    });
    expandButtons.forEach(btn => {
      const card = btn.closest('.chart-card');
      const isLarge = card?.classList.contains('is-large');
      btn.textContent = isLarge ? 'Shrink' : 'Expand';
      btn.dataset.expanded = isLarge ? 'true' : 'false';
    });

    expandButtons.forEach(button => {
      button.addEventListener('click', () => {
        const card = button.closest('.chart-card');
        if (!card) return;
        const isLarge = card.classList.contains('is-large');
        card.classList.toggle('is-large', !isLarge);
        button.textContent = isLarge ? 'Expand' : 'Shrink';
        button.dataset.expanded = isLarge ? 'false' : 'true';

        requestAnimationFrame(() => {
          Object.values(charts).forEach(chart => {
            if (!chart) return;
            const host = chart.plot.root;
            const width = host.clientWidth;
            const height = host.clientHeight;
            if (width && height) chart.plot.setSize({ width, height });
          });
        });
      });
    });
  }

  window.addEventListener('DOMContentLoaded', () => {
    setupEvents();
    addController(DEFAULT_CONTROLLER.p, DEFAULT_CONTROLLER.i, DEFAULT_CONTROLLER.d, DEFAULT_CONTROLLER.f, 'PID 1');
    rebuildChart();
    updateTargetLine();
  });
})();
