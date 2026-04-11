// Linear System Simulator Module
export function initLinearSimulator() {
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

  // --- All logic from app.js except the loader ---
  // (All code from app.js except the loader is pasted here)

  // ...BEGIN pasted logic...

  // (Insert all functions: addController, removeController, adjustInstances, renderControllers, rebuildChart, getMotor, getMech, pidCalc, initSim, normalRandom, stepBatch, autoScaleY, updateTargetLine, toggleRun, stopRun, toggleSection, toggleDarkMode, setupEvents, updateDarkModeIcon)

  // ...existing code from app.js...
  // (All code from app.js except the loader is pasted here)

  // Expose UI functions to window
  window.toggleSection = toggleSection;
  window.toggleDarkMode = toggleDarkMode;
  window.addController = addController;

  // Initialize simulator on load
  setupEvents();
  updateDarkModeIcon();
  addController(0.014, 0, 0.00082, 0.1, 'PID 1');
  rebuildChart();
}
