import { simulators } from './registry.js';

const grid = document.getElementById('simGrid');

if (grid) {
  grid.innerHTML = '';

  simulators.forEach(sim => {
    const isLive = sim.status === 'live' && sim.href;
    const card = document.createElement(isLive ? 'a' : 'div');
    if (isLive) card.href = sim.href;
    if (!isLive) card.setAttribute('aria-disabled', 'true');

    card.className = [
      'group relative rounded-2xl border border-white/10 bg-white/5 p-6',
      'transition hover:-translate-y-1 hover:border-white/30 hover:bg-white/10',
      isLive ? 'cursor-pointer' : 'cursor-not-allowed opacity-70'
    ].join(' ');

    card.innerHTML = `
      <div class="mb-4 text-xs uppercase tracking-[0.3em] text-slate-400">${sim.status === 'live' ? 'Live' : 'Coming soon'}</div>
      <div class="text-2xl font-semibold text-slate-100">${sim.name}</div>
      <p class="mt-3 text-sm leading-relaxed text-slate-400">${sim.description}</p>
      <div class="mt-6 inline-flex items-center gap-2 text-xs font-semibold uppercase tracking-[0.25em] text-cyan-300">
        ${isLive ? 'Open simulator' : 'In progress'}
      </div>
    `;

    grid.appendChild(card);
  });
}
