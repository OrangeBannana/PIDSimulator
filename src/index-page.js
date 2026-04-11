(() => {
  const simulators = Array.isArray(window.simulators) ? window.simulators : [];

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

  if (!customElements.get('site-header')) {
    customElements.define('site-header', SiteHeader);
  }

  const grid = document.getElementById('simGrid');

  if (grid) {
    grid.innerHTML = '';

    simulators.forEach(sim => {
      const isLive = Boolean(sim.href);
      const card = document.createElement(isLive ? 'a' : 'div');
      if (isLive) card.href = sim.href;
      if (!isLive) card.setAttribute('aria-disabled', 'true');

      card.className = [
        'group relative rounded-2xl border border-white/10 bg-white/5 p-6',
        'transition hover:-translate-y-1 hover:border-white/30 hover:bg-white/10',
        isLive ? 'cursor-pointer' : 'cursor-not-allowed opacity-70'
      ].join(' ');

      card.innerHTML = `
        <div class="text-2xl font-semibold text-slate-100">${sim.name}</div>
        <p class="mt-3 text-sm leading-relaxed text-slate-400">${sim.description}</p>
        <div class="mt-6 inline-flex items-center gap-2 text-xs font-semibold uppercase tracking-[0.25em] text-cyan-300">
          ${isLive ? 'Open simulator' : 'In progress'}
        </div>
      `;

      grid.appendChild(card);
    });
  }
})();
