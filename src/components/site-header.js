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

    const logo = document.createElement('div');
    logo.className = 'logo';
    logo.innerHTML = `
      <div class="logo-badge">PID</div>
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

customElements.define('site-header', SiteHeader);
