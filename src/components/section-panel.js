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

customElements.define('section-panel', SectionPanel);
