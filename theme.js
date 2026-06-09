document.addEventListener('DOMContentLoaded', () => {
  const toggle = document.getElementById('themeToggle');
  const html = document.documentElement;
  const saved = localStorage.getItem('theme') || 'dark';

  if (saved === 'light') {
    html.setAttribute('data-theme', 'light');
    toggle.textContent = '切换深色模式';
  } else {
    html.setAttribute('data-theme', 'dark');
    toggle.textContent = '切换浅色模式';
  }

  toggle.addEventListener('click', () => {
    const current = html.getAttribute('data-theme');
    const next = current === 'dark' ? 'light' : 'dark';
    html.setAttribute('data-theme', next);
    localStorage.setItem('theme', next);
    toggle.textContent = next === 'light' ? '切换深色模式' : '切换浅色模式';
  });
});