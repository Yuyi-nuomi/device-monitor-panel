// theme.js - 全局主题同步脚本
document.addEventListener('DOMContentLoaded', () => {
    const themeToggle = document.getElementById('themeToggle');
    const savedTheme = localStorage.getItem('theme') || 'dark';

    // 初始化主题
    if (savedTheme === 'light') {
        document.body.classList.add('light-mode');
        themeToggle.textContent = '切换深色模式';
    } else {
        document.body.classList.remove('light-mode');
        themeToggle.textContent = '切换浅色模式';
    }

    // 切换主题
    themeToggle.addEventListener('click', () => {
        document.body.classList.toggle('light-mode');
        const newTheme = document.body.classList.contains('light-mode') ? 'light' : 'dark';
        localStorage.setItem('theme', newTheme);
        themeToggle.textContent = newTheme === 'light' ? '切换深色模式' : '切换浅色模式';
    });
});