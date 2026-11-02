const { chromium } = require('playwright');
const fs = require('fs');
(async () => {
  const b = await chromium.launch();
  const p = await b.newPage({ viewport: { width: 1600, height: 760 } });
  for (const f of process.argv.slice(2)) {
    const svg = fs.readFileSync(f, 'utf8').replace(/<\?xml[^>]*>/, '');
    await p.setContent(`<html><body style="margin:8px;background:#fff">${svg}</body></html>`);
    await p.evaluate(() => { const s = document.querySelector('svg'); s.setAttribute('width', '1560'); s.setAttribute('height', '720'); });
    await p.screenshot({ path: f.replace(/\.svg$/, '.png') });
  }
  await b.close();
})();
