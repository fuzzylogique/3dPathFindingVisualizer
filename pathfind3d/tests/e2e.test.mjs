// End-to-end browser tests.
//
// Drives a headless Chromium-based browser over the DevTools protocol using
// Node's built-in WebSocket — no test framework, no driver dependency. What
// this covers that nothing else can: the page actually boots, the modules
// resolve, WebGL initialises, and a real drag on the start marker moves the
// start. That last one is the whole point of the editing rewrite, and it
// cannot be checked without synthesising pointer input.
//
// Run with:  node tests/e2e.test.mjs        (after wasm-pack build)
// Set BROWSER=/path/to/chrome to override browser discovery.

import { spawn } from 'node:child_process';
import { existsSync, mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import assert from 'node:assert/strict';
import { serve } from './server.mjs';

const CANDIDATES = [
  process.env.BROWSER,
  'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
  'C:/Program Files/Microsoft/Edge/Application/msedge.exe',
  'C:/Program Files/Google/Chrome/Application/chrome.exe',
  'C:/Program Files (x86)/Google/Chrome/Application/chrome.exe',
  '/usr/bin/google-chrome',
  '/usr/bin/chromium',
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
].filter(Boolean);

const browserPath = CANDIDATES.find(p => existsSync(p));
if (!browserPath) {
  console.log('SKIP e2e: no Chromium-based browser found (set BROWSER=<path>)');
  process.exit(0);
}

const sleep = ms => new Promise(r => setTimeout(r, ms));

let passed = 0;
const failures = [];
async function test(name, fn) {
  try {
    await fn();
    passed++;
    console.log(`ok  ${name}`);
  } catch (err) {
    failures.push(name);
    console.error(`FAIL ${name}`);
    console.error(err.message || err);
    process.exitCode = 1;
  }
}

/* ---- browser plumbing ------------------------------------------------- */

class Session {
  constructor(ws) {
    this.ws = ws;
    this.id = 0;
    this.pending = new Map();
    this.consoleErrors = [];
    ws.addEventListener('message', ev => {
      const msg = JSON.parse(ev.data);
      if (msg.id !== undefined) {
        const p = this.pending.get(msg.id);
        if (!p) return;
        this.pending.delete(msg.id);
        if (msg.error) p.reject(new Error(msg.error.message));
        else p.resolve(msg.result);
        return;
      }
      if (msg.method === 'Runtime.exceptionThrown') {
        const d = msg.params.exceptionDetails;
        this.consoleErrors.push(d.exception?.description || d.text);
      } else if (msg.method === 'Runtime.consoleAPICalled' && msg.params.type === 'error') {
        this.consoleErrors.push(msg.params.args.map(a => a.description || a.value).join(' '));
      }
    });
  }

  send(method, params = {}) {
    const id = ++this.id;
    this.ws.send(JSON.stringify({ id, method, params }));
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      setTimeout(() => {
        if (this.pending.delete(id)) reject(new Error(`${method} timed out`));
      }, 30000);
    });
  }

  /** Evaluate an expression in the page and return its value. */
  async eval(expression) {
    const r = await this.send('Runtime.evaluate', {
      expression: `(async () => { ${expression} })()`,
      awaitPromise: true,
      returnByValue: true,
    });
    if (r.exceptionDetails) {
      throw new Error(r.exceptionDetails.exception?.description
        || r.exceptionDetails.text);
    }
    return r.result.value;
  }

  async mouse(type, x, y, button = 'left', clickCount = 1) {
    await this.send('Input.dispatchMouseEvent', {
      type, x: Math.round(x), y: Math.round(y), button,
      buttons: type === 'mouseMoved' && button === 'left' ? 1 : (button === 'left' ? 1 : 0),
      clickCount,
    });
  }
}

async function launch(url) {
  const profile = mkdtempSync(join(tmpdir(), 'pathgrid-e2e-'));
  const proc = spawn(browserPath, [
    '--headless=new',
    '--remote-debugging-port=0',
    `--user-data-dir=${profile}`,   // a fresh profile, or the port file is stale
    '--no-first-run',
    '--no-default-browser-check',
    '--disable-extensions',
    '--disable-background-networking',
    '--disable-gpu-sandbox',
    // Software WebGL: headless has no real GPU, and the page needs a context.
    '--use-gl=angle',
    '--use-angle=swiftshader',
    '--enable-unsafe-swiftshader',
    '--window-size=1280,800',
    'about:blank',
  ], { stdio: ['ignore', 'ignore', 'pipe'] });

  const portFile = join(profile, 'DevToolsActivePort');
  let port = null;
  for (let i = 0; i < 120 && port === null; i++) {
    await sleep(250);
    if (!existsSync(portFile)) continue;
    const first = readFileSync(portFile, 'utf8').split('\n')[0].trim();
    if (first) port = Number(first);
  }
  if (!port) {
    proc.kill();
    throw new Error('browser did not expose a DevTools port');
  }

  const list = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  const page = list.find(t => t.type === 'page');
  const ws = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((res, rej) => {
    ws.addEventListener('open', res, { once: true });
    ws.addEventListener('error', () => rej(new Error('devtools socket failed')), { once: true });
  });

  const s = new Session(ws);
  await s.send('Runtime.enable');
  await s.send('Page.enable');
  await s.send('Page.navigate', { url });

  return {
    session: s,
    close: async () => {
      try { await s.send('Browser.close'); } catch {}
      try { ws.close(); } catch {}
      proc.kill();
      await sleep(400);
      try { rmSync(profile, { recursive: true, force: true }); } catch {}
    },
  };
}

/* ---- the run ----------------------------------------------------------- */

const site = await serve(0);
const browser = await launch(site.url);
const page = browser.session;

try {
  // Wait for boot: either the app hook appears or the error overlay does.
  let ready = false;
  for (let i = 0; i < 80 && !ready; i++) {
    await sleep(250);
    ready = await page.eval(
      `return !!window.__pg || getComputedStyle(document.getElementById('err')).display !== 'none';`
    ).catch(() => false);
  }

  await test('the page boots into the wasm core with no error overlay', async () => {
    const overlay = await page.eval(
      `const e = document.getElementById('err');
       return { shown: getComputedStyle(e).display !== 'none', text: e.textContent };`);
    assert.equal(overlay.shown, false, `error overlay: ${overlay.text}`);
    assert.ok(await page.eval('return !!window.__pg'), 'window.__pg missing — boot failed');
    assert.equal(await page.eval(`return document.getElementById('stCore').textContent`), 'WASM');
  });

  await test('no console errors or uncaught exceptions during boot', async () => {
    assert.deepEqual(page.consoleErrors, []);
  });

  await test('WebGL is live and the scene is actually drawing', async () => {
    const info = await page.eval(
      `const r = __pg.app.stage.renderer;
       return { calls: r.info.render.calls, tris: r.info.render.triangles,
                w: r.domElement.width, h: r.domElement.height };`);
    assert.ok(info.w > 0 && info.h > 0, 'canvas has no size');
    assert.ok(info.calls > 0, 'nothing was drawn this frame');
    assert.ok(info.tris > 0, 'no geometry rasterised');
  });

  await test('the world is populated and the search runs on load', async () => {
    const before = await page.eval('return __pg.world.expanded()');
    await sleep(600);
    const after = await page.eval(
      `return { exp: __pg.world.expanded(), cells: __pg.world.cell_count(),
                total: __pg.totalSteps };`);
    assert.ok(after.cells > 1000, `only ${after.cells} cells`);
    assert.ok(after.total > 0, 'scrub range never computed');
    assert.ok(after.exp >= before, 'the search did not advance');
  });

  await test('pause sticks, and an algorithm change does not resume it', async () => {
    await page.eval(`__pg.app.togglePlay();`);
    // Whatever the state was, force paused.
    await page.eval(`if (__pg.ui.playing) __pg.app.togglePlay();`);
    assert.equal(await page.eval('return __pg.ui.playing'), false);

    await page.eval(
      `const s = document.getElementById('selAlgo');
       s.value = '1'; s.dispatchEvent(new Event('change'));`);
    await sleep(300);
    assert.equal(await page.eval('return __pg.ui.playing'), false,
      'changing the algorithm resumed playback');
    assert.equal(await page.eval('return __pg.world.algo()'), 1);
    assert.equal(await page.eval('return __pg.world.expanded()'), 0,
      'the search should restart at zero');
  });

  await test('stepping forward and back moves exactly one expansion', async () => {
    await page.eval(`__pg.app.rewind(); __pg.app.stepForward(); __pg.app.stepForward();`);
    assert.equal(await page.eval('return __pg.world.expanded()'), 2);
    await page.eval(`__pg.app.stepBack();`);
    assert.equal(await page.eval('return __pg.world.expanded()'), 1);
    await page.eval(`__pg.app.stepBack(); __pg.app.stepBack();`);
    assert.equal(await page.eval('return __pg.world.expanded()'), 0,
      'stepping back past the start must clamp, not underflow');
  });

  await test('the scrub bar seeks and stays in sync with the core', async () => {
    const mid = await page.eval(`
      const m = Math.floor(__pg.totalSteps / 2);
      const s = document.getElementById('sScrub');
      s.value = String(m); s.dispatchEvent(new Event('input'));
      return m;`);
    await sleep(120);
    assert.equal(await page.eval('return __pg.world.expanded()'), mid);
    const label = await page.eval(`return document.getElementById('oScrub').textContent`);
    assert.ok(label.startsWith(mid.toLocaleString()), `scrub label reads "${label}"`);
  });

  await test('running to the end finds a path and draws its trail', async () => {
    await page.eval(`
      const s = document.getElementById('selAlgo');
      s.value = '0'; s.dispatchEvent(new Event('change'));
      __pg.app.runToEnd();`);
    await sleep(500);
    const r = await page.eval(
      `return { done: __pg.world.is_done(), found: __pg.world.found(),
                cost: __pg.world.cost(), len: __pg.world.path().length,
                trail: !!__pg.app.trails.curve,
                cycle: __pg.app.stage.cycle.visible };`);
    assert.ok(r.done && r.found, 'the default world should be solvable');
    assert.ok(r.len >= 2 && r.cost > 0);
    assert.ok(r.trail, 'the light trail was not built');
    assert.ok(r.cycle, 'the cycle should ride a found path');
  });

  await test('DRAGGING THE START MARKER MOVES THE START', async () => {
    // The headline of the editing rewrite: no mode to enter, just grab it.
    const before = await page.eval('return __pg.world.start_cell()');
    const from = await page.eval(`
      __pg.app.setAutoRotate(false);
      return __pg.screenOf(__pg.world.start_cell());`);
    assert.ok(from.x > 0 && from.x < 1280 && from.y > 0 && from.y < 800,
      `marker off-screen at ${JSON.stringify(from)}`);

    // Hovering it must advertise that it is grabbable before any click.
    await page.mouse('mouseMoved', from.x, from.y);
    await sleep(150);
    assert.equal(await page.eval(`return document.body.dataset.grab`), '1',
      'hovering the marker did not flag it as grabbable');

    await page.mouse('mousePressed', from.x, from.y);
    await sleep(60);
    for (let i = 1; i <= 6; i++) {
      await page.mouse('mouseMoved', from.x + i * 14, from.y - i * 6);
      await sleep(40);
    }
    await page.mouse('mouseReleased', from.x + 84, from.y - 36);
    await sleep(250);

    const after = await page.eval('return __pg.world.start_cell()');
    assert.notEqual(after, before, 'the drag did not move the start');
    const ok = await page.eval(
      `return { t: __pg.world.terrain_at(__pg.world.start_cell()),
                goal: __pg.world.goal_cell(),
                start: __pg.world.start_cell(),
                exp: __pg.world.expanded() };`);
    assert.notEqual(ok.t, 255, 'the start landed inside a wall');
    assert.notEqual(ok.t, 0, 'the start landed outside the domain');
    assert.notEqual(ok.start, ok.goal, 'the start landed on the goal');
    assert.equal(ok.exp, 0, 'moving an endpoint should restart the search');
  });

  await test('the goal marker drags the same way', async () => {
    const before = await page.eval('return __pg.world.goal_cell()');
    const from = await page.eval('return __pg.screenOf(__pg.world.goal_cell())');
    await page.mouse('mouseMoved', from.x, from.y);
    await sleep(120);
    await page.mouse('mousePressed', from.x, from.y);
    for (let i = 1; i <= 5; i++) {
      await page.mouse('mouseMoved', from.x - i * 12, from.y + i * 7);
      await sleep(40);
    }
    await page.mouse('mouseReleased', from.x - 60, from.y + 35);
    await sleep(250);
    assert.notEqual(await page.eval('return __pg.world.goal_cell()'), before);
  });

  await test('the wall tool previews a cell on hover and builds on click', async () => {
    await page.eval(`__pg.app.setTool('wall');`);
    assert.equal(await page.eval('return document.body.dataset.tool'), 'wall');

    // Point at the middle of the viewport, where the volume is.
    await page.mouse('mouseMoved', 560, 400);
    await sleep(200);
    const hover = await page.eval(
      `return { visible: __pg.app.volume.hover.visible,
                readout: document.getElementById('cellReadout').textContent };`);
    assert.ok(hover.visible, 'no hover highlight — the click target is invisible');
    assert.ok(hover.readout.length > 0, 'no cell readout while hovering');

    const before = await page.eval('return __pg.world.obstacle_cells().length');
    await page.mouse('mousePressed', 560, 400);
    await page.mouse('mouseReleased', 560, 400);
    await sleep(250);
    const after = await page.eval(
      `return { walls: __pg.world.obstacle_cells().length,
                exp: __pg.world.expanded(), stale: __pg.world.is_stale() };`);
    assert.equal(after.walls, before + 1, 'the click did not place a wall');
    assert.equal(after.stale, false, 'releasing should commit and reset the search');
    assert.equal(after.exp, 0, 'an edit should restart the search');
  });

  await test('an edit does not silently resume a paused search', async () => {
    assert.equal(await page.eval('return __pg.ui.playing'), false,
      'editing restarted playback — the original complaint');
  });

  await test('alt-click removes the wall it previews', async () => {
    const before = await page.eval('return __pg.world.obstacle_cells().length');
    await page.send('Input.dispatchMouseEvent', {
      type: 'mousePressed', x: 560, y: 400, button: 'left', buttons: 1,
      clickCount: 1, modifiers: 1, // Alt
    });
    await page.send('Input.dispatchMouseEvent', {
      type: 'mouseReleased', x: 560, y: 400, button: 'left', buttons: 0,
      clickCount: 1, modifiers: 1,
    });
    await sleep(250);
    const after = await page.eval('return __pg.world.obstacle_cells().length');
    assert.equal(after, before - 1, 'alt-click did not remove a wall');
  });

  await test('keyboard shortcuts switch tools and drive the transport', async () => {
    for (const [key, tool] of [['w', 'wall'], ['e', 'weight'], ['r', 'route'], ['v', 'view']]) {
      await page.send('Input.dispatchKeyEvent', { type: 'keyDown', key, text: key });
      await page.send('Input.dispatchKeyEvent', { type: 'keyUp', key });
      await sleep(80);
      assert.equal(await page.eval('return __pg.ui.tool'), tool, `"${key}" should select ${tool}`);
    }
    await page.eval(`__pg.app.rewind();`);
    await page.send('Input.dispatchKeyEvent', { type: 'keyDown', key: '.', text: '.' });
    await page.send('Input.dispatchKeyEvent', { type: 'keyUp', key: '.' });
    await sleep(120);
    assert.equal(await page.eval('return __pg.world.expanded()'), 1, '"." should step forward');
  });

  await test('every domain preset rebuilds and renders', async () => {
    for (const preset of ['prism', 'pyramid', 'sphere', 'torus', 'cube']) {
      await page.eval(`
        const s = document.getElementById('selDomain');
        s.value = '${preset}'; s.dispatchEvent(new Event('change'));`);
      await sleep(450);
      const r = await page.eval(
        `return { preset: __pg.world.preset(), cells: __pg.world.cell_count(),
                  calls: __pg.app.stage.renderer.info.render.calls,
                  overlay: getComputedStyle(document.getElementById('err')).display };`);
      assert.equal(r.preset, preset);
      assert.ok(r.cells > 100, `${preset}: only ${r.cells} cells`);
      assert.ok(r.calls > 0, `${preset}: nothing drawn`);
      assert.equal(r.overlay, 'none', `${preset}: error overlay appeared`);
    }
  });

  await test('drawing a route scores it against the optimum', async () => {
    await page.eval(`
      const s = document.getElementById('selDomain');
      s.value = 'cube'; s.dispatchEvent(new Event('change'));`);
    await sleep(400);
    const verdict = await page.eval(`
      __pg.app.setTool('route');
      __pg.world.route_begin();
      __pg.world.route_append(__pg.world.goal_cell());
      __pg.app.onRouteChanged();
      await new Promise(r => requestAnimationFrame(r));
      await new Promise(r => requestAnimationFrame(r));
      return { hidden: document.getElementById('cmp').hidden,
               you: document.getElementById('cmpYou').textContent,
               opt: document.getElementById('cmpOpt').textContent,
               verdict: document.getElementById('cmpVerdict').textContent,
               user: __pg.world.route_cost(),
               best: __pg.world.optimal_cost() };`);
    assert.equal(verdict.hidden, false, 'the scoreboard stayed hidden');
    assert.ok(Number(verdict.you) > 0, `"you" reads ${verdict.you}`);
    assert.ok(Number(verdict.opt) > 0, `"optimal" reads ${verdict.opt}`);
    assert.ok(verdict.user >= verdict.best - 1e-9, 'a drawn route beat the optimum');
    assert.ok(verdict.verdict.length > 0, 'no verdict line');
  });

  await test('the session ends with no console errors', async () => {
    assert.deepEqual(page.consoleErrors, []);
  });
} finally {
  await browser.close();
  await site.close();
}

console.log(`\n${passed} tests passed${failures.length ? `, ${failures.length} failed` : ''}`);
// Browser helper processes can outlive the kill and pin the event loop.
process.exit(process.exitCode ?? 0);
