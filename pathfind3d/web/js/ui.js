/* The control panel: DOM wiring and the words on screen.

   Every number shown here is read from the Rust core; nothing is recomputed.
   The panel's one piece of judgement is *when* to ask. */

const $ = id => document.getElementById(id);

const ALGO_META = [
  ['A*',         'guarantees the shortest path — heuristic-guided'],
  ['DIJKSTRA',   'guarantees the shortest path — explores evenly'],
  ['GREEDY',     'beelines to the goal — fast, no guarantee'],
  ['SWARM',      'goal-biased weighted A* — broad, not optimal'],
  ['CONV SWARM', 'heavy goal bias — narrow funnel, not optimal'],
  ['BI-SWARM',   'two swarms meet in the middle — not optimal'],
  ['BFS',        'fewest hops — ignores weights & diagonal cost'],
  ['DFS',        'dives deep — finds a path, rarely a short one'],
];

const DOMAIN_META = {
  cube: 'the classic full box',
  prism: 'non-cubic bounding box',
  pyramid: 'cross-section shrinks with height',
  sphere: 'cells masked by radius — the rest is void',
  torus: 'x wraps around the ring — routes cross the seam',
};

const TOOL_HELP = {
  view: 'Drag to orbit, scroll to zoom. Drag either glowing marker to move the start or the goal — that works in every tool.',
  wall: 'Click to place a wall against the face you are pointing at. Alt-click (or right-click) removes one. Shift-scroll pushes the build plane deeper when there is nothing to build against.',
  weight: 'Click to paint a costly cell at the brush weight. Alt-click resets it to normal ground. Weighted cells cost more to cross, so the weighted algorithms route around them.',
  route: 'Click your own way from start to goal — distant clicks auto-connect. Finish at the goal to score your route against the optimum.',
};

const HINTS = {
  view: 'DRAG TO ORBIT · SCROLL TO ZOOM · DRAG THE MARKERS TO MOVE START AND GOAL',
  wall: 'CLICK TO BUILD · ALT-CLICK TO REMOVE',
  weight: 'CLICK TO PAINT COST · ALT-CLICK TO RESET',
  route: 'CLICK FROM START TO GOAL · FAR CLICKS AUTO-CONNECT',
};

export function createPanel(app) {
  const { world, ui } = app;
  const el = {
    status: $('stStatus'), exp: $('stExp'), fro: $('stFro'),
    path: $('stPath'), cost: $('stCost'),
    cells: $('stCells'), seed: $('stSeed'),
    cmp: $('cmp'), cmpYou: $('cmpYou'), cmpOpt: $('cmpOpt'),
    cmpAlg: $('cmpAlg'), cmpAlgName: $('cmpAlgName'), cmpVerdict: $('cmpVerdict'),
    play: $('bPlay'), back: $('bBack'), fwd: $('bFwd'),
    rewind: $('bRewind'), end: $('bEnd'),
    scrub: $('sScrub'), oScrub: $('oScrub'),
    hint: $('hint'), readout: $('cellReadout'), announce: $('announce'),
    toolHelp: $('toolHelp'), routeBtns: $('routeBtns'),
    algoDesc: $('algoDesc'), domDesc: $('domDesc'),
    clipOut: $('oClip'),
  };

  let hintTimer = 0;
  let bootFaded = false;
  let pinnedHint = null;   // a drag in progress overrides the tool hint
  let lastCell = -1;

  // ---- about -----------------------------------------------------------

  const aboutBtn = $('bAbout');
  const setAbout = open => {
    aboutBtn.setAttribute('aria-expanded', String(open));
    // Focus alone keeps it open (for keyboard users), so closing by click
    // has to drop focus too.
    if (!open) aboutBtn.blur();
  };
  aboutBtn.addEventListener('click', e => {
    e.stopPropagation();
    setAbout(aboutBtn.getAttribute('aria-expanded') !== 'true');
  });
  document.addEventListener('pointerdown', e => {
    if (!e.target.closest('.about')) setAbout(false);
  });
  addEventListener('keydown', e => { if (e.key === 'Escape') setAbout(false); });

  // ---- transport -------------------------------------------------------

  el.play.addEventListener('click', () => app.togglePlay());
  el.back.addEventListener('click', () => app.stepBack());
  el.fwd.addEventListener('click', () => app.stepForward());
  el.rewind.addEventListener('click', () => app.rewind());
  el.end.addEventListener('click', () => app.runToEnd());
  el.scrub.addEventListener('input', e => app.seek(+e.target.value));

  $('sSpeed').addEventListener('input', e => {
    ui.speed = +e.target.value;
    $('oSpeed').textContent = ui.speed;
  });

  // ---- algorithm & world ----------------------------------------------

  $('selAlgo').addEventListener('change', e => {
    world.set_algo(+e.target.value);
    el.algoDesc.textContent = ALGO_META[world.algo()][1];
    app.onSearchReset();
  });

  $('selDomain').addEventListener('change', e => {
    ui.preset = e.target.value;
    el.domDesc.textContent = DOMAIN_META[ui.preset];
    app.rebuildWorld();
  });

  $('sGrid').addEventListener('input', e => { $('oGrid').textContent = e.target.value; });
  $('sGrid').addEventListener('change', e => {
    ui.size = +e.target.value;
    app.rebuildWorld();
  });
  $('sDens').addEventListener('input', e => { $('oDens').textContent = `${e.target.value}%`; });
  $('sDens').addEventListener('change', e => {
    ui.density = +e.target.value;
    app.rebuildWorld();
  });
  $('bRand').addEventListener('click', () => app.randomize());
  $('bClear').addEventListener('click', () => app.clearWorld());
  $('tEmpty').addEventListener('change', e => { ui.emptyOnRebuild = e.target.checked; });

  // ---- tools -----------------------------------------------------------

  const toolBtns = [...document.querySelectorAll('.tools button')];
  for (const b of toolBtns) b.addEventListener('click', () => app.setTool(b.dataset.tool));

  $('sWeight').addEventListener('input', e => {
    ui.brushW = +e.target.value;
    $('oWeight').textContent = ui.brushW;
  });
  $('bUndoRoute').addEventListener('click', () => { world.route_undo(); app.onRouteChanged(); });
  $('bClearRoute').addEventListener('click', () => {
    world.route_clear();
    world.route_begin();
    app.onRouteChanged();
  });

  // ---- visibility ------------------------------------------------------

  $('sXray').addEventListener('input', e => {
    ui.xray = +e.target.value / 100;
    $('oXray').textContent = `${e.target.value}%`;
    app.onVisibilityChanged();
  });
  $('selClipAxis').addEventListener('change', e => {
    ui.clipAxis = +e.target.value;
    app.onVisibilityChanged();
  });
  $('sClip').addEventListener('input', e => {
    ui.clipT = +e.target.value / 100;
    if (!ui.clipOn && ui.clipT < 1) { ui.clipOn = true; $('tClip').checked = true; }
    app.onVisibilityChanged();
  });
  $('tClip').addEventListener('change', e => {
    ui.clipOn = e.target.checked;
    app.onVisibilityChanged();
  });
  $('tCloud').addEventListener('change', e => {
    ui.showCloud = e.target.checked;
    app.onVisibilityChanged();
  });
  $('tRotate').addEventListener('change', e => app.setAutoRotate(e.target.checked));
  $('tDiag').addEventListener('change', e => {
    world.set_diagonals(e.target.checked);
    app.onSearchReset();
  });

  // ---- rendering the panel ---------------------------------------------

  const panel = {
    clipOut: el.clipOut,

    /** Reflect the tool everywhere it shows: buttons, body attribute, help
     *  copy, route controls and the hint bar. */
    showTool(tool) {
      document.body.dataset.tool = tool;
      for (const b of toolBtns) {
        const on = b.dataset.tool === tool;
        b.classList.toggle('on', on);
        b.setAttribute('aria-pressed', String(on));
      }
      el.routeBtns.hidden = tool !== 'route';
      el.toolHelp.textContent = TOOL_HELP[tool];
      panel.setHint(null);
    },

    /** `null` restores the tool's own hint; a string pins a message. */
    setHint(text) {
      pinnedHint = text;
      clearTimeout(hintTimer);
      el.hint.classList.remove('warn');
      el.hint.textContent = text || HINTS[ui.tool];
      el.hint.classList.toggle('gone', !text && ui.tool === 'view' && bootFaded);
    },

    warn(msg) {
      clearTimeout(hintTimer);
      el.hint.textContent = msg;
      el.hint.classList.add('warn');
      el.hint.classList.remove('gone');
      el.announce.textContent = msg; // screen-reader channel
      hintTimer = setTimeout(() => panel.setHint(pinnedHint), 1800);
    },

    /** Bottom-left readout of the cell under the cursor. */
    showCell(cell) {
      if (cell === lastCell) return;
      lastCell = cell;
      if (cell < 0) { el.readout.textContent = ''; return; }
      const nx = world.nx(), ny = world.ny();
      const x = cell % nx, y = ((cell / nx) | 0) % ny, z = (cell / (nx * ny)) | 0;
      const t = world.terrain_at(cell);
      const kind = t === 255 ? '<em>WALL</em>' : t > 1 ? `<em>COST ×${t}</em>` : 'OPEN';
      el.readout.innerHTML = `<b>${x},${y},${z}</b> &middot; ${kind}`;
    },

    bootDone() { bootFaded = true; panel.setHint(pinnedHint); },

    /** Domain identity line; only changes when the world is rebuilt. */
    showWorld() {
      el.cells.textContent =
        `${world.nx()}×${world.ny()}×${world.nz()} · ${world.cell_count().toLocaleString()} CELLS`;
      el.seed.textContent = world.seed();
      $('selDomain').value = world.preset();
      $('sGrid').value = ui.size;
      $('oGrid').textContent = ui.size;
      $('sDens').value = ui.density;
      $('oDens').textContent = `${ui.density}%`;
      el.domDesc.textContent = DOMAIN_META[world.preset()];
      el.algoDesc.textContent = ALGO_META[world.algo()][1];
    },

    /** Transport buttons and the scrub bar. `total` is the expansion count of
     *  a completed search, which only the core can tell us. */
    showTransport(total) {
      const at = world.expanded();
      const done = world.is_done();
      el.play.innerHTML = ui.playing ? '&#10073;&#10073; PAUSE' : '&#9654; RUN';
      el.back.disabled = at === 0;
      el.rewind.disabled = at === 0;
      el.fwd.disabled = done;
      el.end.disabled = done;
      if (+el.scrub.max !== total) el.scrub.max = String(Math.max(1, total));
      if (+el.scrub.value !== at) el.scrub.value = String(at);
      el.oScrub.textContent = `${at.toLocaleString()} / ${total.toLocaleString()}`;
    },

    showStats(pathLen, cost) {
      el.exp.textContent = world.expanded().toLocaleString();
      el.fro.textContent = world.frontier().toLocaleString();
      el.path.textContent = pathLen > 0 ? pathLen : '—';
      el.cost.textContent = Number.isFinite(cost) ? cost.toFixed(2) : '—';
    },

    showStatus() {
      let txt, flag;
      if (world.is_stale()) {
        txt = 'EDITING'; flag = 'stale';
      } else if (world.is_done()) {
        txt = world.found() ? 'PATH FOUND' : 'NO PATH';
        flag = world.found() ? 'found' : 'nopath';
      } else if (world.expanded() > 0) {
        txt = ui.playing ? 'SEARCHING' : 'PAUSED';
        flag = ui.playing ? 'run' : 'paused';
      } else if (ui.tool === 'route' && world.route_len() > 1) {
        txt = world.route_complete() ? 'ROUTE READY · RUN' : `ROUTE · ${world.route_len()} CELLS`;
        flag = 'draw';
      } else {
        txt = 'READY'; flag = 'ready';
      }
      if (el.status.textContent !== txt) {
        el.status.textContent = txt;
        el.status.dataset.s = flag;
      }
    },

    /** The route scoreboard. Hidden until a complete, legal route exists. */
    showCompare(show, algCost) {
      if (!show) { el.cmp.hidden = true; return; }
      const user = world.route_cost();
      const opt = world.optimal_cost();
      el.cmp.hidden = false;
      el.cmpYou.textContent = Number.isFinite(user) ? user.toFixed(2) : '—';
      el.cmpOpt.textContent = Number.isFinite(opt) ? opt.toFixed(2) : 'NO PATH';
      el.cmpAlgName.textContent = ALGO_META[world.algo()][0];
      el.cmpAlg.textContent = Number.isFinite(algCost) ? algCost.toFixed(2) : '—';
      if (Number.isFinite(opt) && opt > 0 && Number.isFinite(user)) {
        const over = (user / opt - 1) * 100;
        el.cmpVerdict.textContent = over < 0.005
          ? 'YOUR ROUTE MATCHES THE OPTIMUM'
          : `+${(user - opt).toFixed(2)} · ${over.toFixed(over < 10 ? 1 : 0)}% LONGER THAN OPTIMAL`;
      } else {
        el.cmpVerdict.textContent = '';
      }
      el.announce.textContent = el.cmpVerdict.textContent;
    },

    setRotateChecked(on) { $('tRotate').checked = on; },
  };

  // Initial paint of the static copy.
  $('oSpeed').textContent = ui.speed;
  $('oWeight').textContent = ui.brushW;
  $('oXray').textContent = `${Math.round(ui.xray * 100)}%`;
  setTimeout(panel.bootDone, 6500);

  return panel;
}

/** Full-screen failure notice. WASM is not optional any more, so a missing
 *  build has to say exactly how to produce one. */
export function showError(title, detail) {
  const e = $('err');
  e.innerHTML = detail
    ? `${title}<code>${detail}</code>`
    : title;
  e.style.display = 'grid';
}
