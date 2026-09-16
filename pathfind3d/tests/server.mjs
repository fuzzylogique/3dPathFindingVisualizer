// Minimal static file server for web/.
//
// ES modules and WebAssembly both refuse to load over file://, so the page
// needs a real HTTP origin — during development and in the browser tests.
//
//   node tests/server.mjs [port]      (default 8080)

import { createServer } from 'node:http';
import { createReadStream, statSync } from 'node:fs';
import { extname, join, normalize, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = fileURLToPath(new URL('../web/', import.meta.url));

const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.wasm': 'application/wasm',
  '.ts': 'text/plain; charset=utf-8',
  '.md': 'text/plain; charset=utf-8',
};

export function serve(port = 0) {
  const server = createServer((req, res) => {
    const url = new URL(req.url, 'http://localhost');
    let rel = decodeURIComponent(url.pathname);
    if (rel.endsWith('/')) rel += 'index.html';
    // Keep the served tree inside web/ whatever the request says.
    const path = join(ROOT, normalize(rel).replace(/^([/\\]|\.\.)+/, ''));
    if (!path.startsWith(ROOT.replace(/[/\\]$/, '') + sep)) {
      res.writeHead(403).end('forbidden');
      return;
    }
    let size;
    try {
      const st = statSync(path);
      if (st.isDirectory()) throw new Error('directory');
      size = st.size;
    } catch {
      res.writeHead(404, { 'content-type': 'text/plain' }).end('not found');
      return;
    }
    res.writeHead(200, {
      'content-type': TYPES[extname(path)] || 'application/octet-stream',
      'content-length': size,
      'cache-control': 'no-store',
    });
    createReadStream(path).pipe(res);
  });
  return new Promise(resolve => {
    server.listen(port, '127.0.0.1', () => resolve({
      server,
      port: server.address().port,
      url: `http://127.0.0.1:${server.address().port}/`,
      close: () => new Promise(r => {
        // A killed browser can leave keep-alive sockets half-open on Windows;
        // without this, close() waits on them forever.
        server.closeAllConnections();
        server.close(r);
      }),
    }));
  });
}

// Run directly: serve web/ on the requested port and stay up.
if (import.meta.url === `file://${process.argv[1]}`.replace(/\\/g, '/')
  || process.argv[1]?.endsWith('server.mjs')) {
  const port = Number(process.argv[2]) || 8080;
  serve(port).then(s => console.log(`serving ${ROOT} at ${s.url}`));
}
