/**
 * serve.mjs
 * ---------------------------------------------------------------------------
 * Zero-dependency static file server.
 *
 * ES modules and the .glb fetch both fall foul of the file:// origin, so the
 * project has to be served over HTTP. Run `npm start` and open the URL it
 * prints. Any other static server (python -m http.server, live-server, ...)
 * works just as well.
 * ---------------------------------------------------------------------------
 */

import { createServer } from 'node:http';
import { createReadStream, statSync } from 'node:fs';
import { extname, join, normalize, resolve, sep } from 'node:path';

const ROOT = resolve(process.cwd());
const PORT = Number(process.env.PORT) || 5173;

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.glb': 'model/gltf-binary',
  '.gltf': 'model/gltf+json',
  '.bin': 'application/octet-stream',
  '.hdr': 'image/vnd.radiance',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.jpeg': 'image/jpeg',
  '.webp': 'image/webp',
  '.ktx2': 'image/ktx2',
  '.svg': 'image/svg+xml',
  '.woff2': 'font/woff2',
  '.map': 'application/json; charset=utf-8',
};

const server = createServer((req, res) => {
  let urlPath;
  try {
    urlPath = decodeURIComponent(new URL(req.url, 'http://localhost').pathname);
  } catch {
    res.writeHead(400).end('Bad request');
    return;
  }
  if (urlPath === '/') urlPath = '/index.html';

  // Contain every request inside the project directory.
  const filePath = join(ROOT, normalize(urlPath).replace(/^(\.\.[/\\])+/, ''));
  if (!filePath.startsWith(ROOT + sep) && filePath !== ROOT) {
    res.writeHead(403).end('Forbidden');
    return;
  }

  let stat;
  try {
    stat = statSync(filePath);
    if (stat.isDirectory()) throw new Error('directory');
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain' }).end('404 Not Found');
    return;
  }

  res.writeHead(200, {
    'Content-Type': MIME[extname(filePath).toLowerCase()] || 'application/octet-stream',
    'Content-Length': stat.size,
    'Cache-Control': 'no-cache',
  });
  createReadStream(filePath).pipe(res);
});

server.listen(PORT, () => {
  console.log(`\n  BMW physics simulation\n  http://localhost:${PORT}\n`);
});
