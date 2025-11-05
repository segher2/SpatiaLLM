#!/usr/bin/env node
/**
 * Start both frontend (Vite) and backend (API server) concurrently
 */
import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { dirname, resolve } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const projectRoot = resolve(__dirname, '../../..');

// ANSI color codes
const colors = {
  reset: '\x1b[0m',
  cyan: '\x1b[36m',
  yellow: '\x1b[33m',
  green: '\x1b[32m',
  red: '\x1b[31m',
  magenta: '\x1b[35m'
};

function log(prefix, color, message) {
  console.log(`${color}[${prefix}]${colors.reset} ${message}`);
}

// Start API server
log('API', colors.cyan, 'Starting API server on port 8090...');
const apiServer = spawn('python3', [
  resolve(projectRoot, 'scripts/api_server.py'),
  '--port', '8090'
], {
  cwd: projectRoot,
  stdio: 'pipe'
});

apiServer.stdout.on('data', (data) => {
  const lines = data.toString().split('\n').filter(line => line.trim());
  lines.forEach(line => log('API', colors.cyan, line));
});

apiServer.stderr.on('data', (data) => {
  const lines = data.toString().split('\n').filter(line => line.trim());
  lines.forEach(line => log('API', colors.red, line));
});

apiServer.on('close', (code) => {
  if (code !== 0) {
    log('API', colors.red, `Server exited with code ${code}`);
  }
  process.exit(code);
});

// Wait a bit for API server to start
setTimeout(() => {
  // Start Vite dev server
  log('VITE', colors.magenta, 'Starting Vite dev server on port 5173...');
  const viteServer = spawn('npm', ['run', 'dev'], {
    cwd: resolve(__dirname, '..'),
    stdio: 'inherit',
    shell: true
  });

  viteServer.on('close', (code) => {
    log('VITE', colors.magenta, 'Dev server stopped');
    apiServer.kill();
    process.exit(code);
  });
}, 2000);

// Handle Ctrl+C
process.on('SIGINT', () => {
  log('SHUTDOWN', colors.yellow, 'Shutting down servers...');
  apiServer.kill();
  process.exit(0);
});

log('STARTUP', colors.green, '✓ Starting both servers...');
log('STARTUP', colors.green, '  Frontend: http://localhost:5173/');
log('STARTUP', colors.green, '  API:      http://localhost:8090/');
log('STARTUP', colors.yellow, 'Press Ctrl+C to stop both servers');
