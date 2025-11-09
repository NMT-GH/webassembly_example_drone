// site/main.js (ES module)
import createModule from './sim.js';

const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

// ——— Canvas & DPI ———
function setupCanvasSize() {
  const cssWidth  = 1000;
  const cssHeight = 400;

  const dpr = window.devicePixelRatio || 1;
  canvas.style.width  = cssWidth + 'px';
  canvas.style.height = cssHeight + 'px';
  canvas.width  = Math.floor(cssWidth  * dpr);
  canvas.height = Math.floor(cssHeight * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0); // draw in CSS pixels
}
setupCanvasSize();
window.addEventListener('resize', setupCanvasSize);

// ——— Load WASM ———
const Module = await createModule();

// Bind C exports
const sim_init = Module.cwrap('sim_init', 'number', ['number']);
const sim_step = Module.cwrap('sim_step', null, ['number', 'number']);

const get_interceptor_pos_x = Module.cwrap('get_interceptor_pos_x', 'number', []);
const get_interceptor_pos_y = Module.cwrap('get_interceptor_pos_y', 'number', []);
const get_interceptor_kin_ang = Module.cwrap('get_interceptor_kin_ang', 'number', []);

const get_target_pos_x = Module.cwrap('get_target_pos_x', 'number', []);
const get_target_pos_y = Module.cwrap('get_target_pos_y', 'number', []);
const get_target_kin_ang = Module.cwrap('get_target_kin_ang', 'number', []);

// --- Sim/controls setup ---
const DT = 0.01; // s
if (sim_init(DT) !== 0) throw new Error('sim_init failed');

let keys = { up:false, down:false, left:false, right:false };
window.addEventListener('keydown', (e) => {
  if (e.key === 'ArrowUp')    { keys.up = true; e.preventDefault(); }
  if (e.key === 'ArrowDown')  { keys.down = true; e.preventDefault(); }
  if (e.key === 'ArrowLeft')  { keys.left = true; e.preventDefault(); }
  if (e.key === 'ArrowRight') { keys.right = true; e.preventDefault(); }
});
window.addEventListener('keyup', (e) => {
  if (e.key === 'ArrowUp')    keys.up = false;
  if (e.key === 'ArrowDown')  keys.down = false;
  if (e.key === 'ArrowLeft')  keys.left = false;
  if (e.key === 'ArrowRight') keys.right = false;
});

// Command scaling (maps to sim_step(leftRight, frontBack))
const X_STEP = 1.0; // left/right
const Y_STEP = 1.0; // front/back

// ——— World units & camera config ———
const ppm         = 0.25;     // pixels per meter
const GRID_STEP_M = 100;    // grid spacing (meters)
const GROUND_Y_M  = 0.0;     // ground line at y=0 in world units

// Camera is “center + zoom”. We render the world multiple times with cameras.
function beginCamera(camera, viewportPx) {
  ctx.save();
  ctx.beginPath();
  ctx.rect(viewportPx.x, viewportPx.y, viewportPx.w, viewportPx.h);
  ctx.clip();

  ctx.translate(viewportPx.x + viewportPx.w/2, viewportPx.y + viewportPx.h/2);
  ctx.scale(camera.zoom * ppm, -camera.zoom * ppm);
  ctx.translate(-camera.x, -camera.y);

  ctx.lineWidth = 1 / (camera.zoom * ppm);
}

function endCamera() { ctx.restore(); }

function worldViewBounds(camera, viewportPx) {
  const halfWm = (viewportPx.w / (camera.zoom * ppm)) * 0.5;
  const halfHm = (viewportPx.h / (camera.zoom * ppm)) * 0.5;
  return {
    xmin: camera.x - halfWm,
    xmax: camera.x + halfWm,
    ymin: camera.y - halfHm,
    ymax: camera.y + halfHm,
  };
}

// ——— World rendering (in meters) ———
function drawGridWorld(camera, viewportPx) {
  const b = worldViewBounds(camera, viewportPx);
  const gx0 = Math.ceil(b.xmin / GRID_STEP_M);
  const gx1 = Math.floor(b.xmax / GRID_STEP_M);
  const gy0 = Math.ceil(b.ymin / GRID_STEP_M);
  const gy1 = Math.floor(b.ymax / GRID_STEP_M);

  ctx.save();
  ctx.strokeStyle = '#383838ff';
  ctx.beginPath();
  for (let k = gx0; k <= gx1; k++) {
    const x = k * GRID_STEP_M;
    ctx.moveTo(x, b.ymin);
    ctx.lineTo(x, b.ymax);
  }
  for (let k = gy0; k <= gy1; k++) {
    const y = k * GRID_STEP_M;
    ctx.moveTo(b.xmin, y);
    ctx.lineTo(b.xmax, y);
  }
  ctx.stroke();
  ctx.restore();
}

function drawGroundWorld(camera, viewportPx) {
  const b = worldViewBounds(camera, viewportPx);
  ctx.save();
  ctx.strokeStyle = '#666';
  ctx.lineWidth = 1.5 / (camera.zoom * ppm);
  ctx.beginPath();
  ctx.moveTo(b.xmin, GROUND_Y_M);
  ctx.lineTo(b.xmax, GROUND_Y_M);
  ctx.stroke();
  ctx.restore();
}

// ——— Elongated triangle (for both interceptor & target) ———
function drawTriangleWorld(x, y, ang, camera, color = '#ffffffff') {
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(ang);

  // Triangle dimensions (meters): long and skinny
  const L = 0.30 * 100;   // length
  const W = 0.10 * 100;   // width

  ctx.beginPath();
  // Nose forward (+X local), base at negative X
  ctx.moveTo(+L/2, 0);            // tip
  ctx.lineTo(-L/2, +W/2);         // rear-left
  ctx.lineTo(-L/2, -W/2);         // rear-right
  ctx.closePath();

  ctx.fillStyle = color;
  ctx.fill();
  ctx.lineWidth = 1.0 / (camera.zoom * ppm);
  ctx.strokeStyle = color;
  ctx.stroke();

  ctx.restore();
}

function renderWorld(camera, viewportPx, interceptor, target) {
  beginCamera(camera, viewportPx);
  drawGridWorld(camera, viewportPx);
  drawGroundWorld(camera, viewportPx);

  // Draw target first (behind), interceptor on top
  if (target)      drawTriangleWorld(target.x,      target.y,      target.ang,      camera, '#ffcc00ff'); // yellow
  if (interceptor) drawTriangleWorld(interceptor.x, interceptor.y, interceptor.ang, camera, '#ffffffff'); // white

  endCamera();
}

// ——— UI panels (screen space) ———
function drawInsetPanel(ix, iy, size) {
  ctx.save();
  ctx.beginPath();
  ctx.rect(ix, iy, size, size);
  ctx.fillStyle = '#191919ff';
  ctx.fill();

  ctx.beginPath();
  ctx.rect(ix - 2, iy - 2, size + 4, size + 4);
  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(0,0,0,0.35)';
  ctx.stroke();
  ctx.restore();
}

// ——— Inset config ———
const INSET_SIZE_PX = 90;
const INSET_PADDING = 8;
const INSET_ZOOM    = 2.5;

// ——— Frame render ———
function drawFrame(interceptor, target) {
  // Clear whole canvas to background color
  ctx.fillStyle = '#191919ff';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const dpr = window.devicePixelRatio || 1;
  const W = canvas.width / dpr;
  const H = canvas.height / dpr;

  // MAIN CAMERA (center between interceptor and target)
  const cx = 0.5 * (interceptor.x + target.x);
  const cy = 0.5 * (interceptor.y + target.y);
  //const mainCam = { x: cx, y: cy, zoom: 1.5 };
  const mainCam = { x: 0, y: 0, zoom: 1.5 };
  const mainViewport = { x: 0, y: 0, w: W, h: H };
  renderWorld(mainCam, mainViewport, interceptor, target);

  // TWO INSETS:
  // Left inset = target zoom
  const size = Math.min(INSET_SIZE_PX, Math.min(W, H) - 2 * INSET_PADDING);
  const left_ix  = INSET_PADDING;
  const right_ix = W - size - INSET_PADDING;
  const iy       = INSET_PADDING;

  // Left: target
  drawInsetPanel(left_ix, iy, size);
  const targetCam = { x: target.x, y: target.y, zoom: INSET_ZOOM };
  const targetViewport = { x: left_ix, y: iy, w: size, h: size };
  renderWorld(targetCam, targetViewport, interceptor, target);

  // Right: interceptor
  drawInsetPanel(right_ix, iy, size);
  const interceptorCam = { x: interceptor.x, y: interceptor.y, zoom: INSET_ZOOM };
  const interceptorViewport = { x: right_ix, y: iy, w: size, h: size };
  renderWorld(interceptorCam, interceptorViewport, interceptor, target);
}

// ——— Fixed-step sim loop ———
let last = performance.now();
let acc = 0;

function tick(now) {
  let dt = (now - last) / 1000;
  last = now;
  if (dt > 0.25) dt = 0.25;
  acc += dt;

  const frontBack = (keys.up ? +Y_STEP : 0) + (keys.down ? -Y_STEP : 0);
  const leftRight = (keys.right ? -X_STEP : 0) + (keys.left ? +X_STEP : 0);

  while (acc >= DT) {
    // New step signature: sim_step(float leftRight, float frontBack);
    sim_step(leftRight, frontBack);
    acc -= DT;
  }

  const interceptor = {
    x:   get_interceptor_pos_x(),
    y:   get_interceptor_pos_y(),
    ang: get_interceptor_kin_ang()
  };

  const target = {
    x:   get_target_pos_x(),
    y:   get_target_pos_y(),
    ang: get_target_kin_ang()
  };

  drawFrame(interceptor, target);
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
