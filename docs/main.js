// site/main.js (ES module)
import createModule from './sim.js';

const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

// ——— Canvas & DPI ———
function setupCanvasSize() {
  const cssWidth  = 800;
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
const sim_init      = Module.cwrap('sim_init', 'number', ['number']);
const sim_step      = Module.cwrap('sim_step', null, ['number', 'number']);
const drone_get_x   = Module.cwrap('drone_get_x', 'number', []);
const drone_get_y   = Module.cwrap('drone_get_y', 'number', []);
const drone_get_ang = Module.cwrap('drone_get_angle', 'number', []);

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

// Command scaling
const X_STEP = 1.0; // lateral/torque-ish
const Y_STEP = 1.0; // thrust-ish

// ——— World units & camera config ———
const ppm         = 200;     // pixels per meter
const GRID_STEP_M = 0.25;    // grid spacing (meters)
const GROUND_Y_M  = 0.0;     // ground line at y=0 in world units

// Camera is “center + zoom”. We render the world twice with two cameras.
function beginCamera(camera, viewportPx) {
  // viewportPx = { x, y, w, h } in CSS pixels
  ctx.save();

  // Clip to viewport rectangle
  ctx.beginPath();
  ctx.rect(viewportPx.x, viewportPx.y, viewportPx.w, viewportPx.h);
  ctx.clip();

  // Transform from world meters → screen pixels inside this viewport
  // 1) move origin to viewport center
  ctx.translate(viewportPx.x + viewportPx.w/2, viewportPx.y + viewportPx.h/2);
  // 2) scale: meters → pixels; flip Y so +Y is upward in world
  ctx.scale(camera.zoom * ppm, -camera.zoom * ppm);
  // 3) center on camera (world coords)
  ctx.translate(-camera.x, -camera.y);

  // Make 1-pixel strokes regardless of zoom (stroke widths are in world units now)
  ctx.lineWidth = 1 / (camera.zoom * ppm);
}

function endCamera() {
  ctx.restore();
}

// Compute visible world bounds for current camera + viewport
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

  // Find first/last lines on the 0.25 m lattice
  const gx0 = Math.ceil(b.xmin / GRID_STEP_M);
  const gx1 = Math.floor(b.xmax / GRID_STEP_M);
  const gy0 = Math.ceil(b.ymin / GRID_STEP_M);
  const gy1 = Math.floor(b.ymax / GRID_STEP_M);

  ctx.save();
  ctx.strokeStyle = '#383838ff';
  ctx.beginPath();

  // Vertical lines at x = k * GRID_STEP_M
  for (let k = gx0; k <= gx1; k++) {
    const x = k * GRID_STEP_M;
    ctx.moveTo(x, b.ymin);
    ctx.lineTo(x, b.ymax);
  }
  // Horizontal lines at y = k * GRID_STEP_M
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
  ctx.lineWidth = 1.5 / (camera.zoom * ppm); // 1.5 px
  ctx.beginPath();
  ctx.moveTo(b.xmin, GROUND_Y_M);
  ctx.lineTo(b.xmax, GROUND_Y_M);
  ctx.stroke();
  ctx.restore();
}

// --- NEW: target '+' marker (world units) ---
function drawTargetWorld(x, y, camera) {
  ctx.save();
  const s = 0.05; // half-size of '+' in meters (~10 px at ppm=200)
  ctx.beginPath();
  ctx.moveTo(x - s, y);
  ctx.lineTo(x + s, y);
  ctx.moveTo(x, y - s);
  ctx.lineTo(x, y + s);
  ctx.strokeStyle = '#ffcc00aa';               // subtle yellow, slightly transparent
  ctx.lineWidth = 1.5 / (camera.zoom * ppm);   // ~1.5 px regardless of zoom
  ctx.stroke();
  ctx.restore();
}

function drawDroneWorld(x, y, ang, camera) {
  ctx.save();
  ctx.translate(x, y);

  // Use the sim angle directly. Because we already flipped the canvas Y in the camera
  // (scale(..., -...)), rotating by +ang now matches a standard math CCW convention.
  ctx.rotate(ang);

  const armM = 0.127; // ~arm length in meters

  // NOTE: This shape was originally authored for a "Y down" canvas.
  // We are in a "Y up" world now, so invert the local Y offsets.
  ctx.beginPath();
  const c = 0, a = armM;

  ctx.moveTo(c, c);
  ctx.lineTo(c - a/2, c);
  ctx.lineTo(c + a/2, c);
  ctx.lineTo(c + a/2, c - a/20);
  ctx.lineTo(c + a/5,  c - a/20);
  ctx.lineTo(c + a/5,  c - a/20); // keep duplicate from your original
  ctx.lineTo(c + a/8,  c - a/7);
  ctx.lineTo(c - a/8,  c - a/7);
  ctx.lineTo(c - a/5,  c - a/20);
  ctx.lineTo(c - a/5,  c - a/20);
  ctx.lineTo(c - a/2,  c - a/20);
  ctx.lineTo(c - a/2,  c);
  ctx.closePath();

  ctx.fillStyle = '#ffffffff';
  ctx.fill();
  ctx.strokeStyle = '#ffffffff';
  ctx.lineWidth = 0 / (camera.zoom * ppm); // keep strokes ~2px on screen
  ctx.stroke();

  ctx.restore();
}

function renderWorld(camera, viewportPx, drone, target) {
  beginCamera(camera, viewportPx);
  drawGridWorld(camera, viewportPx);
  drawGroundWorld(camera, viewportPx);

  // draw target behind the drone
  if (target) drawTargetWorld(target.x, target.y, camera);

  drawDroneWorld(drone.x, drone.y, drone.ang, camera);
  endCamera();
}

// ——— UI panels (screen space) ———
function drawInsetPanel(ix, iy, size) {
  ctx.save();
  // Opaque background — same color as the main view background
  ctx.beginPath();
  ctx.rect(ix, iy, size, size);
  ctx.fillStyle = '#191919ff'; // ← background color (set same as your page/canvas bg)
  ctx.fill();

  // Border
  ctx.beginPath();
  ctx.rect(ix - 2, iy - 2, size + 4, size + 4);
  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(0,0,0,0.35)';
  ctx.stroke();
  ctx.restore();
}

// ——— Inset config ———
const INSET_SIZE_PX = 170;
const INSET_PADDING = 12;
const INSET_ZOOM    = 2.5;

// ——— Frame render ———
function drawFrame(drone, target) {
  // Clear whole canvas to background color
  ctx.fillStyle = '#191919ff'; // ← same as inset bg
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  const dpr = window.devicePixelRatio || 1;
  const W = canvas.width / dpr;
  const H = canvas.height / dpr;

  // MAIN CAMERA
  const mainCam = {
    x: 0.3 * drone.x,
    y: 0.3 * drone.y + 0.3,
    zoom: 1.5
  };
  const mainViewport = { x: 0, y: 0, w: W, h: H };
  renderWorld(mainCam, mainViewport, drone, target);

  // INSET CAMERA
  const size = Math.min(INSET_SIZE_PX, Math.min(W, H) - 2 * INSET_PADDING);
  const ix = W - size - INSET_PADDING;
  const iy = INSET_PADDING;

  drawInsetPanel(ix, iy, size);

  const insetCam = {
    x: drone.x,
    y: drone.y,
    zoom: INSET_ZOOM
  };
  const insetViewport = { x: ix, y: iy, w: size, h: size };
  renderWorld(insetCam, insetViewport, drone, target);
}

// ——— Fixed-step sim loop ———
let last = performance.now();
let acc = 0;

function tick(now) {
  let dt = (now - last) / 1000;
  last = now;
  if (dt > 0.25) dt = 0.25;
  acc += dt;

  const Y_pos = (keys.up ? +Y_STEP : 0) + (keys.down ? -Y_STEP : 0);
  const X_pos = (keys.right ? X_STEP : 0) + (keys.left ? -X_STEP : 0);

  while (acc >= DT) {
    sim_step(0.6 * X_pos, 0.3 * Y_pos + 0.5);
    acc -= DT;
  }

  const drone = {
    x:   drone_get_x(),
    y:   drone_get_y(),
    ang: drone_get_ang()
  };

  // Use the exact same mapping the sim sees so the '+' matches the control input
  const target = {
    x: 0.6 * X_pos,
    y: 0.3 * Y_pos + 0.5
  };

  drawFrame(drone, target);
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
