// site/main.js (ES module)
import createModule from './sim.js';

const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');

// Load WASM (built with -s MODULARIZE=1 -s EXPORT_ES6=1)
const Module = await createModule();

// Bind C exports
const sim_init      = Module.cwrap('sim_init', 'number', ['number']);          // uint8 -> number
const sim_step      = Module.cwrap('sim_step', null, ['number', 'number']);    // (thr, steer)
const drone_get_x   = Module.cwrap('drone_get_x', 'number', []);
const drone_get_y   = Module.cwrap('drone_get_y', 'number', []);
const drone_get_ang = Module.cwrap('drone_get_angle', 'number', []);

// --- Sim/controls setup ---
const DT = 0.01;                     // fixed integrator step (s) — matches your C dt
const ok = sim_init(DT);
if (ok !== 0) throw new Error('sim_init failed');

let keys = { up:false, down:false, left:false, right:false };

// Basic input mapping: up/down -> thrust, left/right -> steer (torque)
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

// Thrust/steer scaling — tune to taste
const THRUST_STEP = 1.5;  // N-equivalent command scaling (your C turns this into per-prop thrust)
const STEER_STEP  = 1.0;  // torque-like steering command scaling (unitless here; you scale inside sim)

// World->screen helpers (meters -> pixels)
const ppm = 60;                                // pixels per meter
const origin = { x: canvas.width/2, y: canvas.height*0.75 }; // put ground near bottom
function worldToScreen(x, y) {
  return {
    sx: origin.x + x * ppm,
    sy: origin.y - y * ppm   // invert y (canvas y grows down)
  };
}

// Draw a simple “drone” rectangle centered at (x,y) rotated by angle (rad)
function drawDrone(x, y, ang) {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  // ground line
  ctx.beginPath();
  ctx.moveTo(0, origin.y);
  ctx.lineTo(canvas.width, origin.y);
  ctx.strokeStyle = '#666';
  ctx.lineWidth = 1;
  ctx.stroke();

  const { sx, sy } = worldToScreen(x, y);

  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-ang); // canvas y-down; if your angle is “0=up, left positive”, this keeps nose-up as 0
  // Body (0.25 m wide, 0.05 m tall)
  const bw = 0.25 * ppm;
  const bh = 0.05 * ppm;
  ctx.fillStyle = '#66d9ef';
  ctx.fillRect(-bw/2, -bh/2, bw, bh);

  // Arms / props (visual only)
  ctx.fillStyle = '#fff';
  const arm = 0.127 * ppm; // your prop-to-prop (approx)
  ctx.beginPath();
  ctx.arc(+arm/2, 0, 4, 0, Math.PI*2);
  ctx.arc(-arm/2, 0, 4, 0, Math.PI*2);
  ctx.fill();
  ctx.restore();
}

// Fixed-step sim loop decoupled from render
let last = performance.now();
let acc = 0;

function tick(now) {
  let dt = (now - last) / 1000;
  last = now;
  if (dt > 0.25) dt = 0.25;
  acc += dt;

  // Input shaping
  const thrust = (keys.up ? +THRUST_STEP : 0) + (keys.down ? -THRUST_STEP : 0);
  const steer  = (keys.right ? -STEER_STEP  : 0) + (keys.left ? +STEER_STEP  : 0);

  // Step sim in fixed quanta
  while (acc >= DT) {
    sim_step(thrust, steer);
    acc -= DT;
  }

  // Read and draw
  const x   = drone_get_x();
  const y   = drone_get_y();
  const ang = drone_get_ang(); // radians
  drawDrone(x, y, ang);

  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);