// main.js — simple 2-part rotation demo (reuses existing <canvas id="c">)

document.body.style.margin = 0;
document.body.style.background = "#111";
document.body.style.display = "flex";
document.body.style.justifyContent = "center";
document.body.style.alignItems = "center";
document.body.style.height = "100vh";

const canvas = document.getElementById("c");
canvas.width = 1000;
canvas.height = 400;
canvas.style.border = "1px solid #333";
canvas.style.background = "#222";

const ctx = canvas.getContext("2d");

let imgPlane, imgElevator;
let planeAngle = 0;
let elevatorAngle = 0;

// Choose the CG in PLANE LOCAL COORDS (origin = plane image center, units = image px)
let cgLocal = { x: 0, y: 100 }; // <-- set this later to the aircraft point you want

// Keep aircraft roughly centered by pinning CG to this screen pivot
const screenPivot = { x: canvas.width / 2, y: canvas.height / 2 };

// Original elevator pivot (in plane-local coords relative to plane CENTER)
const elevatorPivot = { x: -665, y: 125 };

function loadImage(src) {
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve(img);
    img.onerror = reject;
    img.src = src;
  });
}

async function loadAll() {
  imgPlane = await loadImage("./F16_noElevator.svg");
  imgElevator = await loadImage("./F16_onlyElevator.svg");
  drawScene();
}

function drawScene() {
  const totalScale = 0.3;
  const { width, height } = canvas;
  ctx.clearRect(0, 0, width, height);

  // --- draw aircraft, rotating about CG ---
  ctx.save();
  ctx.translate(screenPivot.x, screenPivot.y);
  ctx.scale(totalScale, totalScale);
  ctx.rotate((planeAngle * Math.PI) / 180);

  // draw plane so that cgLocal is at the origin
  ctx.drawImage(
    imgPlane,
    -imgPlane.width / 2 - cgLocal.x,
    -imgPlane.height / 2 - cgLocal.y
  );

  // draw elevator: pivot relative to new origin (subtract cgLocal)
  ctx.save();
  ctx.translate(elevatorPivot.x - cgLocal.x, elevatorPivot.y - cgLocal.y);
  ctx.rotate((elevatorAngle * Math.PI) / 180);
  ctx.drawImage(
    imgElevator,
    -imgElevator.width / 2,
    -imgElevator.height / 2
  );
  ctx.restore();

  ctx.restore();

  // --- draw CG marker on TOP ---
  ctx.beginPath();
  ctx.arc(screenPivot.x, screenPivot.y, 6, 0, Math.PI * 2);
  ctx.fillStyle = "yellow";
  ctx.strokeStyle = "#aa0";
  ctx.lineWidth = 2;
  ctx.fill();
  ctx.stroke();
}

function setPlaneAngle(deg) {
  planeAngle = deg;
  drawScene();
}

function setElevatorAngle(deg) {
  elevatorAngle = deg;
  drawScene();
}

// call this to "pick the CG" on the aircraft (in plane image px, origin = plane center)
function setCGLocal(x, y) {
  cgLocal = { x, y };
  drawScene();
}

window.addEventListener("keydown", (e) => {
  switch (e.key) {
    case "ArrowLeft":  setPlaneAngle(planeAngle - 10); break;
    case "ArrowRight": setPlaneAngle(planeAngle + 10); break;
    case "ArrowUp":    setElevatorAngle(elevatorAngle + 20); break;
    case "ArrowDown":  setElevatorAngle(elevatorAngle - 20); break;
  }
});

loadAll();
