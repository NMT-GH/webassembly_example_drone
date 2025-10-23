@echo off
rem Build script for WebAssembly
call "D:\Programs\Emscripten\emsdk\emsdk_env.bat" >nul 2>&1

emcc sim\box.c -O3 ^
  -s WASM=1 ^
  -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT=web ^
  -s EXPORTED_FUNCTIONS="["_sim_init","_sim_step","_drone_get_x","_drone_get_y","_drone_get_angle"]" ^
  -s EXPORTED_RUNTIME_METHODS="["cwrap"]" ^
  -o site\sim.js

echo Build complete: site\sim.jsb