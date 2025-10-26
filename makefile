# Minimal Makefile (Git Bash / MSYS2)
SHELL := bash
EMSDK := D:/Programs/Emscripten/emsdk
OUT   := docs/sim.js

CFLAGS := -s WASM=1 -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT=web \
  -s EXPORTED_FUNCTIONS='["_sim_init","_sim_step","_drone_get_x","_drone_get_y","_drone_get_angle"]' \
  -s EXPORTED_RUNTIME_METHODS='["cwrap"]'

.PHONY: all clean

all:
	@. "$(EMSDK)/emsdk_env.sh" >/dev/null 2>&1 && \
	emcc sim/*.c $(CFLAGS) -o "$(OUT)"
	@echo "Build complete: $(OUT)"

clean:
	@rm -f "$(OUT)"