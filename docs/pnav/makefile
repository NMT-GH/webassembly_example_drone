# Minimal Makefile (Git Bash / MSYS2)
SHELL := bash
EMSDK := D:/Programs/Emscripten/emsdk
OUT   := pnav_page/sim.js

CFLAGS := -s WASM=1 -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT=web \
  -s EXPORTED_FUNCTIONS='["_sim_init","_sim_step","_get_interceptor_pos_x", "_get_interceptor_pos_y", "_get_interceptor_kin_ang", "_get_target_pos_x", "_get_target_pos_y", "_get_target_kin_ang"]' \
  -s EXPORTED_RUNTIME_METHODS='["cwrap"]'

.PHONY: all clean

all:
	@. "$(EMSDK)/emsdk_env.sh" >/dev/null 2>&1 && \
	emcc sim/*.c $(CFLAGS) -o "$(OUT)"
	@echo "Build complete: $(OUT)"

clean:
	@rm -f "$(OUT)"



