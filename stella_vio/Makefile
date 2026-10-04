# stella_vio: standalone build of the sv_run driver. C99, no libraries besides libm.
SRC := $(shell head -1 c/sv_run.c | sed 's/.*SV_PORT_SOURCES: *//; s/ *\*\/.*//')
CFLAGS ?= -O2 -ffp-contract=off -fno-fast-math
sv_run: $(addprefix c/,$(SRC)) $(wildcard c/*.h)
	gcc -std=c99 $(CFLAGS) -o $@ $(addprefix c/,$(SRC)) -lm
asan:
	gcc -std=c99 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -ffp-contract=off -o sv_run_asan $(addprefix c/,$(SRC)) -lm
clean:
	rm -f sv_run sv_run_asan
