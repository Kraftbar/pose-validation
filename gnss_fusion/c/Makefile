# SPDX-License-Identifier: MIT (own code)
CC ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -pedantic
all: gf_run gf_gait_run gf_georef_run
gf_run: gf_run.c gf_fusion.c gf_geo.c gf_fusion.h gf_geo.h gf_math.h
	$(CC) $(CFLAGS) -o $@ gf_run.c gf_fusion.c gf_geo.c -lm
gf_gait_run: gf_gait_run.c gf_gait.c gf_gait.h
	$(CC) $(CFLAGS) -o $@ gf_gait_run.c gf_gait.c -lm
gf_georef_run: gf_georef_run.c gf_georef.c gf_georef.h
	$(CC) $(CFLAGS) -o $@ gf_georef_run.c gf_georef.c -lm
clean:
	rm -f gf_run gf_gait_run gf_georef_run
