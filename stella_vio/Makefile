# stella_vio: standalone build of the sv_run driver. C99, no libraries besides libm.
SRC := $(shell head -1 c/sv_run.c | sed 's/.*SV_PORT_SOURCES: *//; s/ *\*\/.*//')
CFLAGS ?= -O2 -ffp-contract=off -fno-fast-math
sv_run: $(addprefix c/,$(SRC)) $(wildcard c/*.h)
	gcc -std=c99 $(CFLAGS) -o $@ $(addprefix c/,$(SRC)) -lm
asan:
	gcc -std=c99 -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer -ffp-contract=off -o sv_run_asan $(addprefix c/,$(SRC)) -lm
clean:
	rm -f sv_run sv_run_asan
# unit/benchmark harness of the PoseLib-style blocks (c/sv_poselib.c); needs runs/blocks/poselib/pairs.txt (tools/blocks/make_pairs.py): ./check_sv_poselib ../runs/blocks/poselib/pairs.txt
CHK_SRC := sv_init.c sv_solve_homography.c sv_solve_fundamental.c sv_solve_essential.c sv_solve_essential_5pt.c sv_solve_essential_ransac.c sv_solve_common.c sv_triangulate.c sv_poselib.c sv_pnp.c sv_eigen_pnp.c sv_eigen_svd.c sv_eigen_qr.c sv_linalg.c sv_rng.c sv_eigen_eigensolver.c sv_eigen_fullpivlu.c sv_eigen_lu3.c sv_eigen_llt.c sv_eigen_mat4.c sv_eigen_quaternion.c
check_sv_poselib: c/check_sv_poselib.c $(addprefix c/,$(CHK_SRC)) $(wildcard c/*.h)
	gcc -std=c99 $(CFLAGS) -D_POSIX_C_SOURCE=199309L -o $@ c/check_sv_poselib.c $(addprefix c/,$(CHK_SRC)) -lm
