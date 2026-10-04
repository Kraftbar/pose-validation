# Build the IMU leaf tests (standalone, does not touch the sv_run Makefile). Usage: make -f sv_imu.mk -C stella_vio/c check
OK := ../../okvis_port/c
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -ffp-contract=off -fno-fast-math
BIN := ../../runs/stella_vio/imu/bin
LIB := sv_imu.c sv_imu_gyro.c
$(BIN)/check_sv_imu: check_sv_imu.c $(LIB) sv_imu.h $(OK)/ok_imu.c $(OK)/ok_eigen.c $(OK)/ok_time.c
	mkdir -p $(BIN)
	gcc $(CFLAGS) -I$(OK) -o $@ check_sv_imu.c $(LIB) $(OK)/ok_imu.c $(OK)/ok_eigen.c $(OK)/ok_time.c -lm
$(BIN)/check_sv_imu_init: check_sv_imu_init.c $(LIB) sv_imu_init.c sv_imu_init.h sv_imu.h
	mkdir -p $(BIN)
	gcc $(CFLAGS) -o $@ check_sv_imu_init.c $(LIB) sv_imu_init.c -lm
$(BIN)/check_sv_imu_euroc: check_sv_imu_euroc.c $(LIB) sv_imu.h
	mkdir -p $(BIN)
	gcc $(CFLAGS) -o $@ check_sv_imu_euroc.c $(LIB) -lm
$(BIN)/sv_imu_gyro_run: sv_imu_gyro_run.c $(LIB) sv_imu_init.c sv_imu_init.h sv_imu.h
	mkdir -p $(BIN)
	gcc $(CFLAGS) -o $@ sv_imu_gyro_run.c $(LIB) sv_imu_init.c -lm
check: $(BIN)/check_sv_imu
	$(BIN)/check_sv_imu
$(BIN)/check_sv_imu_initsyn: check_sv_imu_initsyn.c $(LIB) sv_imu_init.c sv_imu_init.h sv_imu.h
	mkdir -p $(BIN)
	gcc $(CFLAGS) -o $@ check_sv_imu_initsyn.c $(LIB) sv_imu_init.c -lm
checksyn: $(BIN)/check_sv_imu_initsyn
	$(BIN)/check_sv_imu_initsyn
