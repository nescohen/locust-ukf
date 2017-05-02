#!/bin/bash

if ! gcc -o bin/test.bin -Wall -O3 src/main.c src/hardware/boardutil.c src/hardware/flight-input.c src/error/error_log.c src/pid/pid.c src/kalman/ukf_mrp.c src/kalman/kalman.c src/math/quaternion_util.c src/math/matrix_util.c -lm -lpthread -lgsl -lgslcblas
then
	echo "Compilation failed."
	exit 1
fi
if ! valgrind --error-exitcode=1 --track-origins=yes --log-file=main_error.txt --tool=memcheck -- bin/test.bin > /dev/null
then
	echo "Valgrind returned error(s)."
fi
