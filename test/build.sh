#!/bin/bash

if ! gcc -o test.bin -g -Wall -O3 -DDEBUG ukf_test.c src/error/error_log.c src/kalman/ukf_mrp.c src/kalman/kalman.c src/math/quaternion_util.c src/math/matrix_util.c -lm -lpthread -lgsl -lgslcblas
then
	echo "Compilation failed."
	exit 1
fi
