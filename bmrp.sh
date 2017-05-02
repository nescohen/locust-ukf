#!/bin/bash
if ! gcc -Wall -g -O0 -o bin/mrp.bin src/ukf_mrp.c src/kalman/kalman.c src/math/matrix_util.c src/math/quaternion_util.c -lgsl -lgslcblas -lm
then
	echo "Compilation failed."
	exit 1
fi
if ! valgrind --error-exitcode=1 --track-origins=yes --log-file=mrp_err.txt --tool=memcheck -- bin/mrp.bin > mrp_out.txt
then
	echo "Valgrind returned error(s)."
fi
