#!/bin/bash
if ! gcc -Wall -g -O0 -o mrp.bin ukf_mrp.c kalman.c matrix_util.c quaternion_util.c -lgsl -lgslcblas -lm
then
	echo "Compilation failed."
	exit 1
fi
if ! valgrind --error-exitcode=1 --track-origins=yes --log-file=mrp_err.txt --tool=memcheck -- ./mrp.bin > mrp_out.txt
then
	echo "Valgrind returned error(s)."
fi
