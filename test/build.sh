#!/bin/bash

if ! gcc -o test.bin -g -Wall -O3 ukf_test.c src/error/error_log.c src/kalman/ukf_mrp.c src/kalman/kalman.c src/math/quaternion_util.c src/math/matrix_util.c -lm -lpthread -lgsl -lgslcblas
then
	echo "Compilation failed."
	exit 1
fi
if ! valgrind --error-exitcode=1 --track-origins=yes --log-file=main_error.txt --tool=memcheck -- bin/test.bin > /dev/null
then
	echo "Valgrind returned error(s). NOTE: This means that either valgrind found problems OR the program itself returned a non-zero value."
fi
