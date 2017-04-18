#!/bin/bash
set -e
gcc -Wall -g -O0 ukf_test.c kalman.c matrix_util.c quaternion_util.c -lgsl -lgslcblas -lm
valgrind --track-origins=yes --log-file=test_error.txt --tool=memcheck ./a.out > test_out.txt
