#!/bin/bash
set -e
gcc -Wall -g -O0 -o ukf_quat.bin ukf_test.c kalman.c matrix_util.c quaternion_util.c -lgsl -lgslcblas -lm
valgrind --track-origins=yes --log-file=test_err.txt --tool=memcheck -- ./ukf_quat.bin > test_out.txt
