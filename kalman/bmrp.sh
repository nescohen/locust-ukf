#!/bin/bash
set -e
gcc -Wall -g -O0 -o mrp.bin ukf_mrp.c kalman.c matrix_util.c quaternion_util.c -lgsl -lgslcblas -lm
valgrind --track-origins=yes --log-file=mrp_err.txt --tool=memcheck -- ./mrp.bin > mrp_out.txt
