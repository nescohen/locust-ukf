#!/bin/bash

gcc -o test.bin -Wall -O3 boardutil.c flight-control.c flight-input.c error_log.c pid/pid.c -lm -lpthread
