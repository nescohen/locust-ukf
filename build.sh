#!/bin/bash

gcc -o test.bin boardutil.c flight-control.c flight-input.c error_log.c -lm -lpthread
