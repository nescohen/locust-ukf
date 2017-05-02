#!/bin/bash

if ! gcc -o bin/test.bin -Wall -O3 src/hardware/boardutil.c src/flight-control.c src/hardware/flight-input.c src/error/error_log.c src/pid/pid.c -lm -lpthread
then
	echo "Compilation failed."
	exit 1
fi
if ! valgrind --error-exitcode=1 --track-origins=yes --log-file=main_error.txt --tool=memcheck -- bin/test.bin > main_out.txt
then
	echo "Valgrind returned error(s)."
fi
