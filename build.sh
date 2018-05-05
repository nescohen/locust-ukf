#!/bin/bash

SOURCE_FILES="src/main.c src/hardware/boardutil.c src/error/error_log.c src/pid/pid.c src/kalman/ukf_mrp.c src/kalman/kalman.c src/math/quaternion_util.c src/math/matrix_util.c src/nav/navigation.c src/client/client.c src/stop.c"

if ! gcc -o bin/test.bin -g -Wall -O3 -D DEBUG $SOURCE_FILES -lm -lpthread -lgsl -lgslcblas
then
	echo "Compilation failed."
	exit 1
fi
#if ! valgrind --error-exitcode=1 --track-origins=yes --log-file=main_error.txt --tool=memcheck -- bin/test.bin > /dev/null
#then
#	echo "Valgrind returned error(s). NOTE: This means that either valgrind found problems OR the program itself returned a non-zero value."
#fi
