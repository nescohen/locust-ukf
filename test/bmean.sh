gcc -g -o mean.bin mean_test.c src/math/matrix_util.c src/math/quaternion_util.c src/kalman/ukf_mrp.c src/kalman/kalman.c -lm -lgsl -lgslcblas
