// for testing mean state function
#include "src/kalman/ukf_mrp.h"
#include "src/math/matrix_util.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define BASE_ANGLE 1.25*M_PI

double g_north[3];
double g_down[3];

double rand_gauss()
// returns a guassian distributed random number with mean 0 and standard deviation 1
// assumes srand() has already been seeded
{
	double u1 = (double)rand() / (double) RAND_MAX;
	double u2 = (double)rand() / (double) RAND_MAX;
	return sqrt(-2*log(u1))*cos(2*M_PI*u2);
}

int main()
{
	int i;
	double base_mrp[3] = {1, 0, 0};
	double points[10][6];
	double weights[10] = { [0 ... 9] = 0.1 };

	srand(1);

	for (i = 0; i < 10; i++) {
		double angle = BASE_ANGLE + 0.1*rand_gauss();
		matrix_scale(base_mrp, points[i], tan(angle/4), 3, 1);
		matrix_init(points[i] + 3, 0, 3, 1);
		printf("{%f, %f, %f}\n", points[i][0], points[i][1], points[i][2]);
	}

	double mean_mrp[6];
	mean_state((double*)points, weights, mean_mrp, 6, 10);
	printf("MEAN:\n{%f, %f, %f}\n", mean_mrp[0], mean_mrp[1], mean_mrp[2]);

	return 0;
}
