#ifndef QUATERNION_UTIL_H
#define QUATERNION_UTIL_H

void gen_quaternion(double theta, double vect[3], double result[4]);
void decomp_quaternion(double quat[4], double vect[3]);
void mult_quaternion(double op_a[4], double op_b[4], double result[4]);
void generate_matrix(double q[4], double matrix[9]);
void axis_angle_matrix(double axis[3], double angle, double matrix[9]);
void matrix_to_euler(double matrix[9], double axis[3], double *angle);
void vector_by_matrix(double v[3], double r[9], double result[3]);
void vector_by_scalar(double v[3], double scalar, double result[3]);
void cross_product(double v_a[3], double v_b[3], double result[3]);
double dot_product(double v_a[3], double v_b[3]);
void add_vectors(double v_a[3], double v_b[3], double result[3]);
double vector_magnitude(double vector[3]);
void normalize_vector(double vector[3], double result[3]);
void normalize_quaternion(double q[4], double result[4]);
double convert_angle(double angle);

#endif
