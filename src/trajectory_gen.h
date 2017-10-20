#ifndef trajectory_gen_h
#define trajectory_gen_h

#include <vector>

using namespace std;

/**
We find the coefficients of the polynomial such that, all coefficients for degree > 5 are 0. 
in the polynomial. Such a polynomial is smooth having less jerks
**/
vector<double> generate_poly_coefficients(vector< double> start, vector <double> end, double T);

/**
Generate some future points of the polynomial
*/
vector<double> generate_points_using_poly(vector<double> coeffs, int N,  double time_interval);


/**
Get the end target distance, speed and acceleration. 
Given; Initial values
*/
vector<double> get_end_vals(vector<double> start, int N, double dt,
	double max_speed, double max_accl, double max_jerk);

/**
Get the lane trajectory which meets all constraints
Given; Initial values
*/
vector<double> get_lane_trajectory( 
	vector<double> start, int N, double dt,
	double max_speed, double max_accl, double max_jerk);


/** Make points move more smoothly, reduce turn at every point if normal accl/jerk 
above threshold
*/
void clean_normal_jerk(double *last_pt_v, double *last_pt_a, double prev_angle, 
						vector<vector<double>>& xy_list, 
						double prev_x, double prev_y,
						double dt, double max_speed, 
						double max_accl, double max_jerk);

#endif