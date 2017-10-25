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
Given a lane changee requirement generate the poly of 'd' for it
**/
vector<double> get_lane_change_poly(int from_lane, int to_lane, double dt, double time_limit);

/**
Can we change lanes to a given lane. 
Check for future positions of cars in that lane, and see if there is a clash. 
Return false early, if there is
**/
bool can_change_lane_to(vector<vector<double>> cars_in_lane, double my_speed, double car_ahead_speed, 
												double cur_s, double time_limit, 
												int prev_path_size, double dt);
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
vector<double> get_lane_trajectory( double *last_pt_v, double *last_pt_a,
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

void get_speed_accleration(double *last_pt_v, double *last_pt_a, 	
	double dt, double max_speed, double max_accl, double max_jerk);

int get_lane_from_d(double d);

/**
get the list of distance fractions covereed in each time unit dt
*/
vector<double> get_distance_fractions(double *last_pt_v, double *last_pt_a, 	
	double dist, int N, double dt, double target_speed, 
	double max_speed, double max_accl, double max_jerk);


#endif