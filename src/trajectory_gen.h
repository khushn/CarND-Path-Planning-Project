#ifndef trajectory_gen_h
#define trajectory_gen_h

#include <vector>

using namespace std;

/**
We find the coefficients of the polynomial such that, all coefficients for degree > 5 are 0. 
in the polynomial. Such a polynomial is smooth having less jerks
**/
vector<double> generate_poly_coefficients(vector< double> start, vector <double> end, double T);

vector<double> generate_points_using_poly(vector<double> coeffs, double time_interval=.020, int N=50);
#endif