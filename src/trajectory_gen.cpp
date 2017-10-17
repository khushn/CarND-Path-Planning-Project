#include "trajectory_gen.h"

#include <iostream>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU" // For matrix inverse()

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
Implementation notes: 
See Udacity SDCND Term 3, Lesson 5, Implement Quintic Polynomial Solver C++
 for implementation technique
**/
vector<double> generate_poly_coefficients(vector< double> start, vector <double> end, double T){

	double a0 = start[0];
	double a1 = start[1];
	double a2 = .5 * start[2];
	MatrixXd m(3, 3);
	m << pow(T, 3) , pow(T, 4), pow(T, 5),
		 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
		 6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
	VectorXd rhs(3);
	rhs << end[0] - (start[0] + start[1]*T + .5 * start[2]* T*T), 
		   end[1] - (start[1] + start[2]*T),
		   end[2] - start[2];

	MatrixXd m_inv = m.inverse();
	VectorXd coeffs =  m_inv * rhs;
	return {a0, a1, a2, coeffs[0], coeffs[1], coeffs[2] };
}

vector<double> generate_points_using_poly(vector<double> coeffs, double time_interval, 
	int N){
	vector<double> vals;
	if (coeffs.size() < 6) {
		cerr << "Error in generate_points_using_poly(). Insufficient coefficients in polynomial" 
			 << coeffs.size() << endl;
	}
	for(int i=0; i<N; i++) {
		double x = i*time_interval;
		vals.push_back(coeffs[0] + x * coeffs[1] + x*x*coeffs[2] + pow(x, 3)* coeffs[3] +
			pow(x, 4) * coeffs[4] + pow(x, 5) * coeffs[5]);

	}
	return vals;
}