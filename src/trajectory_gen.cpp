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

vector<double> generate_points_using_poly(vector<double> coeffs, int N, double time_interval){
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

vector<double> get_end_vals(vector<double> start, int N, double dt, 
	double max_speed, double max_accl, double max_jerk) {
	//vector<double> tmp;
	double x = start[0];
	//tmp.push_back(x);
	double v = start[1];
	double a = start[2];
	for(int i=0; i<N; i++) {
		x += v*dt;
		//tmp.push_back(x);

		// compute speed for next iteration
		v += a*dt;
		if (v > max_speed) {			
			v = max_speed;
		}

		// compute accleration for next iteration
		double a2 = (max_speed - v) / dt;
		if (abs(a2) > max_accl) {
			int sign = 1;
			if (a2 < 0) 
				sign = -1;
			a2 = max_accl * sign;
		}

		double jerk = (a2 - a) / dt;
		if (abs(jerk) > max_jerk ) {
			int sign = 1;
			if (jerk < 0)
				sign = -1;
			jerk = max_jerk * sign;
			a2 = a + jerk * dt;
		}
		a = a2;		
	}


	vector<double> end(3);
	end[0] = x;
	end[1] = v;
	end[2] = a;
	return end;
}