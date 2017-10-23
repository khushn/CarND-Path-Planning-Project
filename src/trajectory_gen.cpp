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
		double val = coeffs[0] + x * coeffs[1] + x*x*coeffs[2] + pow(x, 3)* coeffs[3] +
			pow(x, 4) * coeffs[4] + pow(x, 5) * coeffs[5];
		vals.push_back(val);
		//cout << "In generate_points_using_poly(), val: " << val << endl;

	}
	return vals;
}

double getCeilVal(double val, double max) {
	if (abs(val) > max) {
		int sign = 1;
		if (val < 0) 
			sign = -1;
		val = max * sign;
	}
	return val;
}

double normalize_angle(double turn) {
	while(turn > M_PI) {
		turn-=2*M_PI;
	}
	while(turn < -M_PI) {
		turn+=2*M_PI;
	}
	return turn;
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
		v += a*dt;
		//tmp.push_back(x);

		
		// compute accleration for next iteration
		double a2 = (max_speed - v) / dt;
		if (abs(a2) > max_accl) {
			a2 = getCeilVal(a2, max_accl);			
		}

		double jerk = (a2 - a) / dt;
		if (abs(jerk) > max_jerk) {
			jerk = getCeilVal(jerk, max_jerk);		
			a2 = a + jerk * dt;
			a2 = getCeilVal(a2, max_accl);			
		}
		a = a2;				
	}


	vector<double> end(3);
	end[0] = x;
	end[1] = v;
	end[2] = a;
	return end;
}

vector<double> get_lane_trajectory(
	double *last_pt_v, double *last_pt_a,
	vector<double> start, int N, double dt, 
	double max_speed, double max_accl, double max_jerk) {
	vector<double> ret;
	double x = start[0];
	//ret.push_back(x);
	double v = start[1];
	double a = start[2];
	for(int i=0; i<N; i++) {
		x += v*dt;
		v += a*dt;
		ret.push_back(x);

		/**
		// compute speed for next iteration
		double v2 = v + a*dt;
		v2 = getCeilVal(v2, max_speed);	
		**/	

		// compute accleration for next iteration
		double a2 = (max_speed - v) / dt;
		if (abs(a2) > max_accl) {
			a2 = getCeilVal(a2, max_accl);
			//v2 = v + a2*dt;
			//v2 = getCeilVal(v2, max_speed);
		}

		double jerk = (a2 - a) / dt;
		if (abs(jerk) > max_jerk) {
			jerk = getCeilVal(jerk, max_jerk);		
			a2 = a + jerk * dt;
			a2 = getCeilVal(a2, max_accl);
			//v2 = v + a2*dt;	
			//v2 = getCeilVal(v2, max_speed);
		}
		a = a2;		
		//v = v2;
	}
	*last_pt_v = v;
	*last_pt_a = a;
	return ret;
}

void clean_normal_jerk(double *last_pt_v, double *last_pt_a, double prev_angle,
	vector<vector<double>>& xy_list, 
	double prev_x, double prev_y, double dt, 
	double max_speed, double max_accl, double max_jerk){	
	double prev_speed = *last_pt_v;
	double prev_accl = *last_pt_a;
	double prev_normal_acc = 0.;
	for(int i=0; i<xy_list.size(); i++) {		
		double x = xy_list[i][0];
		double y = xy_list[i][1];
		cout << "clean_normal_jerk() pts: " << x << ", " << y << endl;
		double dy = y - prev_y;
		double dx = x - prev_x;
		double s = sqrt(dx*dx+dy*dy);
		double angle = atan2(dy, dx);

		// check for acceleration in the direction of movement
		double v = s/dt;
		double accl = (v - prev_speed) / dt;
		double jerk = (accl - prev_accl) / dt;

		bool recompute=false;
		float small_val = .0000001; // needed to be added for floating point comparison
		if (abs(v) > max_speed + small_val) {
			cout << "VIOLATION -- speed " << v << endl;
			v = getCeilVal(v, max_speed);
			recompute = true;
		}

		
		if (abs(accl) > max_accl + small_val) {
			cout << "VIOLATION -- accl " << accl << endl;
			accl = getCeilVal(accl, max_accl);
			v = prev_speed + accl * dt;
			recompute = true;
		}

		
		if ( abs(jerk) > max_jerk + small_val) {
			cout << "VIOLATION -- jerk " << jerk << endl;
			jerk = getCeilVal(jerk, max_jerk);
			accl = prev_accl + jerk * dt;
			v = prev_speed + accl * dt;
			recompute = true;
		}
		

		// Even if any one of the above changes recompute x, y
		// adding a small value to avoid floating point comparison erro
		if (recompute) {
			double ns = v*dt;
			cout << "corrected dist from " << s << " to " << ns << endl;
			s = ns;
			x = prev_x + s * cos(angle);
			y = prev_y + s * sin(angle);			
			cout << "corrected x, y: " << x << ", " << y << endl;
			
		}


		
		double turn = normalize_angle(angle - prev_angle);
		

		// note normal veloicty and acceleration are same and equal
		// as we aleays use it with (only) the turn
		double normal_vel = s*sin(turn) / dt;

		/**
		//cout << "normal_vel: " << normal_vel << endl;
		if (abs(normal_vel) > max_accl + small_val) {
			cout << "VIOLATION -- normal accl exceeded limit " << normal_vel << endl;
			cout << "prev_angle: " << prev_angle << ". angle: " << angle << endl;
			
			// we can compare straight away as its only for the turn and so this component 
			// was '0' in the previous segment
			normal_vel = getCeilVal(normal_vel, max_accl);
			double new_turn = normalize_angle(asin(normal_vel*dt/s));
			cout << "corrected turn from " << turn << " to " << new_turn << endl;

			
			// put in corrected values
			angle = prev_angle + new_turn;
			x = prev_x + s*cos(angle);
			y = prev_y + s*sin(angle);
			cout << "corrected x, y: " << x << ", " << y << endl;
			
		}
		

		
		
		// check for normal jerk
		double normal_jerk = normal_vel / dt;
		if (abs(normal_jerk) > max_jerk + small_val) {
			cout << "VIOLATION -- normal jerk exceeded limit " << normal_jerk << endl;
			normal_jerk = getCeilVal(normal_jerk, max_jerk);
			normal_vel = normal_jerk * dt;
			normal_vel = getCeilVal(normal_vel, max_accl);
			double new_turn = normalize_angle(asin(normal_vel*dt/s));
			cout << "corrected turn from " << turn << " to " << new_turn << endl;

			// put in corrected values
			angle = prev_angle + new_turn;
			x = prev_x + s*cos(angle);
			y = prev_y + s*sin(angle);
			cout << "corrected x, y: " << x << ", " << y << endl;	
		}
		**/
		
		
		
		// reassign the value in case it is changed
		xy_list[i][0] = x;
		xy_list[i][1] = y;
		// recaluclate angle for next iteration
		dy = y - prev_y;
		dx = x - prev_x;
		angle = atan2(dy, dx);

		prev_angle  = angle;
		prev_speed = v;
		prev_accl = accl;
		//prev_normal_acc = normal_vel;
		prev_x = xy_list[i][0];
		prev_y = xy_list[i][1];
	}
	*last_pt_v = prev_speed;
	*last_pt_a = prev_accl;
}


void get_speed_accleration(double *last_pt_v, double *last_pt_a, 	
	double dt, double max_speed, double max_accl, double max_jerk) {

	double prev_speed = *last_pt_v;
	double prev_accl = *last_pt_a;

	double accl = (max_speed - prev_speed) / dt;
	if (abs(accl) > max_accl) {
		accl = getCeilVal(accl, max_accl);
	}

	double jerk = (accl - prev_accl)/dt;
	if (abs(jerk) > max_jerk) {
		jerk = getCeilVal(jerk, max_jerk);
		accl = prev_accl + jerk*dt;
		accl = getCeilVal(accl, max_accl);
	}	
	*last_pt_v += accl * dt;
	*last_pt_v = getCeilVal(*last_pt_v, max_speed);
	*last_pt_a = accl;
}