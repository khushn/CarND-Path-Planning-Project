#ifndef trajectory_gen_h
#define trajectory_gen_h

#include <vector>

using namespace std;

vector<double> generate_trajectory(double s0, double ds0, double d2s0, 
								   double sf, double dsf, double d2sf);
#endif