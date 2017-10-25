### Path Planning Project notes


1. We set the accleration to 9 m/s^2. Else it mysteriously goes above threshold in console. 
2. The function get_distance_fractions() is used to get the points along the distance (s) frenet, based on appropriate velocity, acceleration and jerk values.
3. It is observed that Spline gives a better result for lane change as well, compared to generating poly coefficients using 'Quintic Polynomial Solver'
4. Using a cost model, for lane changing. Calculate the cost of lane change. Higher cost for crowded lane with lower average speed.




##### TODO
1. Record a video of car running 