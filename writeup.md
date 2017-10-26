### Path Planning Project notes and Model Documentation


#### Spline 
At first I tried to use <i>Quintic Polynomial Solver</i> for smoothing the path generated, but it was quite problematice. So I shifted to using Spline, as advised in the project walkthrough video. Using spline, immidiately gave a better path. 


#### Using of previous path
The other important trick is to utilize the previous_path_[xy] points given pack by the simulator. We should *always* only append to the previous path, and never create a fresh path. Attempting to create a fresh path, invariable results in a very jerky resultant path and car movement. 


#### Translation to car coordinates
Spline takes values in a 2-d coordinate system to do a smooth curve fitting. For e.g. say we have x and y axis, and x being the horizontal one. In that it expects x values to be ever increasing. Else it throws an exception. 

So since our x and y values could be in any direction, if we use them as is to fit using Spline, it may not work. So its better we transform the same to car coordiates. The same is done in main.cpp: 
* Conversion begining at line 614. 
* Spline fitting after that 
* And Conversion back to global cordinates, starting at line 671

#### Important thresolds
These are defined in main.cpp as constant values. Important ones are: 
<code>

	const double SPEED_LIM_MILES = 49.;

    const double MAX_SPEED = SPEED_LIM_MILES * 1.60934 * 1000 / (60 * 60);    

    const double MAX_ACCL = 6.0; // Putting it as 10 metre/sec^2, gives problems. So I needed to reduce it.

    const double MAX_JERK = 10.0; // 10 metre/sec^3

    const double TIME_DELTA = .020; // 20 milisec

    const int N = 50; // points into future

    BUFFER_DIST = 35.; // This is the distance in metres it uses to change lanes. Upper limit.

</code>


#### Core Model
  We just move the car in the starting lane, gradually reaching a speed of just below 50 mph. If the car ahead of us is moving slowly we explore lane change options. The core logic starts at line 417 in main.cpp . Starting with laying out some thresholds to use on the Sensor fusion data. 


<code> 

	const double CAR_AHEAD_RANGE = MAX_SPEED * 1;

    const double CAR_BEHIND_RANGE = -MAX_SPEED *1;
</code>

	The above describe the range ahead an behind we need to look for the cars. 

  Once we have that after checking the speed of the car immediately ahead of us, if needed we invoke the lane changing cost functions described below. 

##### Lane changing cost function
<code>
	
//Can we change lanes to a given lane. 

//Check for future positions of cars in that lane, and see if there is a clash. 
//Return false early, if there is, 

//-ve cost is false,

//if +ve value lane can be changed but may use the cost to compare with other actions

// The function definition is in trajectory_gen.cpp line 66.

	double cost_of_changing_lane_to(vector<vector<double>> cars_in_lane, 
												double my_speed, double car_ahead_speed, 
												double cur_s, double time_limit, 
												int prev_path_size, double dt);

												

</code>

#### Other notes
1. We first set the accleration to 9 m/s^2. Else it mysteriously goes above threshold in console. Finally had to keep it at 6 m/s^2. Which resulted in no violations of acceleration.
2. The function get_distance_fractions() is used to get the points along the distance (s) frenet, based on appropriate velocity, acceleration and jerk values.
3. It is observed that Spline gives a better result for lane change as well, compared to generating poly coefficients using 'Quintic Polynomial Solver'
4. Using a cost model, for lane changing. Calculate the cost of lane change. Higher cost for crowded lane with lower average speed.




#### TODO
1. Record a video of car running 


### References

1. Project walkthrough video, was very helpful. 
	https://www.youtube.com/watch?v=7sI3VHFPP0w

2. Good resource for transformation (shifting and rotation) of points in a 2-d plane to a different reference axis. 
https://www.math10.com/en/geometry/analytic-geometry/geometry1/coordinates-transformation.html