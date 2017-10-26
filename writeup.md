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

</code>


1. We set the accleration to 9 m/s^2. Else it mysteriously goes above threshold in console. 
2. The function get_distance_fractions() is used to get the points along the distance (s) frenet, based on appropriate velocity, acceleration and jerk values.
3. It is observed that Spline gives a better result for lane change as well, compared to generating poly coefficients using 'Quintic Polynomial Solver'
4. Using a cost model, for lane changing. Calculate the cost of lane change. Higher cost for crowded lane with lower average speed.




##### TODO
1. Record a video of car running 