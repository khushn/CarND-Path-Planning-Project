#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "trajectory_gen.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// previously sent last of speed and acceleration values
double prev_speed = 0.;
double prev_acc = 0.;
// The path length sent to simulator
int path_length_sent=0;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

/**
We need to get the begin and s positions of the vector of s
** to curve fit these ranges with x and y maps using spline 
*/
vector<int> getVectorPositions(double s_beg, double s_end, const vector<double> &maps_s) {
  int beg = 0;

  while(s_beg > maps_s[beg] && beg < maps_s.size()) {
    beg++;
  }

  beg=max(0, beg-1);
  int end = beg;
  while(s_end > maps_s[end] && end < maps_s.size()) {
    end++;
  }

  const int MIN_RANGE=6;
  int range = end - beg + 1;
  if (range < MIN_RANGE) {
    int diff = MIN_RANGE - range;
    beg = max(0, int(beg-diff/2));
    end = min(int(maps_s.size())-1, int(end+diff/2));
  }
  return {beg, end};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  /**
  // Fit s, x points using spline
  int lane = 1;
  vector<double> x_for_lane;
  vector<double> y_for_lane;
  for(int i=0; i<map_waypoints_s.size(); i++) {
    vector<double> tmp_xy = getXY(map_waypoints_s[i], lane*4.+2., map_waypoints_s, map_waypoints_x, map_waypoints_y);
    x_for_lane.push_back(tmp_xy[0]);
    y_for_lane.push_back(tmp_xy[1]);
  }
  tk::spline sx;  
  sx.set_points(map_waypoints_s, x_for_lane);

  // fit s, y points using spline
  tk::spline sy;  
  sy.set_points(map_waypoints_s, y_for_lane);
  **/

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            const double TARGET_SPEED = 22.0;
            //const double TARGET_SPEED = 22.22; // 22 metres/ sec translates to 50 miles/hr
            const double MAX_ACCL = 10.0; // 10 metre/sec^2
            const double MAX_JERK = 10.0; // 10 metre/sec^3
            const double TIME_DELTA = .020; // 20 milisec
            const int N = 50; // points into future

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

             // 1st convert the speed from miles/hr to metres / sec
            car_speed *= 1.60934 * 1000 / (60 * 60);
            cout << "----------------------------" << endl;
            cout << "car_speed: " << car_speed << endl;            
            //cout << "car_s: " << car_s << ", car_d: " << car_d <<
            //        ", end_path_s: "<< end_path_s << ", end_path_d: " << end_path_d << endl;
            
            int prev_path_size = previous_path_x.size();
            cout << "prev path_size: " << prev_path_size << endl; 

            // 2nd just add the remaining of previous path to next path 
            // for continuity
            for(int i=0; i<prev_path_size; i++) {              
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double prev_x=0.;
            double prev_y=0.;
            double prev_angle=0.;
            vector<double> start(3);
            double d; // Keep lane 
            vector<double> pts_x;
            vector<double> pts_y;
            if (prev_path_size < 2) {
              // first time 
              start[0] = car_s;
              prev_x = car_x;
              prev_y = car_y;
              prev_angle = deg2rad(car_yaw);
              d = car_d; // keep lane

              double delta_prev_x = prev_x - cos(prev_angle);
              pts_x.push_back(delta_prev_x);
              pts_x.push_back(car_x);

              double delta_prev_y = prev_y - sin(prev_angle);
              pts_y.push_back(delta_prev_y);
              pts_y.push_back(prev_y);

            } else {
              start[0] = end_path_s;
              prev_x = previous_path_x[prev_path_size-1];
              prev_y = previous_path_y[prev_path_size-1];

              double pos_x2 = previous_path_x[prev_path_size-2];
              double pos_y2 = previous_path_y[prev_path_size-2];
              prev_angle = atan2(prev_y-pos_y2,prev_x-pos_x2);
              d = end_path_d;

              pts_x.push_back(pos_x2);
              pts_x.push_back(prev_x);

              pts_y.push_back(pos_y2);
              pts_y.push_back(prev_y);              
            }
           
            start[1] = prev_speed;
            start[2] = prev_acc;           
            
            cout << "start vector: " << start[0] << ", " << start[1] << ", " << start[2] << endl;

            int additional_pts = N-prev_path_size;
            
            /*
            vector<double> end = get_end_vals(start, additional_pts, TIME_DELTA, TARGET_SPEED, MAX_ACCL, MAX_JERK);
            cout << "end vector: " << end[0] << ", " << end[1] << ", " << end[2] << endl;


            vector<double> end_xy = getXY(end[0], d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // compute direction at end point by using delta s
            vector<double> end_xy_delta = getXY(end[0]-.5, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double end_angle = atan2(end_xy[1]-end_xy_delta[1], end_xy[0]-end_xy_delta[0]);

            
            //cout << "before calling generate_poly_coefficients()" << endl;
            vector<double> startx(3), endx(3), starty(3), endy(3);
            startx[0] = prev_x;
            startx[1] = prev_speed * cos(prev_angle);
            startx[2] = 0.; // should be centripetal based on curvature
            endx[0] = end_xy[0];
            endx[1] = TARGET_SPEED * cos(end_angle);
            endx[2] = 0.;
            vector<double> x_poly_coeffs = generate_poly_coefficients(startx, endx, additional_pts*TIME_DELTA);

            starty[0] = prev_y;
            starty[1] = prev_speed * sin(prev_angle);
            starty[2] = 0.; // should be centripetal based on curvature
            endy[0] = end_xy[1];
            endy[1] = TARGET_SPEED * sin(end_angle);
            endy[2] = 0.;
            vector<double> y_poly_coeffs = generate_poly_coefficients(starty, endy, additional_pts*TIME_DELTA);

            //cout << "after calling generate_poly_coefficients()" << endl;
            vector<double> additional_x = generate_points_using_poly(x_poly_coeffs, additional_pts, TIME_DELTA);
            vector<double> additional_y = generate_points_using_poly(y_poly_coeffs, additional_pts, TIME_DELTA);
            
            for(int i=0; i<additional_x.size(); i++) {
              next_x_vals.push_back(additional_x[i]);
              next_y_vals.push_back(additional_y[i]);       
            }
            */
            
            //vector<double> traj2 = get_lane_trajectory(
            //  &prev_speed, &prev_acc,
            //  start, additional_pts, TIME_DELTA, TARGET_SPEED, MAX_ACCL, MAX_JERK);

            


            // We add some more points at 30 metres separation 
            // Logic is as instructed by this Udacity Path planning walkthrough video
            // https://www.youtube.com/watch?time_continue=4&v=7sI3VHFPP0w
            int lane = 1;
            for(int i=0; i<3; i++) {
                vector<double> tmp_xy = getXY(car_s + 30.*(i+1), 2.0 + 4.0 * lane, 
                                              map_waypoints_s, map_waypoints_x, map_waypoints_y);
                pts_x.push_back(tmp_xy[0]);
                pts_y.push_back(tmp_xy[1]);
            }


            // Now shift those coordinates to car- coordinates
            cout << "Conversion to car coordinates --> prev_angle: " << prev_angle << endl;
            double refx = prev_x;
            double refy = prev_y;
            for(int i=0; i< pts_x.size(); i++) {
              double tx = pts_x[i];
              double ty = pts_y[i];
              cout << "initial pts_x[i] = " << pts_x[i] << endl;
              cout << "pts_y[i] = " << pts_y[i] << endl;
              pts_x[i] = (tx - refx)*cos(prev_angle) + (ty - refy) * sin(prev_angle);
              pts_y[i] = (ty - refy)*cos(prev_angle) - (tx - refx) * sin(prev_angle);
              cout << "car pts_x[i] = " << pts_x[i] << endl;
              cout << "pts_y[i] = " << pts_y[i] << endl;
            }

            // Using spline to fit shifted x, y points
            tk::spline spl;            
            spl.set_points(pts_x, pts_y);

            // random ahead point
            double point_x_ahead = 30.;
            double point_y_ahead = spl(point_x_ahead);
            double dist_ahead = sqrt(point_x_ahead*point_x_ahead + point_y_ahead*point_y_ahead);
            // no. of steps it takes to get there at target speed

            get_speed_accleration(&prev_speed, &prev_acc, TIME_DELTA, TARGET_SPEED, MAX_ACCL, MAX_JERK);

            int steps = round (dist_ahead / (prev_speed * TIME_DELTA));

            vector<double> trajx;  
            vector<double> trajy;            
            for(int i=0; i<additional_pts; i++) {
              double tmpx = point_x_ahead*(i+1)/steps;
              double tmpy = spl(tmpx);

              // Shift them back to global coordinates              
              next_x_vals.push_back(refx + tmpx*cos(prev_angle) - tmpy*sin(prev_angle));
              next_y_vals.push_back(refy + tmpx*sin(prev_angle) + tmpy*cos(prev_angle));
            }

            /**
            vector<vector<double>> tmp_xy;         
            for(int i=0; i<additional_pts; i++){
              //cout << "traj2[" << i << "] = " << traj2[i] << endl;
              //vector<double> xy = getXY(traj2[i], d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              vector<double> xy(2);
              double spx = sx(traj2[i]);
              double spy = sy(traj2[i]);
              cout << "spline value for sx: " << traj2[i] << "-->" << spx << endl;
              xy[0] = spx;
              cout << "spline value for sy: " << traj2[i] << "-->" << spy << endl;
              xy[1] = spy;
              tmp_xy.push_back(xy);                  
            }

            
            // Clean xy's for normal jerk                  
            //clean_normal_jerk(&prev_speed, &prev_acc, prev_angle, 
            //                  tmp_xy, prev_x, prev_y, 
            //                  TIME_DELTA, TARGET_SPEED, MAX_ACCL, MAX_JERK);

            
            

            for(int i=0; i<tmp_xy.size(); i++) {

              next_x_vals.push_back(tmp_xy[i][0]);
              next_y_vals.push_back(tmp_xy[i][1]);       
            }
            **/
            
           
            

            path_length_sent = next_x_vals.size();

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
