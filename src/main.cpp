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
#include <stdlib.h>

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
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

  int lane=1;

  //reference velocity in mph
  double ref_vel = 0.0;

  //maximum safe velocity
  double max_vel = 49.3; //mph

  //minimum safe velocity
  double min_vel = 35;//mph

  //constant/default cost of changing lane
  double cost_lane_change = 0.1;

  //safe distance for lane change
  double dist_lane_change = 30;

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


  h.onMessage([&lane,&ref_vel,&max_vel, &min_vel,&cost_lane_change,&dist_lane_change,
    &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,
    &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
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

            int prev_size = previous_path_x.size();

            if(prev_size>0){
              car_s = end_path_s;
            }

            bool too_close = false;

            // possible future state are kl (keep lane), lcl (change lane left), lcr (change lane right)
            vector<string> possible_states = {"kl", "lcl", "lcr"};
            string state = "kl";
            vector<double> cost = {1000,1000,1000};

            //find ref_v to use
            for(int i=0; i< sensor_fusion.size(); i++){
              // car is in my lane
              float d = sensor_fusion[i][6];
              if(d<(2+4*lane+2) && d>(2+4*lane-2)){
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];

                //project out the detected car s since currenlty we are at previous steps
                check_car_s += ((double)prev_size*.02*check_speed);

                //check if s value of the detected car is greater than the ego car and calculate the gap
                if((check_car_s > car_s) && ((check_car_s - car_s) < dist_lane_change)){
                  // flag to start assesing lane change
                  too_close = true;
                }
              }
            }

            //reducing speed to keep the safe distance from the car in front
            if(too_close){
                ref_vel -= .50;

                //Brake Module
                // if checked_car is closer than 10 meter, the ego car decelerate at a much higher value to
                //avoid collision (braking)
                for(int i=0; i< sensor_fusion.size(); i++){
                  // car is in my lane
                  float d = sensor_fusion[i][6];
                  double x = sensor_fusion[i][1];
                  double y = sensor_fusion[i][2];
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  //project d of the detected car to predict not only cars in the same lane but cars that
                  // may want to change their lane and move to the ego car lane
                  x += ((double)prev_size*.2*vx);
                  y += ((double)prev_size*.2*vy);
                  double check_car_theta = atan2(vy,vx);
                  vector<double> check_car_future_sd = getFrenet(x, y, check_car_theta, map_waypoints_x, map_waypoints_y);
                  d = check_car_future_sd[1];

                  if(d<(2+4*lane+2) && d>(2+4*lane-2)){

                    //project out the detected car s since currenlty we are at previous steps
                    check_car_s += ((double)prev_size*.02*check_speed);

                    //check if the detected car is closer than 10m and decelerate (braking)
                    if((check_car_s > car_s) && ((check_car_s - car_s) < 10)){
                      ref_vel -= 2.0;
                    }
                  }
                }
            }else if(ref_vel < max_vel){
              ref_vel += .50;
            }

            //cost weights
            double c_velocity_dif = 0.01;
            double c_lane_change_cars_dist = 0.01;

            if(too_close){
              //adding minimum lane change cost to cost of lcl and lcr
              cost[1] = cost_lane_change;
              cost[2] = cost_lane_change;
              if(lane==0){
                cost[1]=1001;
              }
              if(lane==2){
                cost[2]=1001;
              }


              for(int i=0; i< sensor_fusion.size(); i++){
                float d = sensor_fusion[i][6];
                if(d<(2+4*lane+2) && d>(2+4*lane-2)){
                  if(ref_vel<min_vel){
                    cost[0] = 0;
                  }
                  else{
                  //calculate cost of kl (keep_lane)
                  cost[0] = c_velocity_dif*max_vel/(max_vel - ref_vel);
                  }
                }

                if(d<(2+4*(lane-1)+2) && d>(2+4*(lane-1)-2)){
                  //calculate cost of lcl (change lane left)
                  //C|distance from car ahead of the ego vehicle in the target lane + C|distance of car behind the ego vehicle in the target lane + C|Lane change;
                  //car in the left lane
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  //project out the detected car s since currenlty we are at previous steps
                  check_car_s += ((double)prev_size*.02*check_speed);

                  if(abs(check_car_s - car_s) > dist_lane_change){
                    cost[1] += c_lane_change_cars_dist/exp(abs(check_car_s-car_s)-dist_lane_change);
                  }
                  else{
                    cost[1] = 999;
                  }
                }

                if(d<(2+4*(lane+1)+2) && d>(2+4*(lane+1)-2)){
                  //calculate cost of lcl (change lane left)
                  //C|distance from car ahead of the ego vehicle in the target lane + C|distance of car behind the ego vehicle in the target lane + C|Lane change;
                  //car in the left lane
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  //project out the detected car s since currenlty we are at previous steps
                  check_car_s += ((double)prev_size*.02*check_speed);

                  if(abs(check_car_s - car_s) > dist_lane_change){
                    cost[2] += c_lane_change_cars_dist/exp(abs(check_car_s-car_s)-dist_lane_change);
                  }
                  else{
                    cost[2] = 999;
                  }
                }
              }

                //finding which state has the lowest cost and choose that state
                string best_next_state;
                double min_cost = 999999;
                for(int j=0; j<cost.size(); j++){
                  if(cost[j]<min_cost){
                    min_cost = cost[j];
                    state = possible_states[j];
                  }
                }
              }

              //perform action for the selected state
              if(state=="lcl"){
                if(lane==1 ){
                  lane = 0;
                }
                if(lane==2){
                  lane=1;
                }
              }
              if(state=="lcr"){
                if(lane==1){
                  lane=2;
                }
                if(lane==0){
                  lane = 1;
                }
              }

            // create a list of widely (x,y) points that are evenly spaced at 30 m
            // later we will interapolate waypoints with a spline and fill it in with more waypoints
            vector<double> ptsx;
            vector<double> ptsy;

            // referenced x, y, and yaw states
            // we either use the car starting point or end points from the previous path
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //if the previous path size is almost zero then we use the car starting point as ref states
            if(prev_size < 2){
              // use two points to make the path tangent to the car direction
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }

            // use previous end points as reference waypoints
            else
            {
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            // In Frenet coordinate add evenly 30m spaced points ahead of the starting waypoint
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for(int i=0; i<ptsx.size(); i++)
            {
              //shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i]=(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i]=(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            //create a spline
            tk::spline s;

            //set (x,y) points we will use for Planner
            s.set_points(ptsx,ptsy);

            //Define actual waypoints (x,y) we will use for the planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //start with all of the previous path points from last time
            for(int i=0; i<previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            //calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            //fill up the rest of our path planner after filling it with previous points; here we always output 50 points
            for(int i=0; i<50-previous_path_x.size(); i++){

              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on+(target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to normal after rotating it earlier
              x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

          	json msgJson;

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
