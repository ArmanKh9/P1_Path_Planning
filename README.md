# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

# Write Up

In this project a vehicle has to safely drive in a highway by using path planning strategies. The project includes a simulator provided by Udacity in order to run the path planning algorithm. The path planner algorithm is written in src/main.cpp.
The path planning part of the project can be broken into three parts:

    Prediction
    Planning
    Trajectory Generation

##Prediction
The prediction part of the algorithm looks into future states of other vehicles on the road by pulling in data from sensor fusion. The ego vehicle tends to maintain its driving lane until it gets closer than 30 meter to a vehicle on the same lane. Being closer than 30 meter triggers some computation that will be performed in the planning section. Here is part of the code that predicts future location of the vehicles that are driving on the same lane as the ego vehicle:

```cpp
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
```
If a vehicle on the same lane is closer than 30 meter, a boolean value is flagged.

##Planning
In the planning part, the ego vehicle determines which action it needs to take to not only to drive safely but to maintain highest possible speed in the highway which is 50 mph in this project. The action that the ego vehicle can take are as follows:

    keep lane marked as "kl"
    lane change left marked as "lcl"
    lane change right maked as "lcr"

A cost value is defined for each of these actions. The ego vehicle will take the action with the smallest cost. Initially cost of all actions are set to 1000 and the vehicle always keeps its current lane unless it gets closer than 30 meter to a vehicle in front of it. Once that happens, the ego vehicle starts to decelerate at a low rate and then start computing cost of each action. Below is a piece of code that computes cost associated with keeping lane or changing lane:

```cpp
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
      }
```

```cpp
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
```
A safe lane change is dependent on safe distance in s direction between the ego vehicle and other vehicles in the target lane. Here dist_lane_change is defining that distance which is set to 30 meters. So a minimum of 60 meter gap is needed for a safe lane change. It must be noted that this distance is safe for 50 mph speed and needs to be adjusted for lower speeds.

When the ego vehicle is on the most left or right lane, lcl and lcr associated cost values set to 1001 to put that action out of options.

Finally, the minimum cost value is extracted and its associated action is performed through a decision tree.

```cpp
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
```
This decision tree can be updated to be independent of lane number.

If left and right lanes are not clear for a lane change, the vehicle maintain its lane and adjust its speed to stay more than 30 meters away from the front vehicle. The vehicle keeps this state until other lanes become available for a lane change.

###brake module
Sometimes other vehicles make erratic moves such as sudden lane change or hard brakes. In these cases, collision is likely to happen. Brake module was defined in order for the ego vehicle to avoid collisions by decelerating at a higher rate than usual slow down only if the distance to the front vehicle becomes less than 10 meters. Also, the module predicts future d value of cars in other lanes to check if they are making an unsafe lane change and act accordingly.

```cpp
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
```

##Trajectory Generation
The trajectory generation module takes the lane information from the planning module and creates a set of waypoints for the vehicle to move too. To smooth the trajectory, the last two values from the previous path and three new waypoints set at 30m, 60m, and 90m are placed into vectors. Those waypoints are then fed into a spline generation tool.



#Results
The ego vehicle was able to drive the entire loop successfully at multiple runs. Only one incident happened with the current code and that was due to multiple collisions between other vehicles and the ego vehicle failed to detect those as unsafe condition.

![One Round](https://github.com/ArmanKh9/P1_Path_Plan/images/4.57.png)

The code was ran for mor than 22 mins and the vehicle seemed to be able to continuously drive without any incident.

![Three Rounds](https://github.com/ArmanKh9/P1_Path_Plan/images/15.62.png)


#Addition Wrok
The code does too many iteration and data scanning. Efficiency can be improved by combining several sensor fusion data pulling loops. Also, the safe distance for lane change can be defined in a more dynamic fashion to enable the vehicle to perform a lane change with smaller target lane gaps at lower speeds. For example, if speed is 25 mph, the required gap should be 15 meters instead of 30 meters which is required at 50 mph.
Moreover, collision between other cars can trigger the braking module.
