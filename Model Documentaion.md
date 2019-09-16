
Goal

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

 

Implementation

The car starts from stand still from the middle lane on the highway and its position is updated every 0.02 seconds. 

Criteria followed in developing the path

The speed of the car is limited by the speed limit (50 mph). 

The car tries to maintain a certain distance between the itself and the car in front of it.

Maximum acceleration is limited to the 5 m/s^2.

Maximum jerk is limited to 10 m/s^3.

When the distance between the car and the car in front of it gets lower than the safe distance to maintain, the car tries to change lane and if the next lanes are occupied the car slows down.

 

Current issues

Was not able to change the start lane and start speed to achieve the set goals.