#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <limits>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const double lane_width = 4.0;		// lane width
const double safe_dist = 20.0;		// distance to maintain from other car in the front
const double speed_limit = 49.5;	// speed limit	

int main() 
{
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

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) 
	{
	  std::istringstream iss(line);
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

	// Start on lane 1 
	int start_lane = 1;

	// Reference velocity (mph)
	double start_vel = 0.0;

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
		        &map_waypoints_dx,&map_waypoints_dy, & start_vel, & start_lane]
				(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
				 uWS::OpCode opCode)
	{
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2') 
		{
			auto s = hasData(data);
			
			if (s != "") 
			{
				auto j = json::parse(s);
				
				string event = j[0].get<string>();
				
				if (event == "telemetry") 
				{
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

					// Sensor Fusion Data, a list of all other cars on the same side 
					//   of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					/**
					 * TODO: define a path made up of (x,y) points that the car will visit
					 *   sequentially every .02 seconds
					 */

					int prev_path_size = previous_path_x.size();

					if (prev_path_size > 0)
						car_s = end_path_s;

					bool closer_than_safe_dist = false;
					bool is_left_lane_free = true;
					bool is_right_lane_free = true;

					for (size_t i = 0; i < sensor_fusion.size(); ++i) 
					{
						SensorData sensorData(sensor_fusion[i]);
						if (is_in_lane(sensorData.d, start_lane, lane_width))
						{
							sensorData.s += (double)prev_path_size * 0.02 * sensorData.speed;

							if ((sensorData.s > car_s) && ((sensorData.s - car_s) < safe_dist))
								closer_than_safe_dist = true;
						}
					}

					if (closer_than_safe_dist)
					{
						for (size_t i = 0; i < sensor_fusion.size(); ++i) 
						{
							SensorData sensorData(sensor_fusion[i]);
							if (is_in_lane(sensorData.d, start_lane - 1, lane_width))
							{
								sensorData.s += (double)prev_path_size * 0.02 * sensorData.speed;
								bool too_close_to_change = (sensorData.s > car_s - safe_dist / 2) && (sensorData.s < car_s + safe_dist / 2);
								if (too_close_to_change)
									is_left_lane_free = false;
							}
							else if (is_in_lane(sensorData.d, start_lane + 1, lane_width))
							{
								sensorData.s += (double)prev_path_size * 0.02 * sensorData.speed;
								bool too_close_to_change = (sensorData.s > car_s - safe_dist / 2) && (sensorData.s < car_s + safe_dist / 2);
								if (too_close_to_change)
									is_right_lane_free = false;
							}
						}
					}

					if (closer_than_safe_dist && is_left_lane_free && start_lane > 0)
						start_lane -= 1;
					else if (closer_than_safe_dist && is_right_lane_free && start_lane < 2)
						start_lane += 1;

					if (closer_than_safe_dist)
						start_vel -= 0.224;
					else if (start_vel < speed_limit)
						start_vel += 0.224;

					// List of waypoints. 
					vector<double> pts_x;
					vector<double> pts_y;

					// Reference values 
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);

					// Starting from no reference
					if (prev_path_size < 2) {
						double prev_car_x = car_x - cos(car_yaw);
						double prev_car_y = car_y - sin(car_yaw);

						pts_x.push_back(prev_car_x); pts_x.push_back(car_x);
						pts_y.push_back(prev_car_y); pts_y.push_back(car_y);
					}
					// starting from previous reference
					else {
						ref_x = previous_path_x[prev_path_size - 1];
						ref_y = previous_path_y[prev_path_size - 1];

						double ref_x_prev = previous_path_x[prev_path_size - 2];
						double ref_y_prev = previous_path_y[prev_path_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						pts_x.push_back(ref_x_prev); pts_x.push_back(ref_x);
						pts_y.push_back(ref_y_prev); pts_y.push_back(ref_y);
					}

					// Transforming Frenet coordinates to cartesian
					vector<double> next_wp0 = getXY(car_s + 30, (lane_width * start_lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp1 = getXY(car_s + 60, (lane_width * start_lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					vector<double> next_wp2 = getXY(car_s + 90, (lane_width * start_lane + lane_width / 2), map_waypoints_s, map_waypoints_x, map_waypoints_y);

					{
						pts_x.push_back(next_wp0[0]);
						pts_x.push_back(next_wp1[0]);
						pts_x.push_back(next_wp2[0]);
						pts_y.push_back(next_wp0[1]);
						pts_y.push_back(next_wp1[1]);
						pts_y.push_back(next_wp2[1]);
					}

					// Changing the reference coordinates
					for (size_t i = 0; i < pts_x.size(); ++i) 
					{
						double car_ref_x = pts_x[i] - ref_x;
						double car_ref_y = pts_y[i] - ref_y;
						pts_x[i] = car_ref_x * cos(0 - ref_yaw) - car_ref_y * sin(0 - ref_yaw);
						pts_y[i] = car_ref_x * sin(0 - ref_yaw) + car_ref_y * cos(0 - ref_yaw);
					}

					// Create a spline
					tk::spline spline;
					spline.set_points(pts_x, pts_y);

					// Starting from previous points
					for (size_t i = 0; i < previous_path_x.size(); ++i) {
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					double dest_x = 30.0;
					double dest_y = spline(dest_y);
					double dist = sqrt(dest_x * dest_x + dest_y * dest_y);

					double summing_x = 0.0;

					for (size_t i = 1; i <= 50 - previous_path_x.size(); ++i) {

						double N = dist / (0.02 * start_vel / 2.24);
						double x_point = summing_x + dest_x / N;
						double y_point = spline(x_point);

						summing_x = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						// Rotate back into previous coordinate system
						x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
						y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
					}

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}  // end "telemetry" if
			} 
			else 
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}  // end websocket if
	}); // end h.onMessage
	
	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) 
	{
		std::cout << "Connected!!!" << std::endl;
	});
	
	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                      char *message, size_t length) 
	{
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) 
	{
		std::cout << "Listening to port " << port << std::endl;
	} 
	else 
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}