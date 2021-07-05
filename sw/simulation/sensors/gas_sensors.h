#ifndef GAS_SENSORS_H
#define GAS_SENSORS_H

#include <vector>
#include "auxiliary.h"
#include "main.h"

class laser_ray
{
	public:
		float heading; //heading in ENU frame
		std::vector<std::vector<float>> walls; //all walls that the laser ray intersects with
		std::vector<Point> intersection_points; //intersection points with walls
		std::vector<float> distances;//distances to walls of intersection
		std::vector<std::vector<float>> intersecting_walls; //the walls it's intersecting with
		std::vector<float> intersect_wall; //the wall that it's intersecting with
		float range = 0.0 ; //final outcome, the measured range
		bool wall_following = false;
		// wall following params
		float heading_kp = 4.0;
		float heading_kd = 0.0;
		float heading_ki = 0.0;

		float heading_error = 0.0;
		float old_heading_error = 0.0;
		float heading_error_d = 0.0;
		float heading_error_i = 0.0;
		
		float heading_accumulator = 0.0; // what is finally added to heading
		float desired_laser_distance = 2.5; // desired minimal laser distance when following a wall
		float critical_laser_distance = 0.5; // when this point is reached we should be really really careful
		float engage_laser_distance = 2.7; // the end of the wall following zone, get more than this clearance to get out
		float old_accumulator = 0.0; // old accumulator used to limit the change in accumulation
};


laser_ray get_laser_reads(laser_ray ray, const uint16_t ID);
void load_all_lasers(const uint16_t ID);

inline std::vector<float> laser_headings = {0,M_PI_2,M_PI,3*M_PI_2} ;	// headings in body frame of all lasers

#endif
