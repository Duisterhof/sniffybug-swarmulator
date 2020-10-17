#ifndef BUG_REPULSION_H
#define BUG_REPULSION_H


#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "auxiliary.h"
#include "omniscient_observer.h"

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


class bug_repulsion: public Controller
{
public:
	bug_repulsion():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
	
	void generate_new_wp(const uint16_t ID);
	laser_ray get_laser_reads(laser_ray ray, const uint16_t ID);
	void load_all_lasers(const uint16_t ID);
	void update_best_wps(const uint16_t ID);
	void get_new_line(void);
	void follow_line(float* v_x, float* v_y);
	void update_follow_laser(void);
	void update_direction(const uint16_t ID);

	random_generator rg;
	Point agent_pos, goal, random_point;	// agent position point struct
	Line line_to_goal; // line to goal waypoint

	std::vector<float> state;
	std::vector<laser_ray> laser_rays; // contains all laser ray objects
	float laser_headings[4] = {0,M_PI_2,M_PI,3*M_PI_2};	// headings in body frame of all lasers

	float iteration_start_time = 0.0; // counter to generate a new waypoint each time
	float update_time = 10.0; // every x seconds a new waypoint is generated, if the goal isn't found before then
	float dist_reached_goal = 0.5; // distance threshold for classifying as finding the goal

	float rand_p = 0.0;
	float omega = 0.2;
	float phi_p = 0.3;
	float phi_g = 2.0;

	// line following
	float line_heading; // heading from agent_pos to goal
	int lower_idx; // from a clockwise-postive, the lower idx of the laser in the heading zone
	int upper_idx; // same
	float corrected_heading; // line_heading - agent_heading. 
	float quad_heading;// heading between lower-idx and line_heading
	
	int following_laser; // laser that we're following in body frame heading defined in 'laser_headings'
	float following_heading; // corresponding heading, in body frame

	float line_max_dist = 0.5; // max x [m] from line until we move again to move back to it
	float desired_velocity = 0.5; // [m/s]
};

#endif /*BUG_REPULSION_H*/
