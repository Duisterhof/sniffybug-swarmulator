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
	void follow_line(const uint16_t ID, float* v_x, float* v_y);
	void update_follow_laser(void);
	void update_direction(const uint16_t ID);
	void update_status(const uint16_t ID);
	float get_agent_dist(const uint16_t ID1, const uint16_t ID2);
	void follow_wall(const uint16_t ID, float* v_x, float* v_y);
	void repulse_swarm(const uint16_t ID, float* v_x, float* v_y);
	void wall_follow_init(const uint16_t ID);
	void update_start_laser(void);
	void reset_wall_follower(void);
	bool free_to_goal(const uint16_t ID);
	void check_passed_goal(const uint16_t ID);
	bool avoided_obstacle(const uint16_t ID);
	void update_swarm_cg(const uint16_t ID);
	// float get_laser_min(const uint16_t ID);
	std::vector<float> policy_params = load_vector("conf/policies/gas_params.txt");
	random_generator rg;
	Point agent_pos, goal, random_point, other_agent_pos, start_wall_avoid, previous_position, swarm_cg;	// agent position point struct
	Line line_to_goal; // line to goal waypoint
	OmniscientObserver o; // used to get relative position of other agents

	std::vector<float> state;
	std::vector<laser_ray> laser_rays; // contains all laser ray objects
	float laser_headings[4] = {0,M_PI_2,M_PI,3*M_PI_2};	// headings in body frame of all lasers

	float iteration_start_time = 0.0; // counter to generate a new waypoint each time

	float dist_reached_goal = 1.0; // distance threshold for classifying as finding the goal
	float time_started_wall_avoid = 0.0;
	
	// learned params
	// float rand_p = 0.0;
	// float omega = 0.2;
	// float phi_p = 0.3;
	// float phi_g = 2.0;
	//float update_time = 40.0;
	float time_to_follow = 100.0;
	float best_known_conc = 0.0;

	float rand_p = policy_params[0];
	float omega = policy_params[1];
	float phi_p = policy_params[2];
	float phi_g = policy_params[3];

	float omega_pre = policy_params[4];
	float swarm_cg_pre = policy_params[5];
	float rand_p_pre = policy_params[6];
	float update_time_after = policy_params[7]; // every x seconds a new waypoint is generated, if the goal isn't found before then
	float update_time_pre = policy_params[8];
	float update_time = update_time_pre;
	// float time_to_follow = policy_params[5];
	float old_vx = 0.0;
	float old_vy = 0.0;
	float close_to_source_thres = 1.5;
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

	float laser_warning = 1.5; // x [m] before a laser range value is seen as dangerous --> avoid stuff
	float swarm_warning = 0.8; // x [m] before a distance to another agent is classified as dangerous
	std::vector<uint> closest_agents; // list of other agent IDs, first is closest other agent
	int status = 0; // status of avoidance, 0 = corridor following to wp, 1 = wall-following, 2 = repulsion (swarm)
	int previous_status = 0; // used to initialize when changing status
	float min_obs_avoid_thres = 2.0; // minimum distance to have moved when avoiding obstacle

	bool search_left = false; // searching direction when wall-following
	int start_laser = 0; // laser at which we start 'looking', see paper
	int start_laser_corrected = 0; // to avoid oscillations
	int follow_laser = 0; // the actual laser direction in which we are moving
	int max_reached_laser = 0; // max reached laser it had to go to during safe wall-following
	
	float max_turns = 2; // max number of turns that can be made to avoid an object
	float k_swarm_laser_rep = 5.0; // used for repulsion from lasers
	float k_swarm_avoidance = 15.0; // used for repulsion between agents

};

#endif /*BUG_REPULSION_H*/
