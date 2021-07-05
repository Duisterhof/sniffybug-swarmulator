#ifndef WIND_SEEKER_H
#define WIND_SEEKER_H

#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "auxiliary.h"
#include "omniscient_observer.h"

class wind_seeker: public Controller
{
public:
	wind_seeker():Controller(){};
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);

	void generate_new_wp(const uint16_t ID);
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
	bool check_back_in_line(void);
	void update_animation_direction(const uint16_t ID);
	// float get_laser_min(const uint16_t ID);
	std::vector<float> policy_params = load_vector("conf/policies/gas_params.txt");
	random_generator rg;
	Point agent_pos, goal, random_point, other_agent_pos, start_wall_avoid, previous_position, swarm_cg;	// agent position point struct
	Line line_to_goal; // line to goal waypoint
	OmniscientObserver o; // used to get relative position of other agents

	std::vector<float> state;
	float iteration_start_time = 0.0; // counter to generate a new waypoint each time
	float time_started_wall_avoid = 0.0;

	float best_known_conc = 0.0;
	float init_dist_to_goal = 0.0;

	float omega = policy_params[0];
	float phi_p = policy_params[1];
	float phi_g = policy_params[2];
	float wp_travel = 3.0;
	float wp_travel_after_gas = 1.0;

	float omega_pre = policy_params[3];
	float rand_p_pre = policy_params[4];
	float update_time = policy_params[5]; // every x seconds a new waypoint is generated, if the goal isn't found before then
	float dist_reached_goal = policy_params[6];
	float laser_warning = policy_params[7];
	float swarm_warning = policy_params[8];
	float line_max_dist = policy_params[9]; // max x [m] from line until we move again to move back to it
	float k_swarm_laser_rep = policy_params[10]; // used for repulsion from lasers
	float k_swarm_avoidance = policy_params[11]; // used for repulsion between agents
	float swarm_laser_warning = policy_params[12];

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

	float desired_velocity = 0.5; // [m/s]
	std::vector<uint> closest_agents; // list of other agent IDs, first is closest other agent
	int status = 0; // status of avoidance, 0 = corridor following to wp, 1 = wall-following, 2 = repulsion (swarm)
	int previous_status = 0; // used to initialize when changing status
	float min_obs_avoid_thres = 2.0; // minimum distance to have moved when avoiding obstacle

	bool search_left = false; // searching direction when wall-following
	bool has_seen_gas{false};

	bool left_line_zone = false;
	
	int start_laser = 0; // laser at which we start 'looking', see paper
	int start_laser_corrected = 0; // to avoid oscillations
	int follow_laser = 0; // the actual laser direction in which we are moving
	int max_reached_laser = 0; // max reached laser it had to go to during safe wall-following
	float max_turns = 2; // max number of turns that can be made to avoid an object
};

#endif /*WIND_SEEKER_H*/
