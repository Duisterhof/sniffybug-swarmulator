#include "wind_seeker.h"
#include "draw.h"
#include "randomgenerator.h"
#include <tuple>
#include "main.h"
#include <math.h>
#include "auxiliary.h"
#include "omniscient_observer.h"
#include "gas_sensors.h"

void wind_seeker::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  state = s.at(ID)->state; // load agent state
  load_all_lasers(ID);
  agent_pos.x = state[1]; // loading agent pos struct Point
  agent_pos.y = state[0];
  // load_all_lasers(ID); // modelling multirangers
  update_best_wps(ID); // use gas sensing to update the best seen points 
  closest_agents = o.request_closest(ID);

  s.at(ID)->line = line_to_goal;

  if ( simtime_seconds-iteration_start_time >= update_time || getDistance(goal,agent_pos) < dist_reached_goal )
  {
    generate_new_wp(ID);
    status = 0;
    previous_status = 0;
  }
  else if (simtime_seconds == 0.0)
  {
    generate_new_wp(ID);
    status = 0;
    previous_status = 0;
  }

  update_animation_direction(ID);

  update_status(ID); // get new status
  // status = 2;
  switch(status)
  {
    case 0:
      follow_line(ID,&v_x, &v_y);
      break;
    case 1:
      follow_wall(ID, &v_x,&v_y);
      break;
    case 2:
      repulse_swarm(ID, &v_x, &v_y);
      break;
  }

  float distance_to_source = sqrtf(powf((s.at(ID)->state[1]-environment.gas_obj.source_location[0]-environment.x_min),2)+powf((s.at(ID)->state[0]-environment.gas_obj.source_location[1]-environment.y_min),2));
  s.at(ID)->distance_accumulator = s.at(ID)->distance_accumulator + distance_to_source ;
  s.at(ID)->num_steps = s.at(ID)->num_steps + 1;

  if (distance_to_source < close_to_source_thres)
  {
    s.at(ID)->num_close_to_source += 1;
    if(s.at(ID)->first_in_source == 100)
    {
      s.at(ID)->first_in_source = simtime_seconds;
    }
  }
}
bool wind_seeker::check_back_in_line(void)
{
  float dist = get_distance_to_line(line_to_goal,agent_pos);
  float dist_goal = getDistance(agent_pos,goal);
  if (dist > line_max_dist && left_line_zone == false)
  {
    left_line_zone = true;
  }

  if (left_line_zone == true && dist < line_max_dist && (init_dist_to_goal-dist_goal) > 0)
  {
    left_line_zone = false;
    return true;
  }
  else 
  {
    return false;
  }
}


void wind_seeker::wall_follow_init(const uint16_t ID)
{
  init_dist_to_goal = getDistance(agent_pos,goal);
  left_line_zone = false;
  float temp_line_heading = get_heading_to_point(agent_pos,goal); // used to follow the line
  float temp_corrected_heading = temp_line_heading - s.at(ID)->get_orientation();
  positive_angle(&temp_corrected_heading);

  int lower_idx_temp = (int)(temp_corrected_heading /M_PI_2);
  int upper_idx_temp = lower_idx_temp+1;
  cap_laser(&lower_idx_temp);
  cap_laser(&upper_idx_temp);

  float lower_heading = lower_idx_temp*M_PI_2 + s.at(ID)->get_orientation();
  float upper_heading = upper_idx_temp*M_PI_2 + s.at(ID)->get_orientation();
  positive_angle(&lower_heading);
  positive_angle(&upper_heading);

  if ( abs(temp_corrected_heading-lower_heading) > abs(temp_corrected_heading-upper_heading))
  {
    start_laser = upper_idx_temp;
    search_left = true;
  }
  else
  {
    start_laser = lower_idx_temp;
    search_left = false;
  }
  
}

bool wind_seeker::free_to_goal(const uint16_t ID)
{
  if (s.at(ID)->laser_ranges[lower_idx] > laser_warning && s.at(ID)->laser_ranges[upper_idx] > laser_warning)
  {
    return true;
  }
  else
  {
    return false;
  }
  
  
}

void wind_seeker::check_passed_goal(const uint16_t ID)
{
  float current_heading_to_goal = get_heading_to_point(agent_pos,goal);
  float following_heading = following_laser*M_PI_2 + s.at(ID)->get_orientation();
  if (abs(current_heading_to_goal-following_heading) > M_PI_2)
  {
    update_direction(ID);
  }
}


void wind_seeker::update_start_laser(void)
{
  if (search_left)
  {
    if (max_reached_laser < start_laser)
    {
      start_laser_corrected = max_reached_laser + 1;
    }
    else
    {
      start_laser_corrected = start_laser;
    }
  }
  else
  {
    if (max_reached_laser > start_laser)
    {
      start_laser_corrected = max_reached_laser - 1;
    }
    else
    {
      start_laser_corrected = start_laser;
    }
  }
}

bool wind_seeker::avoided_obstacle(const uint16_t ID)
{
  // if (get_distance_to_line(line_to_goal,agent_pos) > line_max_dist)
  // {
  //   return false;
  // }
  // else if (getDistance(start_wall_avoid,agent_pos) > min_obs_avoid_thres)  
  // {
  //   return true;
  // }
  // else
  // {
  //   return false;
  // }
  // bool avoiding = false;
  // std::tuple<bool,Point> intersect_return_obs;

  // Point laser_temp, goal_extended;
  // goal_extended = {.x= line_to_goal.p0.x + 100*(line_to_goal.p1.x-line_to_goal.p0.x), .y= line_to_goal.p0.y + 100*(line_to_goal.p1.y-line_to_goal.p0.y)};
  // // goal_extended = goal;
  // float start_goal_dist = getDistance(start_wall_avoid,goal_extended);

  // for (int i = 0; i < 4; i++)
  // {
  //   if (s.at(ID)->laser_ranges[i] > 2.5)
  //   {
  //     float laser_glob_heading = i*M_PI_2 + s.at(ID)->get_orientation();
  //     positive_angle(&laser_glob_heading);
  //     laser_temp.x = agent_pos.x + sinf(laser_glob_heading)*100;
  //     laser_temp.y = agent_pos.y + cosf(laser_glob_heading)*100;
  //     //get intersection point and a bool if it's on the wall or not
  //     intersect_return_obs = getIntersect(agent_pos,laser_temp,line_to_goal.p0,goal_extended);
  //     bool on_line = std::get<0>(intersect_return_obs);
  //     Point intersect = std::get<1>(intersect_return_obs);

  //     if (on_line)
  //     {
  //       float dist = getDistance(intersect,goal_extended);
  //       if ((start_goal_dist-dist)> min_obs_avoid_thres)
  //       {
  //         avoiding = true;
  //       }
  //     }
  //   }
  // }

  bool avoiding = false;
  float goal_heading = get_heading_to_point(agent_pos,goal);
  float body_angle = s.at(ID)->get_orientation();
  positive_angle(&body_angle);
  positive_angle(&goal_heading);

  float goal_body_heading = goal_heading - body_angle;
  positive_angle(&goal_body_heading);

  int closest_laser = (int)(goal_body_heading/M_PI_2);
  if ((goal_body_heading - closest_laser*M_PI_2)>M_PI_4)
  {
    closest_laser = closest_laser + 1;
  }

  if (s.at(ID)->laser_ranges[closest_laser] > 2.5)
  {
    avoiding= true;
  }


  return avoiding;
}

void wind_seeker::follow_wall(const uint16_t ID, float* v_x, float* v_y)
{
  // terminalinfo::debug_msg(std::to_string(search_left));
  update_start_laser(); // avoid osscialations
  int local_laser_idx = start_laser_corrected;
  int forbidden_direction = following_laser - 2;
  cap_laser(&forbidden_direction);
  if (search_left == false)
  {
    for (int i = start_laser_corrected; i < (start_laser+4); i++)
    {

      local_laser_idx = i;
      cap_laser(&local_laser_idx);
        // if (abs(start_laser-i) > max_turns)
        // {
        //   reset_wall_follower();
        // }

        if (s.at(ID)->laser_ranges[local_laser_idx] > laser_warning)
        {
          if (i > max_reached_laser)
          {
            max_reached_laser = i;
          }
          break;
        }

    }
  }
  else
  {
    for (int i = start_laser_corrected; i > (start_laser-4); i--)
    {
      local_laser_idx = i;
      cap_laser(&local_laser_idx);
      
        // if (abs(start_laser-i) > max_turns)
        // {
        //   reset_wall_follower();
        // }

        if (s.at(ID)->laser_ranges[local_laser_idx] > laser_warning)
        {
          if (i < max_reached_laser)
          {
            max_reached_laser = i;
          }
          break;
        }
      
    }
  }
  if(s.at(ID)->laser_ranges[local_laser_idx] < laser_warning)
  {
    reset_wall_follower();
  }

  following_laser = local_laser_idx;
  *(v_x) = cosf(following_laser*M_PI_2)*desired_velocity;
  *(v_y) = sinf(following_laser*M_PI_2)*desired_velocity;
}

void wind_seeker::reset_wall_follower(void)
{
  if (search_left)
  {
    search_left = false;
  }
  else
  {
    search_left = true;
  }
  max_reached_laser = start_laser;
  
}

void wind_seeker::update_swarm_cg(const uint16_t ID)
{
  if (nagents > 1)
  {
    swarm_cg = {.x=0,.y=0};
    for (uint i = 0 ; i < nagents; i++)
    {
      if (i != ID)
      {
        swarm_cg.x += (s.at(ID)->state[1]);
        swarm_cg.y += (s.at(ID)->state[0]);
      }
    
    }
    swarm_cg = {.x=(swarm_cg.x/(nagents-1)), .y = (swarm_cg.y/(nagents-1))};
  }
  else
  {
    swarm_cg = agent_pos;
  }
  
}

void wind_seeker::repulse_swarm(const uint16_t ID, float* v_x, float* v_y)
{
  *(v_x) = 0;
  *(v_y) = 0;

  // attraction to waypoint
  float local_psi = get_heading_to_point(agent_pos,goal) - s.at(ID)->get_orientation() ;
  positive_angle(&local_psi);
  *(v_x) = cosf(local_psi)*desired_velocity;
  *(v_y) = sinf(local_psi)*desired_velocity;


  // add repulsion from lasers      
  for ( int i = 0; i<4; i++)
  {
    if ( s.at(ID)->laser_ranges[i] < swarm_laser_warning)
    {
      float laser_heading = laser_headings[i];
      float heading_away_from_laser = laser_heading - M_PI;
      *(v_x) += cosf(heading_away_from_laser)*k_swarm_laser_rep*(swarm_laser_warning-s.at(ID)->laser_ranges[i]);
      *(v_y) += sinf(heading_away_from_laser)*k_swarm_laser_rep*(swarm_laser_warning-s.at(ID)->laser_ranges[i]);
    }
  }

  // repulsion from close agents
  for (uint i =0; i<closest_agents.size(); i++)
  {
    if (get_agent_dist(ID,closest_agents[i]) < swarm_warning)
    {
      other_agent_pos.x = s.at(closest_agents[i])->state[1];
      other_agent_pos.y = s.at(closest_agents[i])->state[0];
      float heading_to_other_agent = get_heading_to_point(agent_pos,other_agent_pos) - s.at(ID)->get_orientation();
      float heading_away_from_agent = heading_to_other_agent - M_PI;
      *(v_x) += cosf(heading_away_from_agent)*k_swarm_avoidance*(swarm_warning-get_agent_dist(ID,closest_agents[i]));
      *(v_y) += sinf(heading_away_from_agent)*k_swarm_avoidance*(swarm_warning-get_agent_dist(ID,closest_agents[i]));
    }
  }
  
  float vector_size = sqrtf(powf(*(v_x),2)+powf(*(v_y),2));

  *(v_x) = *(v_x)/vector_size*desired_velocity;
  *(v_y) = *(v_y)/vector_size*desired_velocity;
  
}

void wind_seeker::update_status(const uint16_t ID)
{
  closest_agents = o.request_closest(ID);
  // terminalinfo::debug_msg(std::to_string(free_to_goal(ID)));

  if (closest_agents.size() > 0 && get_agent_dist(closest_agents[0],ID) < swarm_warning)
  {
    status = 2;
  }
  else if(previous_status == 2)
  {
    status = 0;
    get_new_line();
    update_direction(ID);
  }
  else if (check_back_in_line() && previous_status == 1)
  {
    status = 0;
    get_new_line();
    update_direction(ID);
  }
  else if (!free_to_goal(ID) && previous_status == 0)
  {
    status = 1;
    wall_follow_init(ID);
    start_wall_avoid = agent_pos;
    point_on_line(&start_wall_avoid,line_to_goal);
    time_started_wall_avoid = simtime_seconds;
  }


  previous_status = status;

}

float wind_seeker::get_agent_dist(const uint16_t ID1, const uint16_t ID2)
{
  return(sqrtf(powf(s.at(ID1)->state[0]-s.at(ID2)->state[0],2)+powf(s.at(ID1)->state[1]-s.at(ID2)->state[1],2)));
}

// updates individual and swarm best wps
void wind_seeker::get_new_line(void)
{
  line_to_goal.p0 = agent_pos;
  line_to_goal.p1 = goal;
  update_line(&line_to_goal);

}

void wind_seeker::update_follow_laser(void)
{
  if (get_heading_to_point(agent_pos,goal) > line_heading)
  {
    following_laser = upper_idx;
  }
  else
  {
    following_laser = lower_idx;
  }
  
}

// called when following the line within a corridor
void wind_seeker::follow_line(const uint16_t ID, float* v_x, float* v_y)
{
  // check_passed_goal(ID);
  if (get_distance_to_line(line_to_goal,agent_pos) > line_max_dist)
  {
    update_follow_laser();
  }
  following_heading = following_laser*M_PI_2;
  *(v_x) = cosf(following_heading)*desired_velocity;
  *(v_y) = sinf(following_heading)*desired_velocity;
}

void wind_seeker::update_best_wps(const uint16_t ID)
{
  // load gas concentration at current position
  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);
  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);

  if (gas_conc > 0)
  {
    last_seen_plume = {agent_pos.x, agent_pos.y};
  }

  // update best found agent position and best found swarm position if required
  if( gas_conc > s.at(ID)->best_agent_gas)
  {
    s.at(ID)->best_agent_gas = gas_conc;
    s.at(ID)->best_agent_pos = agent_pos;
    if (gas_conc > environment.best_gas)
    {
      environment.best_gas = gas_conc;
      environment.best_gas_pos_x = agent_pos.x;
      environment.best_gas_pos_y = agent_pos.y;
    }
  }
}

void wind_seeker::generate_new_wp(const uint16_t ID)
{
  iteration_start_time = simtime_seconds;

  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);
  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);

  // wind angle computation
  int current_x_wind_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.u_wind.size())),0,environment.u_wind.size());
  int current_y_wind_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.u_wind[0].size())),0,environment.u_wind[0].size());

  float u_vel = environment.u_wind[current_x_wind_indx][current_y_wind_indx];
  float v_vel = environment.v_wind[current_x_wind_indx][current_y_wind_indx];

  float gradient_direction;

  if (gas_conc==0)
  {
    gradient_direction = rg.uniform_float(0,M_PI*2);
    if (!last_seen_plume.empty())
    {
        goal = {.x = last_seen_plume[0] + cosf(gradient_direction)*wp_travel_after_gas ,.y = last_seen_plume[1]+sinf(gradient_direction)*wp_travel_after_gas}; 
    }
    else
    {
      goal = {.x = agent_pos.x + cosf(gradient_direction)*wp_travel ,.y = agent_pos.y+sinf(gradient_direction)*wp_travel}; 
    }
     // this means the gradient is unknown so in a random direction.
  }
  else
  {

    gradient_direction = atan2f(v_vel,u_vel) + M_PI ; // +PI since we're travelling into the wind
    goal = {.x = agent_pos.x + cosf(gradient_direction)*wp_travel_after_gas ,.y = agent_pos.y+sinf(gradient_direction)*wp_travel_after_gas}; 
  }


  s.at(ID)->goal = goal; 
  s.at(ID)->gas_read = gas_conc;
  get_new_line();
  update_direction(ID);
  status = 0;

}

void wind_seeker::update_animation_direction(const uint16_t ID)
{

  // wind angle computation
  int current_x_wind_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.u_wind.size())),0,environment.u_wind.size());
  int current_y_wind_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.u_wind[0].size())),0,environment.u_wind[0].size());

  float u_vel = environment.u_wind[current_x_wind_indx][current_y_wind_indx];
  float v_vel = environment.v_wind[current_x_wind_indx][current_y_wind_indx];

  float gradient_direction = atan2f(v_vel,u_vel) + M_PI ;
  
  s.at(ID)->gradient_direction = gradient_direction;
}

void wind_seeker::update_direction(const uint16_t ID)
{
  line_heading = get_heading_to_point(agent_pos,goal); // used to follow the line
  corrected_heading = line_heading - s.at(ID)->get_orientation();
  positive_angle(&corrected_heading);

  lower_idx = (int)(corrected_heading/M_PI_2);
  upper_idx = lower_idx+1;

  cap_laser(&lower_idx);
  cap_laser(&upper_idx);

  if (abs(line_heading-(lower_idx*M_PI_2+s.at(ID)->get_orientation())) > M_PI_4)
  {
    following_laser = upper_idx;
  }
  else
  {
    following_laser = lower_idx;
  }
  
}

void wind_seeker::animation(const uint16_t ID)
{
  /*** Draw a cricle as agent ***/
  draw d;
  d.circle_loop(rangesensor);
}
