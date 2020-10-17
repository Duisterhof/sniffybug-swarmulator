#include "bug_repulsion.h"
#include "draw.h"
#include "randomgenerator.h"
#include <tuple>
#include "main.h"
#include <math.h>
#include "auxiliary.h"
#include "omniscient_observer.h"

void bug_repulsion::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  state = s.at(ID)->state; // load agent state
  agent_pos.x = state[1]; // loading agent pos struct Point
  agent_pos.y = state[0];
  load_all_lasers(ID); // modelling multirangers
  update_best_wps(ID); // use gas sensing to update the best seen points 

  if ( simtime_seconds-iteration_start_time >= update_time || getDistance(goal,agent_pos) < dist_reached_goal )
  {
    generate_new_wp(ID);
    status = 0;
  }
  else if (simtime_seconds == 0.0)
  {
    // initial velocity for everyone
    goal = {.x = rg.uniform_float(environment.x_min,environment.x_max), .y=rg.uniform_float(environment.y_min,environment.y_max)};
    s.at(ID)->goal = goal; 
    get_new_line();
    update_direction(ID);
  }

  update_status(ID); // get new status
  switch(status)
  {
    case 0:
      follow_line(&v_x, &v_y);
      break;
    case 1:
      follow_wall(ID, &v_x,&v_y);
      break;
    case 2:
      repulse_swarm(ID, &v_x, &v_y);
      break;
  }
  terminalinfo::debug_msg(std::to_string(status));
  // terminalinfo::debug_msg(std::to_string(v_y));
  
}

void bug_repulsion::wall_follow_init(const uint16_t ID)
{
  std::vector<float> lasers =  s.at(ID)->laser_ranges;
  int min_laser_idx = std::distance(lasers.begin(),std::min_element(lasers.begin(),lasers.end())); // min laser idx
  start_laser = min_laser_idx ;
  terminalinfo::debug_msg(std::to_string(lasers[min_laser_idx]));
  cap_laser(&start_laser);
  max_reached_laser = start_laser;

  int right_idx = start_laser + 1;
  int left_idx = start_laser - 1;
  cap_laser(&right_idx);
  cap_laser(&left_idx);

  float heading_right = right_idx*M_PI_2 + s.at(ID)->get_orientation();
  float heading_left = left_idx*M_PI_2 +s.at(ID)->get_orientation();

  positive_angle(&heading_right);
  positive_angle(&heading_left);

  float heading_to_wp = get_heading_to_point(agent_pos,goal);
  positive_angle(&heading_to_wp);

  if (abs(heading_to_wp-heading_left) < abs(heading_to_wp-heading_right))
  {
    search_left = true;
  }
  else
  {
    search_left = false;
  }

}

void bug_repulsion::update_start_laser(void)
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

void bug_repulsion::follow_wall(const uint16_t ID, float* v_x, float* v_y)
{

  update_start_laser(); // avoid osscialations
  int local_laser_idx = start_laser_corrected;
  int forbidden_direction = follow_laser - 2;
  cap_laser(&forbidden_direction);
  if (search_left == false)
  {
    for (int i = start_laser_corrected; i < (start_laser_corrected+4); i++)
    {

      local_laser_idx = i;
      cap_laser(&local_laser_idx);

      if (local_laser_idx != forbidden_direction)
      {
        if (abs(start_laser-i) > max_turns)
        {
          reset_wall_follower();
        }

        if (s.at(ID)->laser_ranges[local_laser_idx] > laser_warning)
        {
          if (local_laser_idx > max_reached_laser)
          {
            max_reached_laser = local_laser_idx;
          }
          break;
        }
      }
    }
  }
  else
  {
    for (int i = start_laser_corrected; i > (start_laser_corrected-4); i--)
    {
      local_laser_idx = i;
      cap_laser(&local_laser_idx);
      
      if (local_laser_idx != forbidden_direction)
      {
        if (abs(start_laser-i) > max_turns)
        {
          reset_wall_follower();
        }

        if (s.at(ID)->laser_ranges[local_laser_idx] > laser_warning)
        {
          if (local_laser_idx < max_reached_laser)
          {
            max_reached_laser = local_laser_idx;
          }
          break;
        }
      }
    }
  }
  follow_laser = local_laser_idx;
  *(v_x) = cosf(follow_laser*M_PI_2)*desired_velocity;
  *(v_y) = sinf(follow_laser*M_PI_2)*desired_velocity;


}

void bug_repulsion::reset_wall_follower(void)
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

void bug_repulsion::repulse_swarm(const uint16_t ID, float* v_x, float* v_y)
{
  *(v_x) = 0;
  *(v_y) = 0;

  // attraction to waypoint
  float local_psi = get_heading_to_point(agent_pos,goal) ;
  positive_angle(&local_psi);
  *(v_x) = cosf(local_psi)*desired_velocity;
  *(v_y) = sinf(local_psi)*desired_velocity;


  // add repulsion from lasers      
  for ( int i = 0; i<4; i++)
  {
    if ( s.at(ID)->laser_ranges[i] < laser_warning)
    {
      float laser_heading = laser_headings[i];
      float heading_away_from_laser = laser_heading - M_PI;
      *(v_x) += cosf(heading_away_from_laser)*k_swarm_laser_rep*(laser_warning-s.at(ID)->laser_ranges[i]);
      *(v_y) += sinf(heading_away_from_laser)*k_swarm_laser_rep*(laser_warning-s.at(ID)->laser_ranges[i]);
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
  
  float vector_size = sqrt(pow(*(v_x),2)+pow(*(v_y),2));

  *(v_x) = *(v_x)/vector_size*desired_velocity;
  *(v_y) = *(v_y)/vector_size*desired_velocity;
  // if (abs(*(v_x)) > abs(*(v_y)))
  // {
  //   *(v_x) = desired_velocity;
  //   *(v_y) = 0.0;
  // }
  // else
  // {
  //   *(v_y) = desired_velocity;
  //   *(v_x) = 0.0;
  // }
  
}


void bug_repulsion::update_status(const uint16_t ID)
{
  closest_agents = o.request_closest(ID);
  
  if (closest_agents.size() > 0 && get_agent_dist(closest_agents[0],ID) < swarm_warning)
  {
    status = 2;
  }
  else if (previous_status == 2)
  {
    generate_new_wp(ID);
    status = 0;
  }
  else if (previous_status == 0 && (s.at(ID)->laser_ranges[lower_idx] < laser_warning || s.at(ID)->laser_ranges[upper_idx] < laser_warning))
  {
    status = 1;
    start_wall_avoid = agent_pos;
    wall_follow_init(ID);
  }
  // else if (previous_status == 1 && avoided_obstacle(ID))
  // {
  //   status = 0;
  // }

  previous_status = status;

}

float bug_repulsion::get_agent_dist(const uint16_t ID1, const uint16_t ID2)
{
  return(sqrtf(pow(s.at(ID1)->state[0]-s.at(ID2)->state[0],2)+pow(s.at(ID1)->state[1]-s.at(ID2)->state[1],2)));
}

// float bug_repulsion::get_laser_min(const uint16_t ID)
// {
//   std::vector<float> lasers =  s.at(ID)->laser_ranges;
//   int min_laser_idx = std::distance(lasers.begin(),std::min_element(lasers.begin(),lasers.end())); // min laser idx
//   return(lasers[min_laser_idx]);
// }

bool bug_repulsion::avoided_obstacle(const uint16_t ID)
{
  if (get_distance_to_line(line_to_goal,agent_pos) > line_max_dist)
  {
    return false;
  }
  else if (getDistance(start_wall_avoid,agent_pos) > min_obs_avoid_thres && s.at(ID)->laser_ranges[lower_idx] > laser_warning && s.at(ID)->laser_ranges[upper_idx] > laser_warning   )  
  {
    return true;
  }
  else
  {
    return false;
  }
  
  
}

// updates individual and swarm best wps
void bug_repulsion::get_new_line(void)
{
  line_to_goal.p0 = agent_pos;
  line_to_goal.p1 = goal;
  update_line(&line_to_goal);
}

void bug_repulsion::update_follow_laser(void)
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
void bug_repulsion::follow_line(float* v_x, float* v_y)
{
  if (get_distance_to_line(line_to_goal,agent_pos) > line_max_dist)
  {
    update_follow_laser();
  }
  following_heading = following_laser*M_PI_2;
  *(v_x) = cosf(following_heading)*desired_velocity;
  *(v_y) = sinf(following_heading)*desired_velocity;


}

void bug_repulsion::update_best_wps(const uint16_t ID)
{
  // load gas concentration at current position
  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);
  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);

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

void bug_repulsion::generate_new_wp(const uint16_t ID)
{
  iteration_start_time = simtime_seconds;
  float r_p = rg.uniform_float(0,1);
  float r_g = rg.uniform_float(0,1);
  
  random_point = {.x = rg.uniform_float(environment.x_min,environment.x_max),.y = rg.uniform_float(environment.y_min,environment.y_max)};
  float v_x = rand_p*(random_point.x-agent_pos.x)+omega*(goal.x-agent_pos.x)+phi_p*r_p*(s.at(ID)->best_agent_pos.x-agent_pos.x)+phi_g*r_g*(environment.best_gas_pos_x-agent_pos.x);
  float v_y = rand_p*(random_point.y-agent_pos.y)+omega*(goal.y-agent_pos.y)+phi_p*r_p*(s.at(ID)->best_agent_pos.y-agent_pos.y)+phi_g*r_g*(environment.best_gas_pos_y-agent_pos.y);
  goal = {.x = agent_pos.x + v_x,.y = agent_pos.y+v_y}; 

  s.at(ID)->goal = goal; 
  get_new_line();
  update_direction(ID);
}

void bug_repulsion::update_direction(const uint16_t ID)
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



void bug_repulsion::load_all_lasers(const uint16_t ID)
{
  s.at(ID)->laser_ranges.clear(); // laser ranges are emptied as they need to be reloaded
  s.at(ID)->laser_pnts.clear(); // laser points (where the lasers intersect with the environment)
  laser_rays.clear();

  // load laser rays
  for (int i = 0; i<4; i++)
	{
    laser_ray ray;
    ray.heading = laser_headings[i];
		laser_rays.push_back(ray);
	}

  for (int i = 0; i<4; i++)
  {
    laser_rays[i] = get_laser_reads(laser_rays[i],ID);
  }
}

laser_ray bug_repulsion::get_laser_reads(laser_ray ray, const uint16_t ID)
{
  //init
  Point laser_point,wall_start, wall_end, agent_pos;
  
  std::vector<float> state = s.at(ID)->state;
  float heading = s.at(ID)->get_orientation() + ray.heading; //global laser ray heading
  
  //construct a point in the right direction that is outside of the environment: laser_point
  rotate_xy(0,environment.env_diagonal,-heading,laser_point.x,laser_point.y);
  laser_point.x += state[1];
  laser_point.y += state[0];

  agent_pos.x = state[1];
  agent_pos.y = state[0];

  // looping through all walls to check if laser ray intersects with it and find the closest wall
  // we check if two lines intersect: (wall_start-wall_end) and (agent_pos-laser_point)
  for(uint i = 0; i<environment.walls.size();i++)
  {
    // init points to be used later
    wall_start.x = environment.walls[i][0];
    wall_start.y = environment.walls[i][1];
    wall_end.x = environment.walls[i][2];
    wall_end.y = environment.walls[i][3];


    std::tuple<bool,Point> intersect_return;
    //get intersection point and a bool if it's on the wall or not
    intersect_return = getIntersect(agent_pos,laser_point,wall_start,wall_end);

    bool on_wall = std::get<0>(intersect_return);
    Point intersect = std::get<1>(intersect_return);

    //storing intersecting walls and its intersection with the laser
    if( on_wall == true){
      ray.intersection_points.push_back(intersect);
      ray.intersecting_walls.push_back(environment.walls[i]);
      s.at(ID)->intersect_walls.push_back(environment.walls[i]);
    }
  }

  // if we found some intersection points
  if ( ray.intersection_points.size() > 0 )
  {
    // get a list of all distances to all intersection points
    for (uint i=0; i<ray.intersection_points.size();i++)
    {
      ray.distances.push_back(getDistance(agent_pos,ray.intersection_points[i]));
    }

    //arg max
    int idx = std::distance(ray.distances.begin(),std::min_element(ray.distances.begin(),ray.distances.end()));
    ray.range = ray.distances[idx];
    s.at(ID)->laser_ranges.push_back(ray.range);
    std::vector<float> v = {ray.intersection_points[idx].x,ray.intersection_points[idx].y};
    s.at(ID)->laser_pnts.push_back(v);
    
  }
  // we didn't find anything (this should be rare if not impossible), we we return the end of the projected laser beams and their size
  else
  {
    ray.range = environment.env_diagonal;
    s.at(ID)->laser_ranges.push_back(environment.env_diagonal);
    std::vector<float> v = {laser_point.x,laser_point.y};
    s.at(ID)->laser_pnts.push_back(v);
  }

  return ray;
}


void bug_repulsion::animation(const uint16_t ID)
{
  /*** Draw a cricle as agent ***/
  draw d;
  d.circle_loop(rangesensor);
}
