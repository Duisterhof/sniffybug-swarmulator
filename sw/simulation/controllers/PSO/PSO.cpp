#include "PSO.h"
#include "draw.h"
#include "randomgenerator.h"
#include <tuple>
#include "main.h"
#include <math.h>
#include "auxiliary.h"
#include "omniscient_observer.h"

// main PSO logic
// PSO with constant heading, changing the particle's heading by modifying vx and vy
// this in order to maintain constant yaw for better UWB estimates
void PSO::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  /*** Put your controller here ***/
  random_generator rg;
  std::vector<float> state = s.at(ID)->state; // load agent state
  s.at(ID)->laser_ranges.clear(); // laser ranges are emptied as they need to be reloaded
  s.at(ID)->laser_pnts.clear(); // laser points (where the lasers intersect with the environment)
  agent_pos.x = state[1]; // loading agent pos struct Point
  agent_pos.y = state[0];
  laser_rays.clear();


  //*** GENERATE NEW WP ***//

  if ( simtime_seconds-iteration_start_time >= update_time || getDistance(goal,agent_pos) < dist_reached_goal )
  {
    iteration_start_time = simtime_seconds;
    float r_p = rg.uniform_float(0,1);
    float r_g = rg.uniform_float(0,1);
    
    random_point = {.x = rg.uniform_float(environment.x_min,environment.x_max),.y = rg.uniform_float(environment.y_min,environment.y_max)};
    float v_x = rand_p*(random_point.x-agent_pos.x)+omega*(goal.x-agent_pos.x)+phi_p*r_p*(s.at(ID)->best_agent_pos.x-agent_pos.x)+phi_g*r_g*(environment.best_gas_pos_x-agent_pos.x);
    float v_y = rand_p*(random_point.y-agent_pos.y)+omega*(goal.y-agent_pos.y)+phi_p*r_p*(s.at(ID)->best_agent_pos.y-agent_pos.y)+phi_g*r_g*(environment.best_gas_pos_y-agent_pos.y);
    goal = {.x = agent_pos.x + v_x,.y = agent_pos.y+v_y}; 

    s.at(ID)->goal = goal;
    got_new_wp = true;
  }
  else if (simtime_seconds == 0.0)
  {
    // initial velocity for everyone
    goal = {.x = rg.uniform_float(environment.x_min,environment.x_max), .y=rg.uniform_float(environment.y_min,environment.y_max)};
    local_psi = get_heading_to_point(agent_pos,goal);
  }


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

  local_psi = get_heading_to_point(agent_pos,goal) ;

  if (local_psi < 0)
  {
    local_psi += 2*M_PI;
  }

  // activate attraction-repulsion framework when close to another agent or very low laser read
  if (critic_avoid)
  {
    // attraction to source
    local_vx = cosf(local_psi)*desired_velocity;
    local_vy = sinf(local_psi)*desired_velocity;

    // add repulsion to walls        
    for( int i = 0; i<4; i++)
    {
      if ( s.at(ID)->laser_ranges[i] < desired_laser_distance)
      {
        float laser_heading = laser_headings[i] + s.at(ID)->get_orientation();
        float heading_away_from_laser = laser_heading - M_PI;
        local_vx += cosf(heading_away_from_laser)*k_swarm_laser_rep*powf(desired_laser_distance-s.at(ID)->laser_ranges[i],2);
        local_vy += sinf(heading_away_from_laser)*k_swarm_laser_rep*powf(desired_laser_distance-s.at(ID)->laser_ranges[i],2);
      }
    }

    // repulsion from other agents
    std::vector<uint> closest_ids = o.request_closest(ID);
    check_swarm_close(closest_ids, ID);
    if ( closest_ids.size() > 0 )
    {
      if (get_agent_dist(ID,closest_ids[0]) < swarm_avoidance_thres)
      {
        // variable used to add a timeout for swarm avoidance
        if (started_agent_avoid == false)
        {
          started_agent_avoid = true;
          started_swarm_avoid_time = simtime_seconds;
        }
        // add a force for all agents that are within a range
        for (uint i =0; i<closest_ids.size(); i++)
        {
          if (get_agent_dist(ID,closest_ids[i]) < swarm_avoidance_thres)
          {
            other_agent_pos.x = s.at(closest_ids[i])->state[1];
            other_agent_pos.y = s.at(closest_ids[i])->state[0];
            float heading_to_other_agent = get_heading_to_point(agent_pos,other_agent_pos);
            float heading_away_from_agent = heading_to_other_agent - M_PI;
            local_vx += cosf(heading_away_from_agent)*k_swarm_avoidance*swarm_avoidance_thres-get_agent_dist(ID,closest_ids[i]);
            local_vy += sinf(heading_away_from_agent)*k_swarm_avoidance*swarm_avoidance_thres-get_agent_dist(ID,closest_ids[i]);
          }
        }
        // reroute after certain time

        if ((simtime_seconds-started_swarm_avoid_time)> swarm_rerout_time)
        {
          started_swarm_avoid_time = simtime_seconds;
          iteration_start_time = simtime_seconds;
          float r_p = rg.uniform_float(0,1);
          float r_g = rg.uniform_float(0,1);
          
          random_point = {.x = rg.uniform_float(environment.x_min,environment.x_max),.y = rg.uniform_float(environment.y_min,environment.y_max)};
          float v_x = rand_p*(random_point.x-agent_pos.x)+omega*(goal.x-agent_pos.x)+phi_p*r_p*(s.at(ID)->best_agent_pos.x-agent_pos.x)+phi_g*r_g*(environment.best_gas_pos_x-agent_pos.x);
          float v_y = rand_p*(random_point.y-agent_pos.y)+omega*(goal.y-agent_pos.y)+phi_p*r_p*(s.at(ID)->best_agent_pos.y-agent_pos.y)+phi_g*r_g*(environment.best_gas_pos_y-agent_pos.y);
          goal = {.x = agent_pos.x + v_x,.y = agent_pos.y+v_y}; 

          s.at(ID)->goal = goal;
        }
      }
      else
      {
      started_agent_avoid = false;
      }
    }
  }
  // if not critic, we do wall following instead
  else
  {
    // check if we should be in this mode
    std::vector<uint> closest_ids = o.request_closest(ID);
    check_swarm_close(closest_ids, ID);
    // if we first start wall following, determine the desired direction (+x, -x, +y, -y)
    if (init_wall_following || got_new_wp )
    {
      x_y_timeout = simtime_seconds;
      desired_laser = (int)((local_psi-s.at(ID)->get_orientation())/M_PI_2); 

      if (desired_laser > 3)
      {
        desired_laser -= 4;
      }
      else if( desired_laser < 0)
      {
        desired_laser += 4;
      }
      random_steering = rg.uniform_float(0,1);
      if (random_steering > 0.2)
      {
        desired_direction = s.at(ID)->get_orientation() + desired_laser*M_PI_2;
        if ((local_psi-desired_direction) > M_PI_4)
        {
          desired_laser += 1;
          search_left = true;
        }
        else
        {
          search_left = false;
        }
      }
      else
      {
        if (random_steering > 0.1)
        {
          desired_laser += 1;
          search_left = true;
        }
        else
        {
          search_left = false;
        }
      }
      
     
      
      init_wall_following = false;
      got_new_wp = false;
      start_searching_laser = desired_laser;
      max_reached_laser = start_searching_laser;
    }
    lasers = s.at(ID)->laser_ranges;

    

    // we 'rotate' left to find the source
    if (search_left)
    {
        // determine start laser to check
        if (max_reached_laser < desired_laser)
        {
          start_searching_laser = max_reached_laser + 1;
        }
        else
        {
          start_searching_laser = desired_laser;
        }
        // start_searching_laser = desired_laser;
        terminalinfo::debug_msg("start");
        terminalinfo::debug_msg(std::to_string(start_searching_laser));
        terminalinfo::debug_msg(std::to_string(desired_laser-4));
        first_safe_laser = -1;
        for (int i = start_searching_laser; i > (start_searching_laser-4) ; i--)
        {
          terminalinfo::debug_msg(std::to_string(i));
          // round to 0-3 bounds
          if (i < 0)
          {
            laser_idx = i + 4;
          }
          else if (i > 3)
          {
            laser_idx = i-4;
          }
          else
          {
            laser_idx = i;
          }

          if (lasers[laser_idx] > desired_laser_distance && first_safe_laser == -1)
          {
            first_safe_laser = i;
          }
        }

        if (first_safe_laser < max_reached_laser)
        {
          max_reached_laser = first_safe_laser;
        }

        desired_psi = s.at(ID)->get_orientation() + first_safe_laser*M_PI_2;
    }
        // we 'rotate' left to find the source
    else
    {

        // determine start laser to check
        if (max_reached_laser > desired_laser)
        {
          start_searching_laser = max_reached_laser - 1;
        }
        else
        {
          start_searching_laser = desired_laser;
        }
        // start_searching_laser = desired_laser;
        terminalinfo::debug_msg("start");
        terminalinfo::debug_msg(std::to_string(start_searching_laser));
        terminalinfo::debug_msg(std::to_string(desired_laser+4));
        first_safe_laser = -1;
        for (int i = start_searching_laser; i < (start_searching_laser+4) ; i++)
        {
          terminalinfo::debug_msg(std::to_string(i));
          // round to 0-3 bounds
          if (i < 0)
          {
            laser_idx = i + 4;
          }
          else if (i > 3)
          {
            laser_idx = i-4;
          }
          else
          {
            laser_idx = i;
          }

          if (lasers[laser_idx] > desired_laser_distance && first_safe_laser == -1)
          {
            first_safe_laser = i;
          }
        }
        
        if (first_safe_laser > max_reached_laser)
        {
          max_reached_laser = first_safe_laser;
        }


        desired_psi = s.at(ID)->get_orientation() + first_safe_laser*M_PI_2;
    
    }
    desired_psi = s.at(ID)->get_orientation() + first_safe_laser*M_PI_2;
    local_vx = cosf(desired_psi)*desired_velocity;
    local_vy = sinf(desired_psi)*desired_velocity;   
  }
  
  // load gas concentration at current position
  int x_indx = clip((int)((s.at(ID)->state[1]-environment.x_min)/(environment.x_max-environment.x_min)*(float)(environment.gas_obj.numcells[0])),0,environment.gas_obj.numcells[0]);
  int y_indx = clip((int)((s.at(ID)->state[0]-environment.y_min)/(environment.y_max-environment.y_min)*(float)(environment.gas_obj.numcells[1])),0,environment.gas_obj.numcells[1]);
  float gas_conc = (float)(environment.gas_obj.gas_data[(int)(floor(simtime_seconds))][x_indx][y_indx]);

  // update best found agent position and best found swarm position if required
  if( gas_conc>s.at(ID)->best_agent_gas)
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
 
  // new goal is computed every 'update_time' [sec]
  // normalize vector to original desired velocity size
  float vector_size = sqrtf(powf(local_vx,2)+powf(local_vy,2));
  local_vx = local_vx/vector_size*desired_velocity;
  local_vy = local_vy/vector_size*desired_velocity;

  v_x = local_vx;
  v_y = local_vy;
  }
void PSO::animation(const uint16_t ID)
{
  /*** Put the animation of the controller/sensors here ***/
  draw d;
  d.circle_loop(rangesensor);
}

float PSO::get_agent_dist(const uint16_t ID1, const uint16_t ID2)
{
  return(sqrtf(pow(s.at(ID1)->state[0]-s.at(ID2)->state[0],2)+pow(s.at(ID1)->state[1]-s.at(ID2)->state[1],2)));
}


/**
 * Helper function to model laser ranger in a specified direction
 * @param (Point) agent_position: current position of agent to consider
 * @param (float) heading: orientation of the laser ranger for the agent
 * @param (Point) wall_1: first point on the wall
 * @param (Point) wall_2: second point on the wall
*/

float PSO::get_ray_control(laser_ray ray, float dt)
{
  ray.heading_error = powf((ray.desired_laser_distance - ray.range),2);
  ray.heading_error_d = (ray.heading_error-ray.old_heading_error)/dt;
  ray.heading_error_i = 0.5*(ray.heading_error+ray.old_heading_error)*dt;

  ray.old_heading_error = ray.heading_error;
  float final_control = ray.heading_kp*ray.heading_error + ray.heading_kd*ray.heading_error_d + ray.heading_ki*ray.heading_error_i;
  // float diff_control = final_control - old_accumulator;

  // if (diff_control > ray.max_accumulator_increase)
  // {
  //   final_control = old_accumulator + ray.max_accumulator_increase;
  // }
  // else if (diff_control < -ray.max_accumulator_increase)
  // {
  //   final_control = old_accumulator - ray.max_accumulator_increase;
  // }
  // old_accumulator = final_control;
  return(final_control);
}

bool PSO::get_follow_direction(std::vector<float> ranges, float desired_heading, float agent_heading)
{
  if( desired_heading < 0)
  {
    desired_heading += M_PI*2;
  }
  if (agent_heading < 0)
  {
    agent_heading += M_PI*2;
  }
  if ( desired_heading< agent_heading)
  {
    desired_heading += M_PI*2;
  }
  int lower_idx = (int)((desired_heading-agent_heading)/M_PI_2); //the quadrant in which the desired heading lies
  int upper_idx = lower_idx + 1;
  if (upper_idx == 4)
  {
    upper_idx = 0;
  }

  if (ranges[lower_idx] > ranges[upper_idx])
  {
    return true;
  }
  else
  {
    return false;
  }
}
void PSO::check_swarm_close(std::vector<uint> closest_ids, uint self_id)
{
  if (closest_ids.size() == 0 )
  {
    critic_avoid = false;
  }
  else
  {
    if ( get_agent_dist(closest_ids[0],self_id) < swarm_avoidance_thres )
    {
      critic_avoid = true;
    }
    else if (get_agent_dist(closest_ids[0],self_id) < swarm_release_thres && critic_avoid == true )
    {
      critic_avoid = true;
    }
    else
    {
      critic_avoid = false;
    }
    
  }
  
}

// returns if going in a desired direction is safe
bool PSO::get_safe_direction(std::vector<float> ranges, float desired_heading, float threshold, float agent_heading)
{
  if( desired_heading < 0)
  {
    desired_heading += M_PI*2;
  }
  if (agent_heading < 0)
  {
    agent_heading += M_PI*2;
  }
  if ( desired_heading< agent_heading)
  {
    desired_heading += M_PI*2;
  }
  int lower_idx = (int)((desired_heading-agent_heading)/M_PI_2); //the quadrant in which the desired heading lies
  int upper_idx = lower_idx + 1;
  if (upper_idx == 4)
  {
    upper_idx = 0;
  }
 

  if (ranges[lower_idx] > threshold && ranges[upper_idx] > threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

laser_ray PSO::get_laser_reads(laser_ray ray, const uint16_t ID)
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