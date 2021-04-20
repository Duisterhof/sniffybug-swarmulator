#include "gas_sensors.h"


void load_all_lasers(const uint16_t ID)
{
  s.at(ID)->laser_ranges.clear(); // laser ranges are emptied as they need to be reloaded
  s.at(ID)->laser_pnts.clear(); // laser points (where the lasers intersect with the environment)
  
  std::vector<laser_ray> laser_rays; // contains all laser ray objects
    

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

laser_ray get_laser_reads(laser_ray ray, const uint16_t ID)
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