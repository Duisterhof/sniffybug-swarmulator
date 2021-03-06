#include "environment.h"
#include "main.h"
#include "settings.h"
#include "auxiliary.h"
#include "draw.h"

#include <vector>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

Environment::Environment(void)
{

}
void Environment::load(int argc, char *argv[]){
  if (argc>=4)
  {
    environment.env_dir = argv[3];
  }
  else
  {
    environment.env_dir = param->environment();
  }
  terminalinfo::debug_msg(environment.env_dir);
  
  string s = param->agent_initialization();

  if (!strcmp(s.c_str(), "in_area")){
    complete_folder(); // runs python script to get headings and spawn points within free area
  }

  load_gas_data();

  s = param->wind_data();
  if(!strcmp(s.c_str(), "True"))
  {
    std::string U_txt_filename = "conf/environments/" + environment.env_dir  + "/wind_simulations/U_grid.txt";
    std::string V_txt_filename = "conf/environments/" + environment.env_dir  + "/wind_simulations/V_grid.txt";

    environment.u_wind = read_points(U_txt_filename);
    environment.v_wind = read_points(V_txt_filename);

  }

  define_walls();
 
  save_gas_object(gas_obj,environment.env_dir);

  if (!strcmp(param->fitness().c_str(), "food")) {
    mtx_env.lock();
    environment.define_food(100);
    environment.define_beacon(0., 0.);
    environment.nest = 8;
    mtx_env.unlock();
  }
}

void Environment::load_gas_data(void){
  string s = param->gas_seeking();
  string euclidean = param->gas_euclidean(); // feed euclidean distance instead of gas concentration (debugging)
  std::string bmp_filename;
  string filename = "conf/environments/" + environment.env_dir + "/gas_simulations/iteration_";
  string gas_obj_file = "conf/environments/" + environment.env_dir + "/gas_data.bin";
  ifstream f(gas_obj_file.c_str());
  bool gas_obj_exists = f.good();

  if (!strcmp(s.c_str(), "True") && !strcmp(euclidean.c_str(), "False")){
    
    if (gas_obj_exists)
    {
      gas_obj = load_gas_object(environment.env_dir);
    }

    else
    {
      bool last_file_found = false;
      int i = 0;
      terminalinfo::debug_msg("loading gas data ");
      while (!last_file_found)
      {
      
      if(! load_gas_file(filename+std::to_string(i),true,gas_obj) || (i > (int)(param->time_limit())))
      {
        last_file_found = true;
      }
      else{
          bmp_filename = filename+std::to_string(i)+".bmp";
          save_as_bmp(bmp_filename.c_str(),gas_obj, i);
          gas_obj.num_it = i;
          i++;
      }
      save_gas_object(gas_obj,environment.env_dir);
    
    }
    }    
  }
  else if (!strcmp(euclidean.c_str(), "True"))
  {
    load_gas_file(filename+std::to_string(0),true,gas_obj);
  }
}

void Environment::load_wind_data(void){
  string s = param->wind_data();
  string filename = "conf/environments/" + environment.env_dir + "/wind_simulations/_";
  string wind_obj_file = "conf/environments/" + environment.env_dir + "/wind_data.bin";

  ifstream f(wind_obj_file.c_str());
  bool wind_obj_exists = f.good();

  if (!strcmp(s.c_str(), "True")){
    if (wind_obj_exists)
    {
      wind_obj = load_wind_object(environment.env_dir);
    }
    
    else
    {
      bool last_file_found = false;
      int i = 0;

      terminalinfo::debug_msg("Loading wind data");
      while (!last_file_found)
      {
      
      if(! load_wind_file(filename+std::to_string(i)+".csv",true,wind_obj, gas_obj ,environment) || (i > (int)(param->time_limit())))
      {
        last_file_found = true;
      }
      else
      {
        i++;
      }
      save_wind_object(wind_obj,environment.env_dir);
    
    }
    }    
  }
}


void Environment::get_min_max(std::vector<std::vector<float>> walls)
{
  for (size_t i = 0; i < walls.size(); i++) {
    x_min = std::min({x_min,walls[i][0],walls[i][2]});
    x_max = std::max({x_max,walls[i][0],walls[i][2]});

    y_min = std::min({y_min,walls[i][1],walls[i][3]});
    y_max = std::max({y_max,walls[i][1],walls[i][3]});
    env_size = std::max({(y_max-y_min),(x_max-x_min)});
    env_diagonal = std::sqrt(env_size*env_size*2);
  }
}

void Environment::generate_dungeon(void)
{
  string s = environment.env_dir;
  if (!strcmp(s.c_str(), "random")) {
    stringstream ss("python3 scripts/python/tools/dungeon_generator.py && mkdir --p ./conf/environments/random && mv rooms.txt ./conf/environments/random/walls.txt ");
    
    system(ss.str().c_str());
    terminalinfo::info_msg("Generating random environment");
  }
}

void Environment::define_walls(void)
{
  string filename = "conf/environments/" + environment.env_dir + "/walls.txt";
  walls = read_matrix(filename);
  get_min_max(walls);
}

void Environment::complete_folder(void)
{
  string s = param->agent_initialization();
  string free_points_file = "conf/environments/" + environment.env_dir + "/spawn_pnts.txt";
  
  stringstream ss("python3 scripts/python/tools/complete_folder.py -env_name="+environment.env_dir);    
  system(ss.str().c_str());
  terminalinfo::info_msg("Locating free area");
  
  free_points = read_points(free_points_file);

  string headings_file = "conf/environments/" + environment.env_dir + "/headings.txt";
  headings = read_vector(headings_file);
}

void Environment::define_food(uint64_t n)
{
  float lim = limits();
  for (size_t i = 0; i < n; i++) {
    food.push_back(std::vector<float>());
    food[i].push_back(rg.uniform_float(-lim, lim));
    food[i].push_back(rg.uniform_float(-lim, lim));
  }
}

void Environment::define_beacon(float x, float y)
{
  beacon.push_back(x);
  beacon.push_back(y);
}

// TODO: Temporary function for initialization, but the initalization should change eventually
// only used when sequential spawning is selected
std::vector<float> Environment::start(void)
{
  std::vector<float> s(2);
  s[0] = walls[0][0] + 1.0;
  s[1] = walls[0][1] - 1.0;
  return s;
}

// TODO: Temporary function for initialization, but the initalization should change eventually
float Environment::limits(void)
{
  float max = 0;
  for (size_t i = 0; i < walls.size(); i++) {
    float v = *max_element(walls[i].begin(), walls[i].end()); // c++11
    if (max < v) {
      max = v;
    }
  }
  return max * 0.95; // 0.95 for margin
}

void Environment::add_wall(float x0, float y0, float x1, float y1)
{
  mtx.lock();
  walls.push_back(std::vector<float>());
  walls[walls.size() - 1].push_back(x0);
  walls[walls.size() - 1].push_back(y0);
  walls[walls.size() - 1].push_back(x1);
  walls[walls.size() - 1].push_back(y1);
  mtx.unlock();
}

bool Environment::sensor(const uint16_t ID, std::vector<float> s_n, std::vector<float> s, float &angle)
{
  Point p1, q1, p2, q2;
  p1.y = s[0]; // Flip axis
  p1.x = s[1];
  q1.y = s_n[0];
  q1.x = s_n[1];
  for (size_t i = 0; i < walls.size(); i++) {
    p2.x = walls[i][0];
    p2.y = walls[i][1];
    q2.x = walls[i][2];
    q2.y = walls[i][3];
    if (doIntersect(p1, q1, p2, q2)) {
      angle = atan2(p2.y - q2.y, p2.x - q2.x);
      return true;
    }
  }
  return false;
}

void Environment::animate(void)
{
  draw d;
  for (size_t i = 0; i < walls.size(); i++) {
    d.segment(walls[i][0], walls[i][1], walls[i][2], walls[i][3]);
  }

  for (size_t i = 0; i < food.size(); i++) {
    d.food(food[i][0], food[i][1]);
  }
}


void Environment::grab_food(uint64_t food_ID)
{
  float lim = limits();
  mtx_env.lock();
  // uncomment one of the two line below
  // food.erase(food.begin() + food_ID); // Use this to grab without replacement
  food[food_ID] = {rg.uniform_float(-lim, lim), rg.uniform_float(-lim, lim)}; // Use this to grab with replacement
  mtx_env.unlock();
}

void Environment::drop_food()
{
  mtx_env.lock();
  nest += 1.;
  mtx_env.unlock();
}

void Environment::eat_food(float amount)
{
  mtx_env.lock();
  if (nest > amount) { nest -= amount; }
  mtx_env.unlock();
}

void Environment::loop()
{
  float rate = (0.001 / param->simulation_updatefreq()) * s.size();
  eat_food(rate);
}
