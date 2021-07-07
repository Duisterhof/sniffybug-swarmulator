#ifndef AGENT_H
#define AGENT_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

#include "settings.h"
#include "terminalinfo.h"
#include CONTROLLER_INCLUDE
#include "auxiliary.h"

/**
 * Parent class defining an agent. The dynamic implementation is handled in children classes.
 */
class Agent
{
public:
  /**
   * Constructor
   */
  Agent();

  /**
   * Destructor
   */
  virtual ~Agent() {};

  uint16_t ID; // ID of agent
  bool activated;
  float dt;
  std::vector<float> state,laser_ranges; // State vector
  float orientation; // Orientation
  bool moving;
  bool happy;
  float manualx, manualy;
  bool manual;
  float manualpsi_delta;
  float gradient_direction{0.0};
  float gas_read = 0; //either gas concentration or distance to source
  float gas_read_lp = 1.0;
  float laser_collision_threshold = 0.1; //if one of the laser rangers detects < 0.1 [m], we count it as a collision
  float swarm_collision_threshold = 0.5; // if two agents get within x[m], it's a crash for both agents
  std::vector<std::vector<float>> laser_pnts, intersect_walls;
  random_generator rg;
  Controller *controller = new CONTROLLER;
  Point best_agent_pos = {.x = 0.0,.y = 0.0}; // position of best position agent has seen 
  Point goal = {.x= 0.0,.y=0.0}; // goal position in PSO
  float best_agent_gas = -1.0; // best found gas concentration
  Line line;
  Point wall_init;
  float distance_accumulator = 0.0;
  int num_steps = 0;
  int num_close_to_source = 0;
  float first_in_source = 100.0;
  bool is_crashed = false;
  /**
   * Returns the position of the agent along a certain dimension (i.e., North(0) and East(1))
   * This is used by the OmniscientObserver class in order to simulate the presence of sensors.
   * @param dim dimension x or y
   * @return position along x or y
   */
  float get_position(uint16_t dim);

  /**
   * Get the orientation object
   *
   * @return float
   */
  float get_orientation();

  /**
   * Get the state along a given dimension
   *
   * @param dim dimension
   * @return float
   */
  float get_state(const uint16_t dim);

  /**
   * The agent class is only the parent class of a child class that specifies the dynamics and control
   * of a given agent (robot/animal/whatever). The state_update function is handled by the child class.
   * @param s current state vector
   * @return next state vector
   */
  virtual std::vector<float> state_update(std::vector<float>) = 0;

  /**
   * The agent class is only the parent class of a child class that specifies the dynamics and control
   * of a given agent (robot/animal/whatever). The way that the agent looks is specified in the child class.
   */
  virtual void animation() = 0;
};

#endif /*AGENT_H*/
