"""
testing the current policy on all environments
"""

import sys, pickle, argparse, random
sys.path.insert(1,'../')
import numpy as np
from tools import fileHandler as fh
from classes import swarmulator
from classes.randomize_environment import get_spawn_pos

controllers = ['bug_repulsion','bug_repulsion','bug_repulsion','wind_seeker','gradient_seeker']
parameters = ['final_evals/parameters/manual.txt','final_evals/parameters/no_doping.txt','final_evals/parameters/doping.txt','final_evals/parameters/manual.txt','final_evals/parameters/manual.txt']
ids = ['manual_pso','no_dopoing_pso','doping_pso','wind_seeking','gradient_seeking']
fitnessess = ['source_first','source_distance_avg']

num_it = 10
batch_size = 5
policy_file = "../../conf/policies/gas_params.txt"
num_agents = 3

def evaluate_once(controller, parameters,id,fitness):
    #setting the right parameters
    fh.save_to_txt(np.loadtxt(parameters),policy_file)

    sim = swarmulator.swarmulator(verbose=True)
    sim.make(controller=controller, agent="particle_oriented_xy", clean=True, logger=False, verbose=False)
    sim.set_n_agents(num_agents)
    sim.runtime_setting("time_limit", str("100")) # Time limit of each simulation 
    sim.runtime_setting("simulation_realtimefactor", str("100")) # Real time factor
    # Swarmulator settings
    sim.runtime_setting("fitness", fitness) # Fitness function to use (in sw/simulation/fitness_functions.h)

    env_low_idx = 0
    env_high_idx = 100

    environments = []
    for i in range(env_low_idx,env_high_idx):
        environments.append("rand_env_"+str(i))

    f = []

    while np.size(environments) > 0:
        f_tmp = sim.batch_run_envs(environments[:batch_size])
        f = np.append(f,f_tmp)
        environments = environments[batch_size:]

    print(f)
    print(np.average(f))
    print(np.median(f))
    np.savetxt('final_evals/'+id+'.txt',f,fmt='%.2f')


for i in range(num_it):
    # randomizing start positions
    get_spawn_pos(num_agents,'../../conf/environments/')
    print("Iteration number: " +str(i))
    for j in range(np.size(controllers)):
        print("testing controller: " +controllers[j])
        evaluate_once(controllers[j],parameters[j],ids[j]+"_distance_"+str(i),fitnessess[1])
        evaluate_once(controllers[j],parameters[j],ids[j]+"_time_"+str(i),fitnessess[0])
