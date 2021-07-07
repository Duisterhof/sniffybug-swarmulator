"""
testing the current policy on all environments
"""

import sys, pickle, argparse, random
sys.path.insert(1,'../')
import numpy as np
from classes import swarmulator

#####################
#  Argument parser  #
#####################
batch_size = 5

parser = argparse.ArgumentParser(description='Test a policy on all environments using gasulator')
parser.add_argument('-controller', type=str, help="(str) Controller to use", default="wind_seeker")
parser.add_argument('-agent', type=str, help="(str) Swramulator agent to use", default="particle_oriented_xy")
args = parser.parse_args()

print("Loading and building Swarmulator")
sim = swarmulator.swarmulator(verbose=True)
sim.make(controller=args.controller, agent=args.agent, clean=True, logger=False, verbose=False)
sim.set_n_agents(3)
sim.runtime_setting("time_limit", str("100")) # Time limit of each simulation 
sim.runtime_setting("simulation_realtimefactor", str("5")) # Real time factor
# Swarmulator settings
sim.runtime_setting("fitness", "source_distance_avg") # Fitness function to use (in sw/simulation/fitness_functions.h)

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
np.savetxt('final_evals/wind_seeker_avg_distance.txt',f,fmt='%.2f')
