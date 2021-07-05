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
parser = argparse.ArgumentParser(description='Test a policy on all environments using gasulator')
parser.add_argument('-controller', type=str, help="(str) Controller to use", default="bug_repulsion")
parser.add_argument('-agent', type=str, help="(str) Swramulator agent to use", default="particle_oriented_xy")
args = parser.parse_args()

print("Loading and building Swarmulator")
sim = swarmulator.swarmulator(verbose=True)
sim.make(controller=args.controller, agent=args.agent, clean=True, logger=False, verbose=False)
sim.set_n_agents(3)
sim.runtime_setting("time_limit", str("100")) # Time limit of each simulation 
sim.runtime_setting("simulation_realtimefactor", str("100")) # Real time factor
# Swarmulator settings
sim.runtime_setting("fitness", "source_first") # Fitness function to use (in sw/simulation/fitness_functions.h)

env_low_idx = 0
env_high_idx = 100

environments = []
for i in range(env_low_idx,env_high_idx):
    environments.append("rand_env_"+str(i))

f = sim.batch_run_envs(environments)
print(f)
print(np.average(f))
np.savetxt('sniffy_bug.txt',f,fmt='%.2f')