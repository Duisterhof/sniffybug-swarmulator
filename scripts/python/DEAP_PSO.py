import random, sys, pickle, argparse
import numpy as np
from tools import fileHandler as fh
from classes import evolution, swarmulator

## Run as
# python3 main_standard_evolution.py CONTROLLER AGENT
# Example:
# python3 main_standard_evolution.py aggregation particle

#####################
#  Argument parser  #
#####################
parser = argparse.ArgumentParser(description='Evolve a controller using swarmulator')
parser.add_argument('-controller', type=str, help="(str) Controller to use", default="bug_repulsion")
parser.add_argument('-agent', type=str, help="(str) Swramulator agent to use", default="particle_oriented_xy")
parser.add_argument('-gen', type=int, help="(int) Max number generations, default = 100", default=400)
parser.add_argument('-batchsize', type=int, help="(int) Batch size. How many parallel tests to try, default = 10", default=5)
parser.add_argument('-resume', type=str, help="(str) Resume after quitting from the indicated saved file, default = None", default=None)
parser.add_argument('-plot', type=str, help="(str) If set, it will plot the evolution from a saved run, default = None", default=None)
parser.add_argument('-id', type=int, help="(int) Evolutionary run ID, default = 1", default=1)
args = parser.parse_args()

##########################
#  Load Swarmulator API  #
##########################
print("Loading and building Swarmulator")
sim = swarmulator.swarmulator(verbose=False)
sim.make(controller=args.controller, agent=args.agent, clean=True, logger=False, verbose=False)
# Swarmulator settings
sim.runtime_setting("time_limit", str("100")) # Time limit of each simulation 
sim.runtime_setting("simulation_realtimefactor", str("300")) # Real time factor
# sim.runtime_setting("environment", "image_testing") # Environment, leave empty for boundless
sim.runtime_setting("fitness", "source_distance_avg") # Fitness function to use (in sw/simulation/fitness_functions.h)
filename = "evo_run_%s_%s_%i" % (args.controller, args.agent, args.id)
print("This run will save at every new generation in the file %s.pkl" % filename)
print("If you want to resume, please load it using the -resume input option.")

# Specify network topology
shape_file = "../../conf/policies/gas_shape.txt"
policy_file = "conf/policies/gas_params.txt"
sim.runtime_setting("policy", policy_file) 
environments = ['rand_env_1','rand_env_2','rand_env_4','rand_env_5','rand_env_6','rand_env_7','rand_env_8','rand_env_9','rand_env_10']
num_params = 13
num_agents = 3
sim.set_n_agents(num_agents)

min_bounds = [-5,-5,-5,-5,0,0,0,0,0,0,0,0,0]
max_bounds = [5,5,5,5,5,100,100,5,5,5,1,20,20]

######################
#  Fitness function  #
######################
def fitness(individual):
	scaled_individual = [j*(max_bounds[i]-min_bounds[i])+min_bounds[i] for i,j in enumerate(individual)]

	fh.save_to_txt(scaled_individual, sim.path+policy_file)
	f = sim.batch_run_envs(environments[:args.batchsize])
	return f.mean(), # Fitness = average (note trailing comma to cast to tuple!)

########################
#  Load evolution API  #
########################
e = evolution.evolution()
# Specify the genome length and the population size
e.setup(fitness, GENOME_LENGTH=num_params, POPULATION_SIZE=50,NUM_AGENTS=num_agents)

# Do not evolve, but only plot an evolution file as specified in args.plot
if args.plot is not None:
    e.load(args.plot)
    e.plot_evolution()

# Resume evolution from file args.resume
elif args.resume is not None:
	e.load(args.resume)
	p = e.evolve(verbose=True, generations=args.gen, checkpoint=filename, population=e.pop)

# Just run normally from the beginning
else:
    p = e.evolve(verbose=True, generations=args.gen, checkpoint=filename)

# Save
e.save(filename)
