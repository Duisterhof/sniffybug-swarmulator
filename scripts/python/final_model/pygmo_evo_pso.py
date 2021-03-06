"""
evolving a neural network for source seeking
"""

import sys, pickle, argparse, random
sys.path.insert(1,'../flat_game')
sys.path.insert(1,'../')
sys.path.insert(1,'../settings')
import numpy as np
from pygmo import *
from tools import fileHandler as fh
from classes import evolution, swarmulator
from classes.randomize_environment import get_spawn_pos

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

print("Loading and building Swarmulator")
sim = swarmulator.swarmulator(verbose=False)
sim.make(controller=args.controller, agent=args.agent, clean=True, logger=False, verbose=False)
# Swarmulator settings
sim.runtime_setting("time_limit", str("100")) # Time limit of each simulation 
sim.runtime_setting("simulation_realtimefactor", str("300")) # Real time factor
# sim.runtime_setting("environment", "image_testing") # Environment, leave empty for boundless
sim.runtime_setting("fitness", "source_distance_avg") # Fitness function to use (in sw/simulation/fitness_functions.h)


# Specify network topology
shape_file = "../../conf/policies/gas_shape.txt"
policy_file = "conf/policies/gas_params.txt"
sim.runtime_setting("policy", policy_file) 
environments = []

for i in range(100):
    string = 'rand_env_'+str(i)
    environments.append(string)


# environments = ['rand_env_1','rand_env_2','rand_env_3','rand_env_4','rand_env_5','rand_env_6','rand_env_7','rand_env_8','rand_env_9','rand_env_10']

num_params = 13
num_agents = 3

min_bounds = [-5,-5,-5,-5,0,0,0,0,0,0,0,0,0]
max_bounds = [5,5,5,5,5,50,5,5,5,1,20,20,5]

sim.set_n_agents(num_agents)

class prob_bart:
    
    def __init__(self):
        self.dim = num_params
    
    def fitness(self,x):
        fh.save_to_txt(x, sim.path+policy_file)
        f = sim.batch_run_envs(environments[:args.batchsize]) # Run with 10-20 robots, 5kimes (default args.batchsize=5)
        return [f.mean()]

    def get_bounds(self):
        return(min_bounds,max_bounds)

if __name__ == "__main__":
    get_spawn_pos(num_agents,'../../conf/environments/',0.8) # giving all agents a new spawning position   
    algo = algorithm(sga(gen=1, m=0.1))

    algo.set_verbosity(1)
    prob = problem(prob_bart())
    pop = population(prob,100)

    for i in range(400):
        get_spawn_pos(num_agents,'../../conf/environments/',0.8) # giving all agents a new spawning position
        np.random.shuffle(environments)

        print("Generation %i"%i)
        pop = algo.evolve(pop)
        print(pop.champion_f)
        fh.save_to_txt(pop.champion_x,'best_individual.txt')
        
        fitness_vector = pop.get_f()
        fh.save_to_txt(fitness_vector,'fitness_vector_'+str(i)+'.txt')
        fh.save_to_txt([pop.champion_f],'best_individual_'+str(i)+'.txt')


    print(pop)

    uda = algo.extract(sga)
    log = uda.get_log()
    #print(log)
    import matplotlib.pyplot as plt 
    plt.plot([entry[0] for entry in log],[entry[2]for entry in log], 'k--') 
    plt.show() 
    #print(model.nn.get_weights())
