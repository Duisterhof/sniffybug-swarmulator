import sys, argparse
sys.path.insert(1,'../classes')
from randomize_environment import get_spawn_pos

parser = argparse.ArgumentParser(description='Build initial positions for all environments')
parser.add_argument('folder', type=str, help="(str) Folder with envs")
parser.add_argument('n_agents',type=int,help="(int) number of agents")
parser.add_argument('min_dist',type=float,help="(float) min distance between points, to make sure multiple agents don't collide")
args = parser.parse_args()
get_spawn_pos(args.n_agents,args.folder, args.min_dist)