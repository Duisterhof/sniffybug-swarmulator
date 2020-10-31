import sys, argparse, os, glob
sys.path.insert(1,'scripts/python/classes')
from randomize_environment import get_spawn_pos

parser = argparse.ArgumentParser(description='Build initial positions for all environments')
parser.add_argument('n_agents',type=int,help="(int) number of agents")
parser.add_argument('min_dist',type=float,help="(float) min distance between points, to make sure multiple agents don't collide")
args = parser.parse_args()

folder = "conf/environments/"
pic_folder = "conf/pics/"

envs = [file for file in glob.glob(folder+'*')]

for env in envs:
    pic_dir = pic_folder+env.split('/')[-1]+".png"
    if (os.path.exists(pic_dir)):
        print(pic_dir)
        os.system("cp "+pic_dir+" "+env)
        os.system("python3 scripts/python/tools/complete_folder.py -env_name="+env.split('/')[-1])

get_spawn_pos(args.n_agents,folder, args.min_dist)