import sys, argparse, os, glob

parser = argparse.ArgumentParser(description='Prep all envs (binaries)')
parser.add_argument('n_agents',type=int,help="(int) number of agents")
args = parser.parse_args()

folder = "conf/environments/"
pic_folder = "conf/pics/"

envs = [file for file in glob.glob(folder+'*')]

for env in envs:
    pic_dir = pic_folder+env.split('/')[-1]+".png"
    if (os.path.exists(pic_dir)):
        # delete gas binary
        os.system("rm "+env+"/gas_data.bin")

        # run simulation to prep env 
        os.system("./swarmulator "+str(args.n_agents)+" 1 "+ env.split('/')[-1])
        print(env)
