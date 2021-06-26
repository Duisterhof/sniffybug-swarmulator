import sys, argparse, os, glob, math
import numpy as np
import pandas as pd

parser = argparse.ArgumentParser(description='Build txts of the wind fields in all environments')
parser.add_argument('--x_grid_res',type=int,help="(int) number of cells in x to discretize to", default = 50)
parser.add_argument('--y_grid_res',type=int,help="(int) number of cells in y to discretize to", default = 50)
args = parser.parse_args()

folder = "conf/environments/"

envs = [file for file in glob.glob(folder+'*')]

for env in envs:
    wind_dir = env + "/wind_simulations/"
    df = pd.read_csv(wind_dir + '_0.csv')

    print("Working on environment " + env.split('/')[-1] )
    x_grid_res = args.x_grid_res
    y_grid_res = args.y_grid_res
    u_grid = np.zeros((x_grid_res,y_grid_res))
    v_grid = np.zeros((x_grid_res,y_grid_res))

    x_coords = df['Points:0'].to_numpy()
    y_coords = df['Points:1'].to_numpy()

    x_vel = df['U:0'].to_numpy()
    y_vel = df['U:1'].to_numpy()

    x_max = np.max(x_coords)
    x_min = np.min(x_coords)

    y_max = np.max(y_coords)
    y_min = np.min(y_coords)

    for i in range(len(x_vel)):
        x_indx = np.clip(math.floor(x_grid_res*(x_coords[i]-x_min)/(x_max-x_min)),0,x_grid_res-1)
        y_indx = np.clip(math.floor(y_grid_res*(y_coords[i]-y_min)/(y_max-y_min)),0,y_grid_res-1)
        u_grid[x_indx][y_indx] = x_vel[i]
        v_grid[x_indx][y_indx] = y_vel[i]

    np.savetxt(wind_dir + 'U_grid.txt',u_grid, fmt = '%f')
    np.savetxt(wind_dir + 'V_grid.txt',v_grid, fmt = '%f')
