import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math

df = pd.read_csv('_0.csv')

slice_indx = 50

x_grid_res = 50
y_grid_res = 50
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

np.savetxt('test.txt',u_grid, delimiter=',')
plt.imshow(v_grid)
plt.show()
# plt.quiver(x_coords[::slice_indx],y_coords[::slice_indx],x_vel[::slice_indx],y_vel[::slice_indx])
# plt.show()





