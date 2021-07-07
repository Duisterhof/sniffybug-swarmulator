import numpy as np
import glob

def get_spawn_pos(n_agents,folder='../../../conf/environments/',min_dist=1.0):
    files = [file for file in glob.glob(folder+'*/free_pnts.txt')]

    for file in files:
        env_folder = file.split('/')[-2]
        print("Working on: "+env_folder)
        all_points = open(file,"r").readlines()

        found_points = False

        while not found_points:
            selected_points_idxs = np.random.uniform(0,len(all_points),n_agents).astype(int)
            selected_points = [all_points[i] for i in selected_points_idxs]

            if check_distance(selected_points, min_dist):
                found_points = True
        
        write_file = open(folder+env_folder+'/spawn_pnts.txt',"w")
        write_file.writelines(selected_points)    

        headings = np.random.uniform(-np.pi,np.pi,n_agents)
        np.savetxt(folder+env_folder+'/headings.txt',headings,delimiter=" ", fmt='%.2f')    

def check_distance(points, min_dist):
    points_x = []
    points_y = []
    for point in points:
        point_split = point.split(' ')
        points_x.append(float(point_split[0]))
        points_y.append(float(point_split[1]))

    keeping_distance = True
    for i in range(len(points_x)):
        for j in range((i+1),len(points_x)):
            dx = points_x[i] - points_x[j]
            dy = points_y[i] - points_y[j]
            dist = np.sqrt(dx**2+dy**2)
            if (dist < min_dist):
                keeping_distance = False
    return keeping_distance


if __name__ == '__main__':
    get_spawn_pos(3)