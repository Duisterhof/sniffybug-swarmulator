import numpy as np
import matplotlib.pyplot as plt
from scipy import stats


def ttest(group_a,group_b):
    len_a = len(group_a)
    len_b = len(group_b)

    mean_a = np.mean(group_a)
    mean_b = np.mean(group_b)
    
    var_a = np.var(group_a)
    var_b = np.var(group_b)

    return (mean_a-mean_b)/np.sqrt(var_a/len_a+var_b/len_b)

baseline = np.loadtxt('../manual_dist_avg.txt')
gradient = np.loadtxt('../gradient_seeker_avg_distance_live_gradient.txt')
wind = np.loadtxt('../wind_seeker_avg_distance.txt')
evolved = np.loadtxt('../evolved_no_doping_dist_avg.txt')
evolved_doping = np.loadtxt('../evolved_doping_dist_avg.txt')

num_it = 100000

null = baseline
competitor = wind

merged = np.append(null,competitor)

t_init = ttest(null,competitor)

null_2 = null-np.mean(null)+np.mean(merged)
competitor_2 = competitor-np.mean(competitor)+np.mean(merged)

# cohens_d = []
bootstrap_ts = []
for i in range(num_it):
    idxs_a = np.random.randint(0,len(null_2),len(null_2))   
    idxs_b = np.random.randint(0,len(competitor_2),len(competitor_2))    

    local_null = [null_2[i] for i in idxs_a]
    local_competitor = [competitor_2[i] for i in idxs_b]

    local_t = ttest(local_null,local_competitor)
    bootstrap_ts.append(local_t)

bootstrap_ts = np.array(bootstrap_ts)
pos_idx = np.where(bootstrap_ts>t_init)[0]

p = len(pos_idx)/num_it
print(p)

# plt.hist(cohens_d)
# plt.show()

