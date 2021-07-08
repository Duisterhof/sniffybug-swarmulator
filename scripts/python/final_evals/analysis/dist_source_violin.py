import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# # baseline = np.loadtxt('base_new_avg_dist.txt')
# baseline = np.loadtxt('../manual_pso_distance_0.txt')
# # evolved = np.loadtxt('evo_new_avg_dist.txt')
# # evolved = np.loadtxt('evo_doping.txt')
# evolved = np.loadtxt('../no_dopoing_pso_distance_0.txt')
# evolved_doping = np.loadtxt('../doping_pso_distance_0.txt')
num_it = 10
# baseline = np.loadtxt('base_new_avg_dist.txt')
baseline_pso = np.array([])
baseline_gradient = np.array([])
baseline_wind = np.array([])
evolved_pso = np.array([])
evolved_pso_doping = np.array([])

for i in range(num_it):
    baseline = np.append(baseline_pso,np.loadtxt('../manual_pso_distance_'+str(i)+'.txt'))
    evolved = np.append(evolved_pso,np.loadtxt('../no_dopoing_pso_distance_'+str(i)+'.txt'))
    evolved_doping = np.append(evolved_pso_doping,np.loadtxt('../doping_pso_distance_'+str(i)+'.txt'))

# print("means")
# print(np.mean(baseline))
# print(np.mean(evolved))
# print(np.mean(evolved_doping))

# print("medians")
# print(np.median(baseline))
# print(np.median(evolved))
# print(np.median(evolved_doping))

sns.set_style(style="whitegrid")
data = [baseline,evolved,evolved_doping]

fig,ax = plt.subplots(figsize=(5,1.5))
labelsize = 10
ax.tick_params(axis="x", labelsize=labelsize)
ax.tick_params(axis="y", labelsize=labelsize)

sns.violinplot(data=data,cut=0,palette="Paired")
plt.xticks([0,1,2],['Manual','Evolved \n Without Doping','Evolved \n With Doping'])
ax.set_ylabel('Avg distance \n to source [m]',fontsize=labelsize)
plt.savefig('avg_distance_eval.pdf',bbox_inches='tight')
plt.show()