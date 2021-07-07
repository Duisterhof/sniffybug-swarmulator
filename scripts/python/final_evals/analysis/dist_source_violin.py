import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# baseline = np.loadtxt('base_new_avg_dist.txt')
baseline = np.loadtxt('../manual_dist_avg.txt')
# evolved = np.loadtxt('evo_new_avg_dist.txt')
# evolved = np.loadtxt('evo_doping.txt')
evolved = np.loadtxt('../evolved_no_doping_dist_avg.txt')
evolved_doping = np.loadtxt('../evolved_doping_dist_avg.txt')

print("means")
print(np.mean(baseline))
print(np.mean(evolved))
print(np.mean(evolved_doping))

print("medians")
print(np.median(baseline))
print(np.median(evolved))
print(np.median(evolved_doping))

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