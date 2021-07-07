import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

baseline = np.loadtxt('../manual_time.txt')
no_doping = np.loadtxt('../evolved_no_doping_time.txt')
doping = np.loadtxt('../evolved_doping_time.txt')

print(np.mean(baseline))
print(np.mean(no_doping))
print(np.mean(doping))

baseline_fails = len(np.where(np.array(baseline==300))[0])/(len(baseline))
no_doping_fails = len(np.where(np.array(no_doping==300))[0])/(len(no_doping))	
doping_fails = len(np.where(np.array(doping==300))[0])/(len(doping))

print(baseline_fails)
print(no_doping_fails)
print(doping_fails)
sns.set_style(style="whitegrid")

fig,ax = plt.subplots(figsize=(5,2.5))
labelsize = 22
ax.tick_params(axis="x", labelsize=labelsize)
ax.tick_params(axis="y", labelsize=labelsize)

data = [baseline/3,no_doping/3,doping/3]
sns.violinplot(data=data,cut=0,palette="Paired")
plt.xticks([0,1,2],['Manual','Evolved','Doping'])
ax.set_ylabel('Avg time \n to source [s]',fontsize=labelsize)
plt.savefig('avg_time_eval.pdf',bbox_inches='tight')
plt.show()
