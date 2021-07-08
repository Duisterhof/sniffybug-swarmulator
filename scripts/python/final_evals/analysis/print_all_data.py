import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

# baseline = np.loadtxt('base_new_avg_dist.txt')
baseline_pso = np.loadtxt('../manual_dist_avg.txt')
baseline_gradient = np.loadtxt('../gradient_seeker_avg_distance_live_gradient.txt')
baseline_wind = np.loadtxt('../wind_seeker_avg_distance.txt')
evolved_pso = np.loadtxt('../evolved_no_doping_dist_avg.txt')
evolved_pso_doping = np.loadtxt('../evolved_doping_dist_avg.txt')

print("means, times: pso, wind, gradient, evolved pso, evolved pso doping")
print(np.mean(baseline_pso))
print(np.mean(baseline_wind))
print(np.mean(baseline_gradient))
print(np.mean(evolved_pso))
print(np.mean(evolved_pso_doping))

# baseline = np.loadtxt('base_new_avg_dist.txt')
baseline_pso = np.loadtxt('../manual_time.txt')
baseline_gradient = np.loadtxt('../gradient_seeker_time_live_gradient.txt')
baseline_wind = np.loadtxt('../wind_seeker_time.txt')
evolved_pso = np.loadtxt('../evolved_no_doping_time.txt')
evolved_pso_doping = np.loadtxt('../evolved_doping_time.txt')

print("means, times: pso, gradient, wind, evolved pso, evolved pso doping")
print(np.mean(baseline_pso)/3.)
print(np.mean(baseline_wind)/3.)
print(np.mean(baseline_gradient)/3.)
print(np.mean(evolved_pso)/3.)
print(np.mean(evolved_pso_doping)/3.)

print("success rates:")

print(1-len(np.where(np.array(baseline_pso==300))[0])/(len(baseline_pso)))
print(1-len(np.where(np.array(baseline_wind==300))[0])/(len(baseline_wind)))
print(1-len(np.where(np.array(baseline_gradient==300))[0])/(len(baseline_gradient)))
print(1-len(np.where(np.array(evolved_pso==300))[0])/(len(evolved_pso)))
print(1-len(np.where(np.array(evolved_pso_doping==300))[0])/(len(evolved_pso_doping)))

print("bootstrap")

