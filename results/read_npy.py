import numpy as np

time = np.load("bi_time_list.npy")
cost = np.load("bi_cost_list.npy")
nav_time = np.load("bi_nav_list.npy")

print(time)
print(cost)
print(nav_time)