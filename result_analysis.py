import os
import numpy as np

having_pedestrian = False
root = os.getcwd()
results_path = os.path.join(root, 'results/')
if having_pedestrian:
    results_path = os.path.join(results_path, 'dynamic')
else:
    results_path = os.path.join(results_path, 'static')
print(results_path)
bi_time_list = np.load(os.path.join(results_path, 'Cbi_time_list.npy'))
bi_cost_list = np.load(os.path.join(results_path, 'Cbi_cost_list.npy'))
# time_list = np.load(os.path.join(results_path, 'time_list.npy'))
# cost_list = np.load(os.path.join(results_path, 'cost_list.npy'))
bi_nav_list = np.load(os.path.join(results_path, 'C2bi_nav_list.npy'))
# nav_list = np.load(os.path.join(results_path, 'nav_list.npy'))
print('bidirectional time: ', round(bi_time_list.mean(), 2), ', ', round(bi_time_list.std(), 2))
print('bidirectional cost: ', round(bi_cost_list.mean(), 2), ', ', round(bi_cost_list.std(), 2))
print('Bidirectional Navigation time: ', round(bi_nav_list.mean(), 2), ', ', round(bi_nav_list.std(), 2))
# print('unidirectional time: ', round(time_list.mean(), 2), ', ', round(time_list.std(), 2))
# print('unidirectional cost: ', round(cost_list.mean(), 2), ', ', round(cost_list.std(), 2))
# print('Navigation time: ', round(nav_list.mean(), 2), ', ', round(nav_list.std(), 2))

