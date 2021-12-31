import os
import numpy as np

having_pedestrian = True
root = os.getcwd()
results_path = os.path.join(root, 'results/')
if having_pedestrian:
    results_path = os.path.join(results_path, 'dynamic')
else:
    results_path = os.path.join(results_path, 'static')
print(results_path)
bi_time_list = np.load(os.path.join(results_path, 'bi_time_list.npy'))
bi_cost_list = np.load(os.path.join(results_path, 'bi_cost_list.npy'))
time_list = np.load(os.path.join(results_path, 'time_list.npy'))
cost_list = np.load(os.path.join(results_path, 'cost_list.npy'))
bi_nav_list = np.load(os.path.join(results_path, 'bi_nav_list.npy'))
nav_list = np.load(os.path.join(results_path, 'nav_list.npy'))
print('bidirectional time: ', bi_time_list.mean(), ', ', bi_time_list.std())
print('bidirectional cost: ', bi_cost_list.mean(), ', ', bi_cost_list.std())
print('Bidirectional Navigation time: ', bi_nav_list.mean(), ', ', bi_nav_list.std())
print('unidirectional time: ', time_list.mean(), ', ', time_list.std())
print('unidirectional cost: ', cost_list.mean(), ', ', cost_list.std())
print('Navigation time: ', nav_list.mean(), ', ', nav_list.std())

