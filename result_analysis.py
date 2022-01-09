import os
import numpy as np

# data_name = 'crowds_zara01'
# data_name = 'crowds_zara02'
data_name = 'crowds_zara03'

# map_name = 'blank'
# map_name = 'A'
map_name = 'B'
# map_name = 'C'
having_pedestrian = True
# having_pedestrian = False
root = os.getcwd()
results_path = os.path.join(root, 'results/')
if having_pedestrian:
    # print('Map name: ', map_name, ' Data name: ', data_name, ' Dynamic')
    print('#### Map' + map_name + ', \' _' + data_name + '_\'')
    results_path = os.path.join(results_path, 'dynamic')
else:
    # print('Map name: ', map_name, 'Static')
    print('#### Map' + map_name)
    results_path = os.path.join(results_path, 'static')
# print(results_path)

bi_time_list = np.load(os.path.join(results_path, 'bi_time_list_' + map_name + '_' + data_name + '.npy'))
bi_cost_list = np.load(os.path.join(results_path, 'bi_cost_list_' + map_name + '_' + data_name + '.npy'))
time_list = np.load(os.path.join(results_path, 'time_list_' + map_name + '_' + data_name + '.npy'))
cost_list = np.load(os.path.join(results_path, 'cost_list_' + map_name + '_' + data_name + '.npy'))
bi_nav_list = np.load(os.path.join(results_path, 'bi_nav_list_' + map_name + '_' + data_name + '.npy'))
nav_list = np.load(os.path.join(results_path, 'nav_list_' + map_name + '_' + data_name + '.npy'))

print('| Items | Means | Standard Error|')
print('| ----  | ----  | ----  |')
print('|Bidirectional time| ', round(bi_time_list.mean(), 2), '| ', round(bi_time_list.std(), 2), '|')
print('|Bidirectional cost| ', round(bi_cost_list.mean(), 2), '| ', round(bi_cost_list.std(), 2), '|')
print('|Bidirectional Navigation time| ', round(bi_nav_list.mean(), 2), '| ', round(bi_nav_list.std(), 2), '|')
print('|Unidirectional time| ', round(time_list.mean(), 2), '| ', round(time_list.std(), 2), '|')
print('|Unidirectional cost| ', round(cost_list.mean(), 2), '| ', round(cost_list.std(), 2), '|')
print('|Navigation time| ', round(nav_list.mean(), 2), '| ', round(nav_list.std(), 2), '|')

