from biriskrrt import BiRiskRRT, BiRiskRRTPed
from param import BiParams
from utils import OccupancyGrid, generateSimulationVideo
import cv2
import time
import numpy as np
import os

data_name = 'crowds_zara03'
root_path = os.getcwd()
experiment_times = 100
bidirectional = True
# bidirectional = False
having_pedestrian = True
# having_pedestrian = False
map_resolution = 0.054 # meter/pixel
map_name = 'B' # prepare several maps with different names

results_path = os.path.join(root_path, 'results')
video_path = os.path.join(root_path, 'results', 'video')
visualization_path = os.path.join(results_path, 'visualizations')
map_path = os.path.join(root_path, 'maps', map_name + '.png')
if having_pedestrian:
    print('having pedestrians')
    results_path = os.path.join(results_path, 'dynamic')
else:
    results_path = os.path.join(results_path, 'static')

map = cv2.imread(map_path)
bi_time_list = []
bi_cost_list = []
bi_nav_time = []
time_list = []
cost_list = []
nav_time = []

for i in range(experiment_times):
    print(str(i + 1) + '/' + str(experiment_times))
    print(map_name)
    ogmap = OccupancyGrid(map, map_resolution)
    param = BiParams()
    if having_pedestrian:
        planner = BiRiskRRTPed(param, ogmap, data_name)
    else:
        planner = BiRiskRRT(param, ogmap)

    start = time.time()
    while True:
        if bidirectional:
            planner.biGrow()
        else:
            planner.grow()
        # cv2.imwrite('/home/mh/Desktop/bitree.png', planner.ogmap.map)
        if planner.terminate:
            print('goal reached!')
            break
    end = time.time()

    traj = planner.findTraj()
    if bidirectional:
        bi_time_list.append(end - start)
        bi_cost_list.append(traj[-1].cost)
        bi_nav_time.append(planner.goal_node.time)
    else:
        time_list.append(end - start)
        cost_list.append(traj[-1].cost)
        nav_time.append(planner.goal_node.time)

    print('Planning time: ', end - start)
    print('Trajectory cost: ', traj[-1].cost)
    print('Trajectory navigation time: ', planner.goal_node.time)
    if having_pedestrian:
        gif_name = 'bi_navi_dynamic_' + map_name + '_' + data_name + str(i) + '.gif' if bidirectional \
            else 'navi_dynamic_' + map_name + '_' + data_name + str(i) + '.gif'
        generateSimulationVideo(traj, planner.ogmap,
                                os.path.join(video_path, gif_name),
                                trajReader=planner.trajectorReader,
                                mode='dynamic')
    else:
        gif_name = 'bi_navi_static_' + map_name + '_' + data_name + str(i) + '.gif' if bidirectional \
            else 'navi_static_'+ map_name + '_' + data_name + str(i) + '.gif'
        generateSimulationVideo(traj, planner.ogmap,
                                os.path.join(video_path, gif_name),
                                trajReader=None,
                                mode='static')
    for node in traj:
        center_coordinates = [planner.ogmap.gridIFromPose(node.pose), planner.ogmap.gridJFromPose(node.pose)]
        planner.ogmap.map = cv2.circle(planner.ogmap.map, center_coordinates, 5, [0, 0, 255], -1)
        # print(node.pose)

    if bidirectional:
        traj = planner.heur_path
        print('bidirectional')
        for node in traj:
            center_coordinates = [planner.ogmap.gridIFromPose(node.pose), planner.ogmap.gridJFromPose(node.pose)]
            planner.ogmap.map = cv2.circle(planner.ogmap.map, center_coordinates, 5, [255, 0, 255], -1)
        # print(node.pose)
    if bidirectional:
        cv2.imwrite(visualization_path + '/bitree_' + map_name + '_' + data_name + '_' + str(i) + '.png', planner.ogmap.map)
    else:
        cv2.imwrite(visualization_path + '/tree_' + map_name + '_' + data_name + '_' + str(i) + '.png', planner.ogmap.map)

if bidirectional:
    np.save(os.path.join(results_path, 'bi_time_list_' + map_name + '_' + data_name + '.npy'), bi_time_list)
    np.save(os.path.join(results_path, 'bi_cost_list_' + map_name + '_' + data_name + '.npy'), bi_cost_list)
    np.save(os.path.join(results_path, 'bi_nav_list_' + map_name + '_' + data_name + '.npy'), bi_nav_time)
else:
    np.save(os.path.join(results_path, 'time_list_' + map_name + '_' + data_name + '.npy'), time_list)
    np.save(os.path.join(results_path, 'cost_list_' + map_name + '_' + data_name + '.npy'), cost_list)
    np.save(os.path.join(results_path, 'nav_list_' + map_name + '_' + data_name + '.npy'), nav_time)

