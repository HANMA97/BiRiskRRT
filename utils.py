import numpy as np
import cv2
import math
from scipy.interpolate import interp1d
import os
import glob


class Custom_pose:
    def __init__(self):
        # in meters
        self.x = None
        # in meters
        self.y = None
        # in radians (-PI, PI)
        self.theta = None

    def __repr__(self):
        return 'x: ' + str(self.x) + ', y: ' + str(self.y) + ', theta: ' + str(self.theta)


class Linear:
    def __init__(self, x_vel, y_vel):
        # consider only the linear velocity in x direction
        self.x_vel = x_vel
        self.y_vel = y_vel


class Twist:
    def __init__(self, linear, angular):
        self.linear = Linear(linear.x_vel, linear.y_vel)
        # consider only the angular velocity in z direction
        self.angular = angular


class Control:
    def __init__(self, twist):
        self.twist = Twist(twist.linear, twist.angular)
        # whether the control has already been tested or not (doesn't matter if the resulting node was in collision
        # or was effectively added to the tree)
        self.open = True


class Node:
    def __init__(self):
        # time at which node is valid
        self.time = None
        # pose of the node
        self.pose = None
        # twist
        self.vel = None
        # List of Control
        self.possible_controls = []
        # node risk (=/= node weight, we don't consider distance to goal)
        self.risk = 0.0
        # depth of node in the tree
        self.depth = 0
        # false if the node is in collision (the risk is higher than the threshold set by user)
        self.isFree = True
        # true if at least one control among all possible controls remain open
        self.isOpen = True
        # in forward tree is the cost to start, otherwise, the cost to end
        self.cost = 0.0
        self.parent = None
        self.sons = []


class OccupancyGrid:
    def __init__(self, image_map, resolution):
        self.map = image_map.copy()
        self.static_map = self.map.copy()
        self.resolution = resolution
        self.origin_offset_x = self.map.shape[1] * self.resolution / 2
        self.origin_offset_y = -self.map.shape[0] * self.resolution / 2
        thresh = 127

        # to gray
        im_gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        self.height = self.map.shape[0]
        self.width = self.map.shape[1]
        self.grid = cv2.threshold(im_gray, thresh, 255, cv2.THRESH_BINARY)[1]
        self.grid = np.ones(self.grid.shape) - self.grid / 255
        # flatten
        self.grid = self.grid.flatten() * 100
        self.static_grid = self.grid.copy()

    def gridIndexFromCoord(self, i, j):
        return i + self.width * j

    def gridIFromIndex(self, index):
        return index % self.width

    def gridJFromIndex(self, index):
        return math.floor(index / self.width)

    def gridIFromPose(self, pose):
        return abs(int(round((pose.x + self.origin_offset_x) / self.resolution)))

    def gridJFromPose(self, pose):
        return abs(int(round((pose.y + self.origin_offset_y) / self.resolution)))

    def poseFromGridCoord(self, i, j):
        pose = Custom_pose()
        pose.x = self.resolution * i - self.origin_offset_x
        pose.y = -self.resolution * j - self.origin_offset_y
        pose.theta = 0.0
        return pose

    def resetGrid(self):
        """
        This function is for environment with dynamic obstacles
        :return:
        """
        self.grid = self.static_grid.copy()


'''************************the code below is the trajectory reader***********************'''


class TrajReader:
    """Read trajectories, interpolate"""

    def __init__(self, data_set_name, delta_t=0.4):
        self.dataset_name = data_set_name
        print(self.dataset_name)
        self.delta_t = delta_t

        # get trajectory folder
        self.traj_path = os.path.join(os.getcwd(), 'data/{}/'.format(self.dataset_name))
        # print('get trajectory path')

        # get dataset from the txt files
        self.traj_files = glob.glob(os.path.join(self.traj_path, "*.txt"))
        self.traj_files = np.sort(self.traj_files)
        self.trajs = dict()
        self._build_traj()  # read and process traj
        # print('build trajectory')
        self._interpolate_traj()
        # print('interpolate a smooth one')
        self.scale = Scale(self.dataset_name)
        # print('updated dateset')

        # print(self.traj_path)
        # print('Dataset Initialized ...')

        # define time duration
        self.start_t = 0
        self.end_t = self.start_t + 35

    def update_ogmap(self, ogmap, time):
        """
        update Occupancy Grid map according to current pedestrians positions
        :param ogmap: OccupancyGrid (defined in utils.py)
        :param time:
        :return:
        """
        # constrain current time in [self.start_t, self.end_t]
        curr_time = time % self.end_t
        trajs = self.get_traj(curr_time)
        # reset the occupancy grid map to static obstacle maps
        ogmap.resetGrid()
        for traj in trajs:
            temp_pose = Custom_pose()
            temp_pose.x = traj[1] + self.scale.offset_x
            temp_pose.y = traj[2] + self.scale.offset_y
            temp_pose.theta = traj[3]
            ped_grid_i = ogmap.gridIFromPose(temp_pose)
            ped_grid_j = ogmap.gridJFromPose(temp_pose)

            min_i = max(ped_grid_i - 4, 0)
            max_i = min(ped_grid_i + 4, int(ogmap.width))
            min_j = max(ped_grid_j - 4, 0)
            max_j = min(ped_grid_j + 4, int(ogmap.height))

            for i in range(min_i, max_i + 1):
                for j in range(min_j, max_j + 1):
                    ogmap.grid[ogmap.gridIndexFromCoord(i, j)] = 100

    def get_traj(self, time):
        """given time, return trajs"""
        trajs = []
        for id in self.trajs:
            data = self.trajs[id].interpolate(time)
            if data:
                trajs.append(data)
        return trajs

    def _build_traj(self):
        """Read data and process into"""
        for i, file in enumerate(self.traj_files):
            f = open(file, "r")
            timestep = i * self.delta_t
            for line in f:
                id, x, y, theta, w, h, uncertainty = [float(n) for n in line.split()]
                id = int(id)
                if id not in self.trajs:
                    self.trajs[id] = TrajEntry(self.delta_t)
                self.trajs[id].add_data(timestep, x, y, theta, id)

    def _interpolate_traj(self):
        """interpolate trajs"""
        for id in self.trajs:
            self.trajs[id].fit_data()


class TrajEntry:
    """Trajcetory entry"""

    def __init__(self, delta_t):
        self.delta_t = delta_t
        self.id = []  # add id
        self.timestep = []
        self.x = []
        self.y = []
        self.theta = []
        self.x_vel = []
        self.y_vel = []

    def add_data(self, timestep, x, y, theta, id):
        if len(self.x) == 0:
            self.x_vel.append(0)
            self.y_vel.append(0)
        else:
            delta_t = timestep - self.timestep[-1]
            self.x_vel.append((x - self.x[-1]) / delta_t)
            self.y_vel.append((y - self.y[-1]) / delta_t)
        self.timestep.append(timestep)
        self.x.append(x)
        self.y.append(y)
        self.theta.append(theta)
        self.id.append(id)

    def fit_data(self):
        """fit by cubic spline interpolation: """
        if len(self.x) == 1:
            return
        method = "linear" if len(self.x) <= 3 else "cubic"
        self.x_func = interp1d(self.timestep, self.x, kind=method)
        self.y_func = interp1d(self.timestep, self.y, kind=method)
        self.theta_func = interp1d(self.timestep, self.theta, kind=method)
        self.x_vel[0] = self.x_vel[1]
        self.y_vel[0] = self.y_vel[1]
        self.x_vel_func = interp1d(self.timestep, self.x_vel, kind=method)
        self.y_vel_func = interp1d(self.timestep, self.y_vel, kind=method)
        self.id_func = interp1d(self.timestep, self.id, kind="linear")

    def interpolate(self, time):
        if time < min(self.timestep) or time > max(self.timestep) or not hasattr(self, "x_func"):
            return None
        x = self.x_func(time)
        y = self.y_func(time)
        theta = self.theta_func(time)
        x_vel = self.x_vel_func(time)
        y_vel = self.y_vel_func(time)
        id = int(self.id_func(time))
        return [id, x, y, theta, x_vel, y_vel]

    def __str__(self):
        return "timestep: {},\nx: {},\ny: {},\ntheta: {},\n" \
               "x vel: {},\ny vel: {}".format(self.timestep, self.x, self.y, self.theta, self.x_vel, self.y_vel)


class Scale:
    def __init__(self, name):
        self.dataset = name
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.bound_list = []
        self.set_param()

    def set_param(self):
        if self.dataset == 'biwi_eth':
            self.offset_x = -5.0
            self.offset_y = -5.0
            self.bound_list.append([+8.0, +3.0])
            self.bound_list.append([+8.0, -3.0])
            self.bound_list.append([-7.0, -3.0])
            self.bound_list.append([-7.0, +3.0])
        elif self.dataset == 'biwi_hotel':
            self.offset_x = 0.0
            self.offset_y = 0.0
            self.bound_list.append([+4.0, +5.0])
            self.bound_list.append([+4.0, -8.0])
            self.bound_list.append([-2.0, -8.0])
            self.bound_list.append([-2.0, +5.0])
        elif self.dataset == 'crowds_zara01' or \
                self.dataset == 'crowds_zara02' or \
                self.dataset == 'crowds_zara03':
            self.offset_x = -7.0
            self.offset_y = -5.0
            self.bound_list.append([+8.0, +3.0])
            self.bound_list.append([+8.0, -3.0])
            self.bound_list.append([-7.0, -3.0])
            self.bound_list.append([-7.0, +3.0])

        elif self.dataset == 'shatin_plaza' or \
                self.dataset == 'shatin_station':
            self.offset_x = 0.0
            self.offset_y = 0.0
            self.bound_list.append([+10.0, +10.0])
            self.bound_list.append([+10.0, -10.0])
            self.bound_list.append([-10.0, -10.0])
            self.bound_list.append([-10.0, +10.0])
        elif self.dataset == 'shatin_cross':
            self.offset_x = -5
            self.offset_y = 0.0
            self.bound_list.append([+5.0, +10.0])
            self.bound_list.append([+5.0, -10.0])
            self.bound_list.append([-6.0, -10.0])
            self.bound_list.append([-6.0, +10.0])
        elif self.dataset == 'students001' or \
                self.dataset == 'students003':
            self.offset_x = -7.0
            self.offset_y = -7.0
            self.bound_list.append([+9.0, +9.0])
            self.bound_list.append([+9.0, -9.0])
            self.bound_list.append([-9.0, -9.0])
            self.bound_list.append([-9.0, +9.0])
        elif self.dataset == 'uni_examples':
            self.offset_x = -7.0
            self.offset_y = -9.0
            self.bound_list.append([+8.0, +4.0])
            self.bound_list.append([+8.0, -5.0])
            self.bound_list.append([-7.0, -5.0])
            self.bound_list.append([-7.0, +4.0])
        else:
            self.bound_list = [[0.0, 0.0]] * 4


def generateSimulationVideo(traj, ogmap, save_path, trajReader=None, mode='static'):
    """
    Function to generate simulation video from frames
    :param traj: Trajectory provided by the planner (Node list)
    :param ogmap: OccupancyGrid
    :param trajReader: trajReader for pedestrian positions
    :param mode: 'static' / 'dynamic'
    :return:
    """
    frames = []
    for node in traj:
        curr_frame = ogmap.static_map.copy()
        pose = node.pose
        corner1 = np.array([0.75, 0.35])
        corner2 = np.array([0.75, - 0.35])
        corner3 = np.array([- 0.75, - 0.35])
        corner4 = np.array([- 0.75, 0.35])
        rotate_matrix = np.array([[math.cos(pose.theta), -math.sin(pose.theta)],
                                  [math.sin(pose.theta), math.cos(pose.theta)]])
        corner1 = rotate_matrix.dot(corner1)
        corner2 = rotate_matrix.dot(corner2)
        corner3 = rotate_matrix.dot(corner3)
        corner4 = rotate_matrix.dot(corner4)
        pose1 = Custom_pose()
        pose1.x = corner1[0] + pose.x
        pose1.y = corner1[1] + pose.y
        pose2 = Custom_pose()
        pose2.x = corner2[0] + pose.x
        pose2.y = corner2[1] + pose.y
        pose3 = Custom_pose()
        pose3.x = corner3[0] + pose.x
        pose3.y = corner3[1] + pose.y
        pose4 = Custom_pose()
        pose4.x = corner4[0] + pose.x
        pose4.y = corner4[1] + pose.y
        coord1 = [ogmap.gridIFromPose(pose1), ogmap.gridJFromPose(pose1)]
        coord2 = [ogmap.gridIFromPose(pose2), ogmap.gridJFromPose(pose2)]
        coord3 = [ogmap.gridIFromPose(pose3), ogmap.gridJFromPose(pose3)]
        coord4 = [ogmap.gridIFromPose(pose4), ogmap.gridJFromPose(pose4)]
        contours = np.array([coord1, coord2, coord3, coord4])
        cv2.fillPoly(curr_frame, pts=[contours], color=(255, 0, 0))
        if mode == 'dynamic':
            ped_poses = trajReader.get_traj(node.time)
            for ped_pose in ped_poses:
                temp_pose = Custom_pose()
                temp_pose.x = ped_pose[1] + trajReader.scale.offset_x
                temp_pose.y = ped_pose[2] + trajReader.scale.offset_y
                ped_grid_i = ogmap.gridIFromPose(temp_pose)
                ped_grid_j = ogmap.gridJFromPose(temp_pose)
                cv2.circle(curr_frame, (ped_grid_i, ped_grid_j), radius=8, color=[255, 255, 0], thickness=-1)

        frames.append(curr_frame)

    out = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'MP4V'), 8, (ogmap.width, ogmap.height))
    for i in range(len(frames)):
        out.write(frames[i])
    out.release()


'''test for DccupancyGrid (comment the lines below when using)'''
# if __name__ == '__main__':
#     image = cv2.imread('/home/mh/catkin_ws/src/riskrrt_ros/riskrrt/worlds/mixDirection/mixDirection.png')
#     cv2.imshow('original image', image)
#     grid = OccupancyGrid(image, 0.054)
#
#     cv2.imshow('grid', grid.grid.reshape(image.shape[0:2]))
#     cv2.waitKey(0)
#     print(grid.grid.shape)
#     print(np.unique(grid.grid))
#     print(grid.poseFromGridCoord(200, 100))
#     print(200*0.054 - 700*0.054/2, -100*0.054 + 490*0.054/2)
'''test for TrajectoryReader (comment the lines below when using)'''
# if __name__ == '__main__':
#     root_path = os.getcwd()
#     map_path = os.path.join(root_path, 'maps/map.png')
#     map = cv2.imread(map_path)
#     grid = OccupancyGrid(map, 0.054)
#     delta_t = 0.1
#     trajectories = TrajReader('crowds_zara01', delta_t)
#     time = 0.0
#     while True:
#         trajectories.update_ogmap(grid, time)
#         time += delta_t
#         if time > 35:
#             time = 0.0
#         cv2.imshow('grid map', grid.grid.reshape(map.shape[0:2]))
#         cv2.waitKey()
