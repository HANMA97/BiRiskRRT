from utils import *
import math
import random
import cv2

# Radius of circle
radius = 5

# Blue color in BGR
color = (255, 0, 0)

# Line thickness of 2 px
thickness = 2


class RiskRRT:
    def __init__(self, param, ogmap):
        # hyper parameters
        self.timeStep = param.timeStep
        self.fwdMaxDepth = param.fwdMaxDepth
        self.nv = param.nv
        self.nphi = param.nphi
        self.threshold = param.threshold
        self.rotationWeight = param.rotationWeight
        self.socialWeight = param.socialWeight
        self.growTime = param.growTime
        self.bias = param.bias
        self.goalTh = param.goalTh
        self.windowSize = param.windowSize
        self.robotLength = param.robotLength
        self.robotWidth = param.robotWidth
        self.vMin = param.vMin
        self.vMax = param.vMax
        self.accMax = param.accMax
        self.omegaMax = param.omegaMax
        self.accOmegaMax = param.accOmegaMax

        # other parameters
        self.start_state = param.start
        self.goal_state = param.goal
        self.root = Node()
        # OccupancyGrid
        self.ogmap = ogmap

        # random tree (i.e., a list of node)
        self.candidate_nodes = []

        self.initRoot()

        # if goal reached terminate = True
        self.terminate = False
        # the node nearest to the goal state is used to back track the solution
        self.goal_node = None

    def initRoot(self):
        self.root.time = 0.0
        self.root.pose = Custom_pose()
        self.root.pose.x = self.start_state[0]
        self.root.pose.y = self.start_state[1]
        self.root.pose.theta = self.start_state[2]
        self.root.vel = Twist(Linear(0.0, 0.0), 0.0)
        self.root.parent = None
        self.root.sons = []
        self.root.possible_controls = self.discretizeVelocities(self.root)
        self.root.isOpen = True
        self.root.depth = 0
        self.root.cost = 0.0
        self.root.risk = self.computeNodeRisk(self.root)
        self.root.isFree = (self.root.risk <= self.threshold)
        self.candidate_nodes.append(self.root)

    def discretizeVelocities(self, node):
        """

        :param node: Node
        :return:
        """
        controls = []

        min_linear_vel = node.vel.linear.x_vel - self.accMax * self.timeStep
        max_linear_vel = node.vel.linear.x_vel + self.accMax * self.timeStep
        min_angular_vel = node.vel.angular - self.accOmegaMax * self.timeStep
        max_angular_vel = node.vel.angular + self.accOmegaMax * self.timeStep

        delta_linear = (max_linear_vel - min_linear_vel) / self.nv
        delta_angular = (max_angular_vel - min_angular_vel) / self.nphi

        for i in range(self.nv):
            for j in range(self.nphi):
                twist = Twist(Linear(0.0, 0.0), 0.0)
                curr_control = Control(twist)
                curr_control.twist.linear.x_vel = min_linear_vel + i * delta_linear
                curr_control.twist.angular = min_angular_vel + j * delta_angular
                curr_control.open = True
                controls.append(curr_control)

        return controls

    def computeNodeRisk(self, node):
        """

        :param node: Node
        :return:
        """
        front_left = Custom_pose()
        front_right = Custom_pose()
        rear_left = Custom_pose()
        rear_right = Custom_pose()
        grid_cells = []
        # the wheel axis is assumed to be in the middle
        l = self.robotLength / 2
        w = self.robotWidth / 2

        # computing the poses of each corner of the robot's footprint
        front_left.x = node.pose.x + (l * math.cos(node.pose.theta) + w * math.cos(node.pose.theta + math.pi / 2.0))
        front_left.y = node.pose.y + (l * math.sin(node.pose.theta) + w * math.sin(node.pose.theta + math.pi / 2.0))
        front_right.x = node.pose.x + (l * math.cos(node.pose.theta) + w * math.cos(node.pose.theta - math.pi / 2.0))
        front_right.y = node.pose.y + (l * math.sin(node.pose.theta) + w * math.sin(node.pose.theta - math.pi / 2.0))
        rear_left.x = node.pose.x + (l * math.cos(node.pose.theta + math.pi) +
                                     w * math.cos(node.pose.theta + math.pi - math.pi / 2.0))
        rear_left.y = node.pose.y + (l * math.sin(node.pose.theta + math.pi) +
                                     w * math.sin(node.pose.theta + math.pi - math.pi / 2.0))
        rear_right.x = node.pose.x + (l * math.cos(node.pose.theta + math.pi) +
                                      w * math.cos(node.pose.theta + math.pi + math.pi / 2.0))
        rear_right.y = node.pose.y + (l * math.sin(node.pose.theta + math.pi) +
                                      w * math.sin(node.pose.theta + math.pi + math.pi / 2.0))

        grid_front_left_x = self.ogmap.gridIFromPose(front_left)
        grid_front_left_y = self.ogmap.gridJFromPose(front_left)
        grid_front_right_x = self.ogmap.gridIFromPose(front_right)
        grid_front_right_y = self.ogmap.gridJFromPose(front_right)
        grid_rear_left_x = self.ogmap.gridIFromPose(rear_left)
        grid_rear_left_y = self.ogmap.gridJFromPose(rear_left)
        grid_rear_right_x = self.ogmap.gridIFromPose(rear_right)
        grid_rear_right_y = self.ogmap.gridJFromPose(rear_right)

        # bounding box of the robot
        grid_max_x = max(grid_front_left_x, max(grid_front_right_x, max(grid_rear_left_x, grid_rear_right_x)))
        grid_max_y = max(grid_front_left_y, max(grid_front_right_y, max(grid_rear_left_y, grid_rear_right_y)))
        grid_min_x = min(grid_front_left_x, min(grid_front_right_x, min(grid_rear_left_x, grid_rear_right_x)))
        grid_min_y = min(grid_front_left_y, min(grid_front_right_y, min(grid_rear_left_y, grid_rear_right_y)))

        # creating a list of all the grid cells within the robot's footprint
        for i in range(grid_min_x, grid_max_x):
            for j in range(grid_min_y, grid_max_y):
                if 0 <= i < self.ogmap.width and 0 <= j < self.ogmap.height:
                    grid_cells.append(self.ogmap.gridIndexFromCoord(i, j))

        # going through all the cells in robot footprint and getting the maximum risk
        max_risk = 0.0
        for i in range(len(grid_cells)):
            risk = self.ogmap.grid[grid_cells[i]]
            max_risk = max(max_risk, risk)

        # risk propagation from a node to his sons if their risk is lower
        if node.parent is not None:
            max_risk = max(max_risk, node.parent.risk)

        return max_risk

    def chooseBestNode(self, goal):
        """
        choose the best node in the forward tree
        :param goal: Custom pose
        :return:
        """
        best_rated_node = None
        best_weight = 0.0

        for i in range(len(self.candidate_nodes)):
            current_node = self.candidate_nodes[i]
            current_weight = self.computeNodeWeight(current_node, goal)
            if best_weight < current_weight and current_node.isOpen:
                best_weight = current_weight
                best_rated_node = current_node

        return best_rated_node

    def goalReached(self, node):
        if math.sqrt(math.pow(node.pose.x - self.goal_state[0], 2)
                     + math.pow(node.pose.y - self.goal_state[1], 2)) < self.goalTh:
            return True
        return False

    def computeNodeWeight(self, node, goal):
        """

        :param node: Node
        :param goal: Custom_pose
        :return:
        """
        weight = 1.0 / (self.socialWeight * node.risk + self.trajLength(node.pose, goal)) * 1e-9
        return weight

    def extend(self, node, rand_goal):
        new_node = Node()
        best_control_index = -1

        best_control_score = 0.0

        for i in range(len(node.possible_controls)):
            if node.possible_controls[i].open:
                expected_pose = self.robotKinematic(node.pose, node.possible_controls[i])
                control_score = self.computeControlScore(expected_pose, rand_goal)
                if control_score >= best_control_score:
                    best_control_score = control_score
                    best_control_index = i

        if best_control_index == -1:
            return None

        new_node.time = node.time + self.timeStep
        # update ogmap with dynamic obstacles
        self.getNewObsPrediction(new_node.time)
        new_node.pose = self.robotKinematic(node.pose, node.possible_controls[best_control_index])
        new_node.vel = node.possible_controls[best_control_index].twist
        new_node.parent = node
        new_node.sons = []
        new_node.possible_controls = self.discretizeVelocities(new_node)
        new_node.depth = node.depth + 1
        new_node.risk = self.computeNodeRisk(new_node)
        new_node.isFree = (new_node.risk <= self.threshold)
        new_node.isOpen = True

        new_node.cost = self.euclideanDistance(new_node, node) + node.cost
        node.sons.append(new_node)
        node.possible_controls[best_control_index].open = False

        node_still_open = False
        for possible_control in node.possible_controls:
            node_still_open = node_still_open or possible_control.open
        node.isOpen = node_still_open

        if new_node.isFree and new_node.depth < self.fwdMaxDepth:
            self.candidate_nodes.append(new_node)
            # print('new node pose: ', new_node.pose)
            '''for visualization------>begin'''
            center = [self.ogmap.gridIFromPose(new_node.pose), self.ogmap.gridJFromPose(new_node.pose)]
            self.ogmap.map = cv2.circle(self.ogmap.map, center, radius, color, thickness)
            # debug
            # cv2.imwrite('/home/mh/GithubProjects/BiRiskRRTValidation/results/' + 'tree.png', self.ogmap.map)
            '''for visualization------>end'''
            if self.goalReached(new_node):
                self.goal_node = new_node
                self.terminate = True

            return new_node
        else:
            return None

    def euclideanDistance(self, new_node, node):
        """
        compute the Euclidean Distance between new_node and node
        :param new_node: Node
        :param node: Node
        :return: Euclidean distance between new_node and node
        """
        return math.sqrt(math.pow(new_node.pose.x - node.pose.x, 2) + math.pow(new_node.pose.y - node.pose.y, 2))

    def robotKinematic(self, pose, control):
        """

        :param pose: Custom_pose
        :param control: Control
        :return: new_pose: Custom_pose
        """
        new_pose = Custom_pose()
        if control.twist.linear.x_vel == 0.0:
            delta_theta = control.twist.angular * self.timeStep
            delta_x = 0.0
            delta_y = 0.0
        elif control.twist.angular == 0.0:
            delta_theta = 0.0
            delta_x = control.twist.linear.x_vel * self.timeStep
            delta_y = 0.0
        else:
            rotation_radius = control.twist.linear.x_vel / control.twist.angular
            delta_theta = control.twist.angular * self.timeStep
            delta_x = rotation_radius * math.sin(delta_theta)
            delta_y = rotation_radius * (1.0 - math.cos(delta_theta))
        new_pose.x = pose.x + (delta_x * math.cos(pose.theta) - delta_y * math.sin(pose.theta))
        new_pose.y = pose.y + (delta_x * math.sin(pose.theta) + delta_y * math.cos(pose.theta))
        new_pose.theta = math.atan2(math.sin(pose.theta + delta_theta), math.cos(pose.theta + delta_theta))

        return new_pose

    def computeControlScore(self, expected_pose, random_goal):
        """
        compute the control score of the forward tree
        :param expected_pose: Custom pose
        :param random_goal: Custom pose
        :return: score (float)
        """
        distance_from_random_goal = self.trajLength(expected_pose, random_goal)
        score = 1.0 / distance_from_random_goal + 1e-9
        return score

    def grow(self):
        random_goal = self.chooseRandomGoal()
        best_node_to_grow = self.chooseBestNode(random_goal)
        if best_node_to_grow:
            self.extend(best_node_to_grow, random_goal)

    def trajLength(self, pose, goal):
        """
        compute the cost from pose to goal. The cost considers both the linear
        :param pose: Custom_pose
        :param goal: Custom_pose
        :return:
        """
        pose_euclide_distance = math.sqrt(math.pow(pose.x - goal.x, 2) + math.pow(pose.y - goal.y, 2))
        root_euclide_distance = math.sqrt(
            math.pow(self.root.pose.x - goal.x, 2) + math.pow(self.root.pose.y - goal.y, 2))
        position_improvement = pose_euclide_distance / root_euclide_distance
        rotation_diff = math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta

        if rotation_diff > math.pi:
            rotation_diff -= 2 * math.pi
        if rotation_diff < -math.pi:
            rotation_diff += 2 * math.pi

        distance_from_goal = position_improvement + self.rotationWeight * abs(rotation_diff)

        return distance_from_goal

    def chooseRandomGoal(self):
        if random.random() > self.bias:
            random_i = random.randint(0, self.ogmap.width)
            random_j = random.randint(0, self.ogmap.height)
            random_goal = self.ogmap.poseFromGridCoord(random_i, random_j)
        else:
            random_goal = Custom_pose()
            random_goal.x = self.goal_state[0]
            random_goal.y = self.goal_state[1]
            random_goal.theta = 0.0
        return random_goal

    def findTraj(self):
        if not self.goal_node:
            return None
        traj = [self.goal_node]
        parent = self.goal_node.parent
        while parent:
            traj.append(parent)
            parent = parent.parent
        return traj

    def getNewObsPrediction(self, time):
        """
        update the occupancy grip map according to new obstacle prediction at 'time'
        wait for overriding in BiRiskRRT_Pedestrian
        :return:
        """
        pass


