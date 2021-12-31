import random

from riskrrt import RiskRRT
from utils import *

# Radius of circle
radius = 5

# Blue color in BGR
color = (0, 255, 0)

# Line thickness of 2 px
thickness = 2


class BiRiskRRT(RiskRRT):
    def __init__(self, params, ogmap):
        super(BiRiskRRT, self).__init__(params, ogmap)
        self.stddev = params.stddev
        self.heur_prob = params.heur_prob
        self.connect_th = params.connect_th
        # maximum depth a node in reverse tree
        self.maxDepth = params.maxDepth
        # connect heuristic steps
        self.connect_heur = params.connect_heur

        # reverse tree node list
        self.candidate_nodes_rev = []
        # heuristic path node list (in the reverse tree)
        self.heur_path = []
        # initialize reverse tree root
        self.root_rev = Node()
        self.initRootRev()
        # indicate the extend order of the forward tree and the reverse tree (int), set it to 0 when larger than 1000
        self.grow_iter_times = 0

    def initRootRev(self):
        self.root_rev.time = 0.0
        self.root_rev.pose = Custom_pose()
        self.root_rev.pose.x = self.goal_state[0]
        self.root_rev.pose.y = self.goal_state[1]
        self.root_rev.pose.theta = self.goal_state[2]
        self.root_rev.vel = Twist(Linear(0.0, 0.0), 0.0)
        self.root_rev.parent = None
        self.root_rev.sons = []
        self.root_rev.possible_controls = self.discretizeVelocities(self.root_rev)
        self.root_rev.isOpen = True
        self.root_rev.depth = 0.0
        self.root_rev.cost = 0.0
        self.root_rev.risk = self.computeNodeRisk(self.root_rev)
        self.root_rev.isFree = (self.root_rev.risk <= self.threshold)
        self.candidate_nodes_rev.append(self.root_rev)

    def chooseBestNodeRev(self, goal):
        """
        choose the best node w.r.t. goal in the reverse tree.
        :param goal: Custom pose
        :return:
        """
        best_rated_node = None
        best_weight = 0.0

        for i in range(len(self.candidate_nodes_rev)):
            current_node = self.candidate_nodes_rev[i]
            current_weight = self.computeNodeWeightRev(current_node, goal)
            if best_weight < current_weight and current_node.isOpen:
                best_weight = current_weight
                best_rated_node = current_node

        return best_rated_node

    def computeNodeWeightRev(self, node, goal):
        """
        compute the node weight in the reverse tree
        :param node: Node()
        :param goal: Custom pose
        :return: weight (float)
        """
        weight = 1.0 / (self.socialWeight * node.risk + self.trajLengthRev(node.pose, goal)) * 1e-9
        return weight

    def heuristicSampling(self):
        """
        heuristic sampling along the heuristic path on the reverse tree
        call when self.heur_path is not empty
        :return:
        """
        # random_index = random.randint(0, len(self.heur_path) - 1)
        # random_center_node = self.heur_path[random_index]
        # random_center_pose = random_center_node.pose
        # random_goal = Custom_pose()
        # random_goal.x = random.gauss(random_center_pose.x, self.stddev)
        # random_goal.y = random.gauss(random_center_pose.y, self.stddev)
        # random_goal.theta = 0.0
        # return random_goal
        ### the sampling strategy is too greedy
        indicator = random.random()
        if indicator < self.bias:
            random_goal = Custom_pose()
            random_goal.x = self.goal_state[0]
            random_goal.y = self.goal_state[1]
            random_goal.theta = 0.0
        elif indicator < self.heur_prob:
            random_i = random.randint(0, self.ogmap.width)
            random_j = random.randint(0, self.ogmap.height)
            random_goal = self.ogmap.poseFromGridCoord(random_i, random_j)
        else:
            random_index = random.randint(0, len(self.heur_path) - 1)
            random_center_node = self.heur_path[random_index]
            random_center_pose = random_center_node.pose
            random_goal = Custom_pose()
            random_goal.x = random.gauss(random_center_pose.x, self.stddev)
            random_goal.y = random.gauss(random_center_pose.y, self.stddev)
            random_goal.theta = 0.0
        return random_goal

    def meet(self, new_node_fwd, new_node_rev):
        """
        check if the forward tree meet the reverse tree
        :param new_node_fwd: Node from forward tree
        :param new_node_rev: Node from reverse tree
        :return: bool, True if meet, otherwise, False
        """
        distance = self.euclideanDistance(new_node_fwd, new_node_rev)
        if distance < self.connect_th:
            return True
        return False

    def connect(self, new_node, extend, chooseBestNode, mode):
        """
        This function implements the connected heuristic in the original paper.
        :param new_node: new extended node from the other tree
        :param extend: extend function of current tree
        :param chooseBestNode: chooseBestNode function of current tree
        :param mode: string 'forward/reverse' (e.g., mode = 'forward' means new_node is from the forward tree)
        :return:
        """
        assert mode == 'forward' or mode == 'reverse', 'This mode is not exist!'
        if not new_node:
            return
        random_goal = Custom_pose()
        random_goal.x = new_node.pose.x
        random_goal.y = new_node.pose.y
        random_goal.theta = new_node.pose.theta

        for i in range(self.connect_heur): # self.connect_heur steps heuristic extend
            best_node_to_grow = chooseBestNode(random_goal)
            new_node_prim = extend(best_node_to_grow, random_goal)
            if new_node_prim and self.meet(new_node_prim, new_node):
                if mode == 'forward':
                    self.heur_path = self.getHeurTraj(new_node_prim)
                elif mode == 'reverse':
                    self.heur_path = self.getHeurTraj(new_node)
                break
            if not new_node_prim:
                break

    def chooseRandomGoalRev(self):
        """
        uniform random sampler for reverse tree
        :return:
        """
        if random.random() > self.bias:
            random_i = random.randint(0, self.ogmap.width)
            random_j = random.randint(0, self.ogmap.height)
            random_goal = self.ogmap.poseFromGridCoord(random_i, random_j)
        else:
            random_goal = Custom_pose()
            random_goal.x = self.start_state[0]
            random_goal.y = self.start_state[1]
            random_goal.theta = 0.0
        return random_goal

    def biGrow(self):
        """
        Bidirectional growth with connected heuristic
        :return:
        """
        # random sampling
        if self.heur_path: # if heur_path is not empty, perform heuristic sampling. Extend only fwd tree.
            # indicator = random.random()
            # if indicator > self.heur_prob:
            #     random_goal = self.heuristicSampling()
            # else:
            #     random_goal = self.chooseRandomGoal()
            random_goal = self.heuristicSampling()
            best_node_fwd = self.chooseBestNode(random_goal)
            if best_node_fwd:
                new_node_fwd = self.extend(best_node_fwd, random_goal)
                # if new_node_fwd and self.euclideanDistance(new_node_fwd, self.root_rev) < self.heur_path[-1].cost:
                #     # print('pop()')
                #     self.heur_path.pop()
        else: # if heur_path is empty, perform uniform random sampling. Extend fwd tree and rev tree.

            if self.grow_iter_times % 2 == 0: # extend the forward tree first
                # extend the forward tree according to random_goal
                random_goal = self.chooseRandomGoal()
                best_node_fwd = self.chooseBestNode(random_goal)
                if best_node_fwd:
                    new_node_fwd = self.extend(best_node_fwd, random_goal)
                    self.connect(new_node_fwd, self.extendRev, self.chooseBestNodeRev, 'forward')
            else: # extend the reverse tree first
                random_goal = self.chooseRandomGoalRev()
                best_node_rev = self.chooseBestNodeRev(random_goal)
                if best_node_rev:
                    new_node_rev = self.extendRev(best_node_rev, random_goal)
                    self.connect(new_node_rev, self.extend, self.chooseBestNode, 'reverse')

        self.grow_iter_times +=1
        if self.grow_iter_times > 1000:
            self.grow_iter_times = 0

    # def biGrow(self):
    #     '''
    #     Bidirectional growth without connected heuristic
    #     :return:
    #     '''
    #     # random sampling
    #     if self.heur_path: # if heur_path is not empty, perform heuristic sampling. Extend only fwd tree.
    #         # indicator = random.random()
    #         # if indicator > self.heur_prob:
    #         #     random_goal = self.heuristicSampling()
    #         # else:
    #         #     random_goal = self.chooseRandomGoal()
    #         random_goal = self.heuristicSampling()
    #         best_node_fwd = self.chooseBestNode(random_goal)
    #         if best_node_fwd:
    #             new_node_fwd = self.extend(best_node_fwd, random_goal)
    #             if self.euclideanDistance(new_node_fwd, self.root_rev) < self.heur_path[-1].cost:
    #                 # print('pop()')
    #                 self.heur_path.pop()
    #
    #     else: # if heur_path is empty, perform uniform random sampling. Extend fwd tree and rev tree.
    #
    #         if self.grow_iter_times % 2 == 0: # extend the forward tree first
    #             # extend the forward tree according to random_goal
    #             random_goal = self.chooseRandomGoal()
    #             best_node_fwd = self.chooseBestNode(random_goal)
    #             if best_node_fwd:
    #                 new_node_fwd = self.extend(best_node_fwd, random_goal)
    #                 best_node_rev = self.chooseBestNodeRev(new_node_fwd.pose)
    #                 if best_node_rev:
    #                     new_node_rev = self.extendRev(best_node_rev, new_node_fwd.pose)
    #                     if new_node_rev and new_node_fwd:
    #                         if self.meet(new_node_fwd, new_node_rev):
    #                             # self.terminate = True
    #                             self.heur_path = self.getHeurTraj(new_node_rev)
    #         else:
    #             random_goal = self.chooseRandomGoalRev()
    #             best_node_rev = self.chooseBestNodeRev(random_goal)
    #             if best_node_rev:
    #                 new_node_rev = self.extendRev(best_node_rev, random_goal)
    #                 best_node_fwd = self.chooseBestNode(new_node_rev.pose)
    #                 if best_node_fwd:
    #                     new_node_fwd = self.extend(best_node_fwd, new_node_rev.pose)
    #                     if new_node_rev and new_node_fwd:
    #                         if self.meet(new_node_fwd, new_node_rev):
    #                             # self.terminate = True
    #                             self.heur_path = self.getHeurTraj(new_node_rev)
    #     self.grow_iter_times +=1
    #     if self.grow_iter_times > 1000:
    #         self.grow_iter_times = 0

    def getHeurTraj(self, meet_node):
        """
        backtrack the heuristic trajectory
        :param meet_node: the met node in the reverse tree
        :return: heuristic trajectory (list of Node)
        """
        traj = [meet_node]
        parent = meet_node.parent
        while parent:
            traj.append(parent)
            parent = parent.parent
        return traj[::-1]

    def computeControlScoreRev(self, expected_pose, random_goal):
        """
        compute the control score of the reverse tree
        :param expected_pose: Custom pose
        :param random_goal: Custom pose
        :return: score (float)
        """
        distance_from_random_goal = self.trajLengthRev(expected_pose, random_goal)
        score = 1.0 / distance_from_random_goal + 1e-9
        return score

    def extendRev(self, node, rand_goal):
        """
        extend the reverse tree from node
        :param node: Node
        :param rand_goal: Custom_pose
        :return: new_node
        """
        new_node = Node()
        best_control_index = -1

        best_control_score = 0.0

        for i in range(len(node.possible_controls)):
            if node.possible_controls[i].open:
                expected_pose = self.robotKinematic(node.pose, node.possible_controls[i])
                control_score = self.computeControlScoreRev(expected_pose, rand_goal)
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

        if new_node.isFree and new_node.depth < self.maxDepth:
            self.candidate_nodes_rev.append(new_node)
            # print('Rev new node pose: ', new_node.pose)
            '''for visualization------>begin'''
            center = [self.ogmap.gridIFromPose(new_node.pose), self.ogmap.gridJFromPose(new_node.pose)]
            self.ogmap.map = cv2.circle(self.ogmap.map, center, radius, color, thickness)
            # debug
            # cv2.imwrite('/home/mh/GithubProjects/BiRiskRRTValidation/results/' + 'tree.png', self.ogmap.map)
            '''for visualization------>end'''

            return new_node
        else:
            return None

    def trajLengthRev(self, pose, goal):
        """
        trajectory length computation on the reverse tree
        :param pose: Custom pose
        :param goal: Custom pose
        :return: distance_from_goal (float)
        """
        pose_euclide_distance = math.sqrt(math.pow(pose.x - goal.x, 2) + math.pow(pose.y - goal.y, 2))
        root_euclide_distance = math.sqrt(math.pow(self.root_rev.pose.x - goal.x, 2) +
                                          math.pow(self.root_rev.pose.y - goal.y, 2))
        position_improvement = pose_euclide_distance / root_euclide_distance
        rotation_diff = math.atan2(goal.y - pose.y, goal.x - pose.x) - pose.theta

        if rotation_diff > math.pi:
            rotation_diff -= 2 * math.pi
        if rotation_diff < -math.pi:
            rotation_diff += 2 * math.pi

        distance_from_goal = position_improvement + self.rotationWeight * abs(rotation_diff)

        return distance_from_goal


class BiRiskRRTPed(BiRiskRRT):
    def __init__(self, params, ogmap, data_name):
        super(BiRiskRRTPed, self).__init__(params, ogmap)
        self.data_name = data_name
        self.trajectorReader = TrajReader(data_name, self.timeStep)

    def getNewObsPrediction(self, time):
        """
        update the occupancy grip map according to new obstacle prediction at 'time'
        :return:
        """
        self.trajectorReader.update_ogmap(self.ogmap, time)

