import math


class Params:
    def __init__(self):
        # time step between each node (s)
        self.timeStep = 0.4
        # maximum depth a node can have (forward tree)
        self.fwdMaxDepth = 300
        # discretization of the robot's linear speed, used to create the set of possible controls from a node
        self.nv = 3 # must be int
        # discretization of the robot's angular speed, used to create the set of possible controls from a node
        self.nphi = 5 # must be int
        # maximum risk value a node can have to be considered free
        self.threshold = 90
        # weight of the risk component in the computation of the node's score
        self.socialWeight = 0.1
        # weight of the risk component in the computation of the node's score
        self.rotationWeight = 0.5
        # time allocated for the tree growth between map updates (s)
        self.growTime = self.timeStep
        # percentage of the time the final goal is chosen as random goal
        self.bias = 0.01
        # maximum distance the robot can be from the final goal to consider the goal reached (m)
        self.goalTh = 0.5
        # size of the window from which to pick a random goal (m)
        self.windowSize = 20.0
        # robot lenght (m) /!\: the wheel axis is assumed to be in the middle
        self.robotLength = 1.5
        # robot width (m)
        self.robotWidth = 0.7
        # minimum robot linear speed (m/s), set it to 0 or a negative value to allow reverse
        self.vMin = -0.1
        # maximum robot linear speed (m/s)
        self.vMax = 1.0
        # maximum linear acceleration (m/s^2)
        self.accMax = 0.5
        # maximum angular speed (rad/s)
        self.omegaMax = 0.5
        # maximum angular acceleration (rad/s^2)
        self.accOmegaMax = 0.5

        # wait for reassigning
        # self.start = [0.0, -11.5, 1.57] # A and blank
        # self.goal = [0.0, 11.5, -1.57] # A and blank
        # self.start = [0.0, -11.5, 0.0]  # B
        # self.goal = [0.0, 11.5, 0.0]  # B
        self.start = [-11.5, -10.0, 0.0] # C
        self.goal = [10.5, 10.0, math.pi] # C


class BiParams(Params):
    def __init__(self):
        super().__init__()
        self.stddev =3.0
        self.heur_prob = 0.5
        self.connect_th = 1.0
        self.maxDepth = 300
        # connect heuristic steps (at least 1, i.e., no connect heuristic)
        self.connect_heur = 5
