"""

LQR local path planning for dubin car

author: Fei Meng

"""

import math
import random
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as la
from param import Params

# import numpy.linalg as la
SHOW_ANIMATION = True
show_middle_animation = False


class LQRPlanner:
    """
    input: Node a and b
    output: a trajectory
    """

    def __init__(self, timeStep):
        self.MAX_TIME = 1000.0  # Maximum simulation time
        self.DT = timeStep  # Time tick
        self.GOAL_DIST = 0.0001
        self.MAX_ITER = 15000
        self.EPS = 0.0001
        self.params = Params()

    def lqr_planning(self, new_node_prim, new_node, show_middle_animation=False):
        ini_linear_vel = math.sqrt(new_node_prim.vel.linear.x_vel ** 2 + new_node_prim.vel.linear.y_vel ** 2)
        goal_linear_vel = math.sqrt(new_node.vel.linear.x_vel ** 2 + new_node.vel.linear.y_vel ** 2)
        start = (new_node_prim.pose.x, new_node_prim.pose.y, new_node_prim.pose.theta,
                 ini_linear_vel,
                 new_node_prim.vel.angular / ini_linear_vel)
        goal = (new_node.pose.x, new_node.pose.y, new_node.pose.theta,
                goal_linear_vel, new_node.vel.angular / goal_linear_vel)
        rx, ry, rt, rv, rk = [start[0]], [start[1]], [start[2]], [start[3]], [start[4]]

        x = np.array([a - b for a, b in zip(start, goal)]).reshape(5, 1)  # State vector
        theta = start[2]
        v = start[3]
        kai = start[4]
        # Linear system model
        # A, B = self.get_system_model()

        found_path = False

        time = 0.0
        while time <= self.MAX_TIME:
            time += self.DT
            A = np.array(
                [[0, 0, -v * np.sin(theta), np.cos(theta), 0],
                 [0, 0, v * np.cos(theta), np.sin(theta), 0],
                 [0, 0, 0, kai, v],
                 [0, 0, 0, 0, 0],
                 [0, 0, 0, 0, 0]])
            B = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 1]).reshape(5, 2)

            u = self.lqr_control(A, B, x)

            v = float(u[0]) * self.DT + v
            v = np.clip(v, self.params.vMin, self.params.vMax)
            kai = float(u[1]) * self.DT + kai
            kai = np.clip(kai, self.params.acckaiMin, self.params.acckaiMax)

            theta = np.clip(v * kai, self.params.omegaMin, self.params.omegaMax) * self.DT + theta
            if theta < -np.pi:
                theta += 2 * np.pi
            elif theta > np.pi:
                theta -= 2 * np.pi

            x = A @ x + B @ u

            rx.append(x[0, 0] + goal[0])
            ry.append(x[1, 0] + goal[1])
            rt.append(x[2, 0] + goal[2])
            rv.append(x[3, 0] + goal[3])
            rk.append(x[4, 0] + goal[4])

            d = math.sqrt((goal[0] - rx[-1]) ** 2 + (goal[1] - ry[-1]) ** 2 + (goal[2] - rt[-1]) ** 2 + (
                    goal[3] - rv[-1]) ** 2 + (goal[4] - rk[-1]) ** 2)
            if d <= self.GOAL_DIST:
                found_path = True
                break

            # animation
            if show_middle_animation:  # pragma: no cover
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(start[0], start[1], "or")
                plt.plot(goal[0], goal[1], "og")
                plt.plot(rx, ry, "-r")
                plt.axis("equal")
                plt.pause(1.0)

        if not found_path:
            print("Cannot found lqr path")
            return [], [], [], [], []
        return rx, ry, rt, rv, rk

    def solve_dare(self, A, B, Q, R):
        """
        solve a discrete time_Algebraic Riccati equation (DARE)
        """
        X, Xn = Q, Q

        for i in range(self.MAX_ITER):
            Xn = A.T @ X @ A - A.T @ X @ B @ (la.inv(R + B.T @ X @ B) + 0.1 * np.eye(2)) @ B.T @ X @ A + Q
            if (abs(Xn - X)).max() < self.EPS:
                break
            X = Xn
        return Xn

    def dlqr(self, A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = (la.inv(B.T @ X @ B + R) + np.eye(2)) @ (B.T @ X @ A)

        eigValues = la.eigvals(A - B @ K)

        return K, X, eigValues

    def get_linearized_system_model(self, x, y, theta, v, kai):
        A = np.array(
            [[0, 0, -v * np.sin(theta), np.cos(theta), 0],
             [0, 0, v * np.cos(theta), np.sin(theta), 0],
             [0, 0, 0, kai, v],
             [0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0]])
        B = np.array([0, 0, 0, 0, 0, 0, 1, 0, 0, 1]).reshape(5, 2)
        return A, B

    def lqr_control(self, A, B, x):
        Q = 1 * np.eye(5)
        R = 0.1 * np.eye(2)
        Kopt, X, ev = self.dlqr(A, B, Q, R)
        u = -Kopt @ x
        uv = np.clip(u[0], self.params.accMin, self.params.accMax)
        uk = np.clip(u[1], self.params.accOmegaMin, self.params.acckaiMax)
        u = np.array([uv, uk])
        return u


def main():
    print(__file__ + " start!!")

    ntest = 5  # number of goal
    area = 100.0  # sampling area

    lqr_planner = LQRPlanner()

    for i in range(ntest):
        start = [0.1, 0.2, np.deg2rad(10), 0.5, 0.1]
        goal = [random.uniform(-area, area), random.uniform(-area, area), random.uniform(-np.pi, np.pi),
                random.uniform(0, 10), random.uniform(-0.25, 0.25)]

        rx, ry = lqr_planner.lqr_planning(start, goal, show_middle_animation=SHOW_ANIMATION)

        if SHOW_ANIMATION:  # pragma: no cover
            plt.plot(start[0], start[1], "sr")
            plt.plot(goal[0], goal[1], "og")
            plt.plot(rx, ry, "-r")
            plt.axis("equal")
            plt.pause(1.0)


if __name__ == '__main__':
    main()
