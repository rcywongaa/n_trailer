####################
# Given a parametric curve defined by x_func(n) and y_func(n) where x_func(n) must be strictly increasing,
# Find the trajectory of the n-trailer system that minimizes the deviation of each trailer from the defined curve.
####################

import numpy as np

from pydrake.all import MathematicalProgram, SolverType
from pydrake.math import sqrt, sin, cos, tan, atan, atan2
import matplotlib.pyplot as plt

def AddEqualityConstraint(mp, expression1, expression2, slack = 0.001):
    mp.AddConstraint(expression1 - expression2 <= slack)
    mp.AddConstraint(expression1 - expression2 >= -slack)

def compute_trajectory(x_func, y_func, num_trailers, dt, T = 1.0, max_steer_angle = np.pi/6, trailer_length = 1.0, slack = 0.001):
    N = int(np.ceil(T/dt))
    mp = MathematicalProgram()

    # Control variable for offset between successive trailers
    c = mp.NewContinuousVariables(1, "c")
    mp.AddConstraint(c[0] >= slack)
    mp.AddConstraint(c[0] <= trailer_length)

    # x[t][i] is the x position of the i-th (0 is the tractor) trailer at time t
    # Desired values
    xd_over_time = np.array([[x_func(t*dt - i*c[0]) for i in range(num_trailers+1)] for t in range(N+1)])
    yd_over_time = np.array([[y_func(t*dt - i*c[0]) for i in range(num_trailers+1)] for t in range(N+1)])
    # Control variables
    x_over_time = mp.NewContinuousVariables(N+1, num_trailers+1, "x")
    y_over_time = mp.NewContinuousVariables(N+1, num_trailers+1, "y")
    theta_over_time = mp.NewContinuousVariables(N+1, num_trailers+1, "theta")
    assert(xd_over_time.shape == x_over_time.shape)
    assert(yd_over_time.shape == y_over_time.shape)

    x_error_sq = np.sum((x_over_time - xd_over_time)**2)
    y_error_sq = np.sum((y_over_time - yd_over_time)**2)
    mp.AddQuadraticCost(x_error_sq + y_error_sq)

    # Initialize x[0] and y[0]
    for i in range(num_trailers+1):
        AddEqualityConstraint(mp, x_over_time[0][i], 0 - i*trailer_length, slack)
        AddEqualityConstraint(mp, y_over_time[0][i], 0, slack)

    for t in range(N):
        for i in range(num_trailers+1):
            x_d = (x_over_time[t+1][i] - x_over_time[t][i]) / dt
            mp.AddConstraint(x_d >= slack) # To avoid division by zero error
            y_d = (y_over_time[t+1][i] - y_over_time[t][i]) / dt
            theta_d = (theta_over_time[t+1][i] - theta_over_time[t][i]) / dt
            v = sqrt(x_d**2 + y_d**2)
            AddEqualityConstraint(mp, atan2(y_d, x_d), theta_over_time[t][i], slack)
            if i == 0:
                mp.AddConstraint(theta_d <= v * tan(max_steer_angle) / trailer_length)
                mp.AddConstraint(theta_d >= v * tan(-max_steer_angle) / trailer_length)
            else:
                angle_difference = theta_over_time[t][i] - theta_over_time[t][i-1]
                preceding_x_d = (x_over_time[t+1][i-1] - x_over_time[t][i-1]) / dt
                preceding_y_d = (y_over_time[t+1][i-1] - y_over_time[t][i-1]) / dt
                preceding_v = np.sqrt(preceding_x_d**2 + preceding_y_d**2)
                AddEqualityConstraint(mp, theta_d, -preceding_v*sin(angle_difference) / trailer_length, slack)
                AddEqualityConstraint(mp, v, preceding_v*cos(angle_difference), slack)

        # Trailer dynamics w.r.t. last trailer
        # for i in reversed(range(num_trailers)): # does not apply to last trailer
                # xdot_next = (x_over_time[t+1][i+1] - x_over_time[t][i+1]) / dt
                # ydot_next = (y_over_time[t+1][i+1] - y_over_time[t][i+1]) / dt
                # c_next = xdot_next / np.sqrt(xdot_next**2 + ydot_next**2 + slack) # cos of theta_next
                # s_next = ydot_next / np.sqrt(xdot_next**2 + ydot_next**2 + slack) # sin of theta_next
                # x_curr = x_over_time[t][i+1] + trailer_length * c_next
                # y_curr = y_over_time[t][i+1] + trailer_length * s_next
                # AddEqualityConstraint(mp, x_over_time[t][i], x_curr)
                # AddEqualityConstraint(mp, y_over_time[t][i], y_curr)

    mp.SetSolverOption(SolverType.kSnopt, 'Print file', "/tmp/snopt.out")
    mp.Solve()
    x_trajectory = mp.GetSolution(x_over_time)
    y_trajectory = mp.GetSolution(y_over_time)
    print("x0_trajectory = " + str(x_trajectory[:, 0]))
    print("y0_trajectory = " + str(y_trajectory[:, 0]))
    # print("x1_trajectory = " + str(x_trajectory[:, 1]))
    # print("y1_trajectory = " + str(y_trajectory[:, 1]))
    c_result = mp.GetSolution(c)
    print("c = " + str(c_result))
    return x_trajectory, y_trajectory

if __name__ == "__main__":
    # define the target parametric curve
    # x must be strictly increasing
    def x_func(n):
        return n
    def y_func(n):
        return n

    dt = 0.1
    num_trailers = 2
    T = 10
    x_trajectory, y_trajectory = compute_trajectory(x_func, y_func, num_trailers, dt, T = T)
    fig = plt.figure()
    ax = fig.add_subplot(111) # 111 means occupies 1/1 (all) rows, 1/1 (all) columns, with index 1
    xd = np.array([x_func(t*dt) for t in range(len(x_trajectory))])
    yd = np.array([y_func(t*dt) for t in range(len(y_trajectory))])
    ax.scatter(xd, yd, s = 10, c = np.array([[1.0, 0.0, 0.0]]))
    markers = ['o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X']
    for i in range(num_trailers+1):
        color = np.random.rand(1, 3)
        ax.scatter(x_trajectory[:, i], y_trajectory[:, i], s = 50, c = color, alpha=0.5, marker = markers[i])
    plt.show()
