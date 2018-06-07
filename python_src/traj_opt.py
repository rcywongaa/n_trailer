import numpy as np

from pydrake.all import MathematicalProgram, SolverType
from pydrake.math import sqrt, sin, cos, tan, atan, atan2
import matplotlib.pyplot as plt

def AddEqualityConstraint(mp, expression1, expression2, slack = 0.001):
    mp.AddConstraint(expression1 - expression2 <= slack)
    mp.AddConstraint(expression1 - expression2 >= -slack)

def compute_trajectory(num_trailers, x_func, y_func, N, dt, max_steer_angle = np.pi/6, trailer_length = 1.0, slack = 0.001):
    '''
    Given a parametric curve defined by x_func(n) and y_func(n) where x_func(n) must be strictly increasing,
    Find the trajectory of the n-trailer system that minimizes the deviation of each trailer from the defined curve.

    :param num_trailers: int, number of trailers in this n-trailer system
    :param x_func, y_func: function, parametric curve defined by x_func(n) and y_func(n)
        x_func must be strictly increasing
    :param N: float, number of time steps to take
        Termianl position is defined by x_func(N*dt), y_func(N*dt)
    :param dt: float, integration time step
    '''

    num_vehicles = num_trailers + 1
    mp = MathematicalProgram()

    # Control variable for offset between successive trailers
    c = mp.NewContinuousVariables(1, "c")
    mp.AddConstraint(c[0] >= 0.0)
    mp.AddConstraint(c[0] <= trailer_length)
    # c = [1.0]
    # Control variable for starting offset
    lag = mp.NewContinuousVariables(1, "lag")
    mp.AddConstraint(lag[0] >= 0.0)
    # lag = [0.0]

    # x[t][i] is the x position of the i-th (0 is the tractor) trailer at time t
    # Desired values
    x_desired_over_time = np.array([[x_func(t*dt - lag[0] - i*c[0]) for i in range(num_vehicles)] for t in range(N+1)])
    y_desired_over_time = np.array([[y_func(t*dt - lag[0] - i*c[0]) for i in range(num_vehicles)] for t in range(N+1)])
    # Control variables
    x_over_time = mp.NewContinuousVariables(N+1, num_vehicles, "x")
    x_d_over_time = mp.NewContinuousVariables(N+1, num_vehicles, "x_d")
    y_over_time = mp.NewContinuousVariables(N+1, num_vehicles, "y")
    y_d_over_time = mp.NewContinuousVariables(N+1, num_vehicles, "y_d")
    theta_over_time = mp.NewContinuousVariables(N+1, num_vehicles, "theta")
    theta_d_over_time = mp.NewContinuousVariables(N+1, num_vehicles, "theta_d")
    assert(x_desired_over_time.shape == x_over_time.shape)
    assert(y_desired_over_time.shape == y_over_time.shape)

    x_error_sq = np.sum((x_over_time - x_desired_over_time)**2)
    y_error_sq = np.sum((y_over_time - y_desired_over_time)**2)
    mp.AddQuadraticCost(x_error_sq + y_error_sq)

    # Initialize x[0] and y[0]
    for i in range(num_vehicles):
        AddEqualityConstraint(mp, x_over_time[0][i], 0 - i*trailer_length, slack)
        AddEqualityConstraint(mp, y_over_time[0][i], 0, slack)
        AddEqualityConstraint(mp, x_d_over_time[0][i], 0, slack)
        AddEqualityConstraint(mp, y_d_over_time[0][i], 0, slack)
        AddEqualityConstraint(mp, theta_over_time[0][i], 0, slack)
        AddEqualityConstraint(mp, theta_d_over_time[0][i], 0, slack)

    for t in range(1, N+1):
        for i in range(num_vehicles):
            # State update
            AddEqualityConstraint(mp, x_over_time[t][i], x_over_time[t-1][i] + x_d_over_time[t-1][i]*dt, slack)
            AddEqualityConstraint(mp, y_over_time[t][i], y_over_time[t-1][i] + y_d_over_time[t-1][i]*dt, slack)
            AddEqualityConstraint(mp, theta_over_time[t][i], theta_over_time[t-1][i] + theta_d_over_time[t-1][i]*dt, slack)

            # No slip constraint
            mp.AddConstraint(x_d_over_time[t][i] >= slack) # To avoid division by zero error
            AddEqualityConstraint(mp, atan2(y_d_over_time[t][i], x_d_over_time[t][i]), theta_over_time[t][i], slack)

            v = sqrt(x_d_over_time[t][i]**2 + y_d_over_time[t][i]**2)
            if i == 0: # Tractor dynamics
                mp.AddConstraint(theta_d_over_time[t][i] <= v * tan(max_steer_angle) / trailer_length)
                mp.AddConstraint(theta_d_over_time[t][i] >= v * tan(-max_steer_angle) / trailer_length)
            else: # Trailer dynamics
                angle_difference = theta_over_time[t][i] - theta_over_time[t][i-1]
                preceding_v = sqrt(x_d_over_time[t][i-1]**2 + y_d_over_time[t][i-1]**2)
                AddEqualityConstraint(mp, theta_d_over_time[t][i], -preceding_v*sin(angle_difference)/trailer_length, slack)
                AddEqualityConstraint(mp, v, preceding_v*cos(angle_difference), slack)
                AddEqualityConstraint(mp, x_d_over_time[t][i], v*cos(theta_over_time[t][i]), slack)
                AddEqualityConstraint(mp, y_d_over_time[t][i], v*sin(theta_over_time[t][i]), slack)

    mp.Solve()
    x_trajectory = mp.GetSolution(x_over_time)
    y_trajectory = mp.GetSolution(y_over_time)
    # print("x_trajectory shape: " + str(x_trajectory.shape))
    # print("y_trajectory shape: " + str(y_trajectory.shape))
    print("x0_trajectory = " + str(x_trajectory[:, 0]))
    print("y0_trajectory = " + str(y_trajectory[:, 0]))
    # print("x1_trajectory = " + str(x_trajectory[:, 1]))
    # print("y1_trajectory = " + str(y_trajectory[:, 1]))
    c = mp.GetSolution(c)
    lag = mp.GetSolution(lag)
    print("c = " + str(c))
    print("lag = " + str(lag))
    return x_trajectory, y_trajectory

if __name__ == "__main__":
    # define the target parametric curve
    # x must be strictly increasing
    def x_func(n):
        return n
    def y_func(n):
        return n

    dt = 0.05
    num_trailers = 2
    N = 100
    x_trajectory, y_trajectory = compute_trajectory(num_trailers, x_func, y_func, N, dt)
    fig = plt.figure()
    ax = fig.add_subplot(111) # 111 means occupies 1/1 (all) rows, 1/1 (all) columns, with index 1
    xd = np.array([x_func(t*dt) for t in range(len(x_trajectory))])
    yd = np.array([y_func(t*dt) for t in range(len(y_trajectory))])
    ax.scatter(xd, yd, s = 10, c = np.array([[1.0, 0.0, 0.0]]))
    markers = ['o', 'v', '^', '<', '>', '8', 's', 'p', '*', 'h', 'H', 'D', 'd', 'P', 'X']
    for i in range(len(x_trajectory[0])):
        color = np.random.rand(1, 3)
        ax.scatter(x_trajectory[:, i], y_trajectory[:, i], s = 50, c = color, alpha=0.5, marker = markers[i])
    plt.show()
