from Path_Planning.level1.circleLineIntersection import circle_line_intersection
from MPC_Alg.v1.CFTOCSolverV1 import CFTOCSolverV1
import numpy as np
import matplotlib.pyplot as plt

# Initialize Molly position
xm = -4
ym = -8

# object length
l = 2

# constraint line
x0 = -10
y0 = -4
x1 = 4
y1 = -10

# Initialize Folly position
xf, yf = circle_line_intersection(xm, ym, l, x0, y0, x1, y1)

# simulation
T = 600  # Total time
dt = 1  # time per iteration
sigma = 0.05  # simulation noise standard deviation

# initialize MPC Alg
controller = CFTOCSolverV1(np.eye(2), dt * np.eye(2), np.array([xf, yf]), np.array([xf, yf]), 10)

for t in range(T):
    # molly movement
    xm += 0.01
    ym += 0.05

    # find optimal folly position
    xfb, yfb = circle_line_intersection(xm, ym, l, x0, y0, x1, y1, xf, yf)

    # find optimal command
    vc = controller.calculate_optimal_actuation(np.array([xf, yf]),
                                            np.array([xfb, yfb]))  # optimal velocity command

    # plot state before actuation hold on pltis equal
    plt.plot([x0, x1], [y0, y1], 'k--', linewidth=1, label='Path')  # constraint path
    plt.plot([xm, xfb], [ym, yfb], 'b-', linewidth=2, label='Load')  # object lifted
    plt.plot(xm, ym, '.', color='olive', label='Molly Position')  # MollyPosition
    plt.plot(xfb, yfb, 'o', color='indigo', label='Folly Optimal')  # Folly optimal position
    plt.plot(xf, yf, 'r.', label='Folly Position')  # Folly Actual Positions
    plt.quiver(xf, yf, 10 * vc[0], 10 * vc[1], color='orange')  # Folly velocity command
    plt.legend()
    plt.title('Level 1 Path Planner Level 1 MPC')
    plt.show(block = False)
    plt.pause(1)
    plt.close()

    # simulate actuation of optimal command
    xf = xf + dt * vc[0] + np.random.normal(0, sigma)
    yf = yf + dt * vc[1] + np.random.normal(0, sigma)
