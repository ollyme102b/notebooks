from FollyHighLevelControllerV1 import FollyHighLevelControllerV1
import numpy as np
import matplotlib.pyplot as plt
import timeit

# Initialize Molly position
molly_position = np.array([-4, -8])

# Initialize Folly position
folly_position = np.array([-2, -7.5])

# object length
object_length = 2

# line path constraint
l0 = np.array([-10, -4])
l1 = np.array([4, -10])

# simulation
T = 600  # Total time
dt = 0.2  # time per iteration
sigma = 0.02  # simulation noise standard deviation

# initialize MPC Alg
follyHLC = FollyHighLevelControllerV1(molly_position,
                                      folly_position,
                                      object_length,
                                      l0, l1,  # constraint path ends
                                      10,  # horizon length
                                      dt,  # step time
                                      0.1  # maximum speed
                                      )

# constant molly velocity
molly_velocity_command = np.array([0.01, 0.05])

iteration_time = timeit.timeit()  # time iteration for real time plot
for t in range(T):
    load_length_deviation = np.linalg.norm(folly_position - molly_position) - object_length

    # plot current state
    plt.plot([l0[0], l1[0]], [l0[1], l1[1]], 'k--', linewidth=1, label='Path')  # constraint path
    plt.plot([molly_position[0], folly_position[0]], [molly_position[1], folly_position[1]], 'b-', linewidth=2,
             label='Load: dev {:.1f}cm'.format(load_length_deviation * 100))  # object lifted
    plt.plot(molly_position[0], molly_position[1], '.', color='olive', label='Molly Position')  # MollyPosition
    plt.plot(folly_position[0], folly_position[1], 'r.', label='Folly Position')  # Folly Actual Positions
    plt.legend()
    plt.title('Folly High Level Controller V1')
    plt.show(block=False)
    plt.pause(dt - (timeit.timeit() - iteration_time))  # pause in real time
    iteration_time = timeit.timeit()  # reset iteration time
    plt.close()

    # actuate molly command with noise
    molly_position = molly_position + dt * molly_velocity_command + dt * np.random.normal(0, sigma, 2)

    # compute folly command
    folly_velocity_command = follyHLC.optimal_input(molly_position, folly_position)

    # actuate optimal command with noise
    folly_position = folly_position + dt * folly_velocity_command + dt * np.random.normal(0, sigma, 2)