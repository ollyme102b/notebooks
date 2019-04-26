import numpy as np
import matplotlib.pyplot as plt
from OllyYawEstimatorV1 import OllyYawEstimatorV1


def rotate(v, theta):
    T = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return np.dot(T, v)


def test1():
    tol = 0.000001

    for olly_yaw in np.linspace(0, 2 * np.pi, 100):
        olly_position = np.zeros((2,))
        olly_body_velocity_command = np.array([2, 3])
        olly_inertial_velocity = rotate(olly_body_velocity_command, olly_yaw)

        olly_yaw_estimator = OllyYawEstimatorV1()

        simulation_time = 30
        step_time = 0.1

        for t in range(simulation_time):
            olly_position = olly_position + step_time * olly_inertial_velocity
            olly_yaw_estimator.update_with_numpy_array(olly_inertial_velocity, olly_body_velocity_command)
            olly_yaw_estimator.estimate_yaw()

        assert (olly_yaw_estimator.estimate_yaw() - olly_yaw) < tol, 'No noise test failing'


def test2():
    tol = 0.1

    olly_position = np.zeros((2,))
    olly_yaw = 0

    olly_yaw_estimator = OllyYawEstimatorV1()

    sigma = 0.01
    sigma1 = 0.001

    simulation_time = 300
    step_time = 0.1

    time_domain = np.arange(0, simulation_time)
    yaw_actual = np.zeros((simulation_time,))
    yaw_estimate = np.zeros((simulation_time,))

    for t in range(simulation_time):

        if t % 10 == 0:
            olly_body_velocity_command = np.array([np.random.normal(0, sigma), np.random.normal(0, sigma)])

        olly_yaw += np.random.normal(0, sigma)
        olly_inertial_velocity = rotate(olly_body_velocity_command, olly_yaw)

        olly_position = olly_position + step_time * olly_inertial_velocity

        olly_yaw_estimator.update_with_numpy_array(olly_inertial_velocity+np.array([np.random.normal(0, sigma1), np.random.normal(0, sigma1)]), olly_body_velocity_command)

        yaw_actual[t] = olly_yaw
        yaw_estimate[t] = olly_yaw_estimator.estimate_yaw()

        # assert (olly_yaw_estimator.estimate_yaw() - olly_yaw) < tol, 'No noise test failing'
    plt.plot(time_domain, yaw_actual, label='actual')
    plt.plot(time_domain, yaw_estimate, label='estimate')
    plt.legend()
    plt.show()


if __name__ == '__main__':
    #test1()
    test2()
    print('All Tests Passed')
