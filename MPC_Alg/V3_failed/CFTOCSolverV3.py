import numpy as np
import cvxpy
from qcqp import *
from packaging import version


class CFTOCSolverV3:
    def calculate_optimal_actuation(self, x0, Xm):
        """
        Updates x0 and xbar parameters then solves
        :param x0: updated x0
        :param xbar: updated xbar
        :return: optimal actuation
        """
        self.x0.value = x0
        self.Xm.value = Xm
        return self.solve_optimal_actuation()

    def solve_optimal_actuation(self):
        """
        solves cvxpy problem and returns problem solution variable
        :return: optimal actuation value
        """
        self.qcqp.improve(IPOPT)
        return np.array(self.U[:, 0].value).flatten()

    def status(self):
        """
        returns problem status
        """
        return self.problem.status

    def __init__(self, A, B, x0, Xm, N, umax, l):
        """
        Initializes cvxpy problem with the following params
        :param A: state transition matrix
        :param B: control matrix
        :param x0: initial state
        :param xbar: desired path
        :param N: number of time steps per cftoc
        """

        # assert correct CVXPY
        assert version.parse(cvxpy.__version__) < version.parse('1'), 'CVXPY version cannot be CVXPY v1'

        nx = x0.shape[0]
        nu = B.shape[1]

        # assert types for debuging
        assert type(A) is np.ndarray, 'A must be a numpy array'
        assert type(B) is np.ndarray, 'B must be a numpy array'
        assert type(x0) is np.ndarray, 'x0 must be a numpy array'
        assert type(Xm) is np.ndarray, 'xbar must be a numpy array'
        assert type(N) is int, 'N must be an int'

        # assert problem requirements
        assert A.shape[0] == nx, 'row A must = number of x0 states'
        assert A.shape[0] == A.shape[1], 'A must be square'
        assert B.shape[0] == nx, 'row B must = number of x0 states'
        assert x0.ndim == 1, 'x0 must be flat'
        assert umax is None or umax > 0, 'umax must be None or greater than 0'

        # initialize cvxpy problem
        self.X = cvxpy.Variable(nx, N + 1)
        self.U = cvxpy.Variable(nu, N)
        self.x0 = cvxpy.Parameter(nx)
        self.Xm = cvxpy.Parameter(nx, N)

        # initialize Parameters
        self.x0.value = x0
        self.Xm.value = Xm

        # initialize constraints
        constraints = []

        # dynamics constraints
        constraints += [self.X[:, 0] == self.x0]
        for t in range(N):
            constraints += [self.X[:, t + 1] == A * self.X[:, t] + B * self.U[:, t]]

        # input constraints
        if umax is not None:
            self.umax = cvxpy.Parameter()
            self.umax.value = umax
            for t in range(N):
                constraints += [cvxpy.sum_squares(self.U[:, t]) <= self.umax ** 2]

        # cost function initialization
        cost = 0

        # state cost
        for t in range(1, N + 1):
            cost += 0 * cvxpy.sum_squares(self.X[:, t] - self.Xm[:, t - 1])

        # input cost
        for t in range(N):
            cost += cvxpy.sum_squares(self.U[:, t])

        # create cvxpy problem
        self.problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)

        # Create a QCQP handler.
        self.qcqp = QCQP(self.problem)

        # Solve the SDP relaxation and get a starting point to a local method
        self.qcqp.suggest()

        # Attempt to improve the starting point given by the suggest method
        qcqp.improve(COORD_DESCENT)
