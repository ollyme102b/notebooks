import numpy as np
import gekko


class CFTOCSolverV3:
    def calculate_optimal_actuation(self, x0, Xm):
        """
        Updates x0 and xbar parameters then solves
        :param x0: updated x0
        :param Xm: updated Xm
        :return: optimal actuation
        """
        for i in range(self.nx):
            self.x0[i].value = x0[i]
        for t in range(self.N):
            for i in range(self.nx):
                self.Xm[i, t].value = Xm[i, t]
        return self.solve_optimal_actuation()

    def solve_optimal_actuation(self):
        """
        solves cvxpy problem and returns problem solution variable
        :return: optimal actuation value
        """
        self.m.solve(disp=False)
        return np.array([self.U[0, 0].value, self.U[1, 0].value]).flatten()

    def status(self):
        """
        returns problem status
        """
        # return self.problem.status
        pass

    def __init__(self, A, B, x0, Xm, N, umax, l):
        """
        Initializes cvxpy problem with the following params
        :param A: state transition matrix
        :param B: control matrix
        :param x0: initial state
        :param xbar: desired path
        :param N: number of time steps per cftoc
        """

        self.nx = x0.shape[0]
        self.nu = B.shape[1]
        self.N = N

        # assert types for debuging
        assert type(A) is np.ndarray, 'A must be a numpy array'
        assert type(B) is np.ndarray, 'B must be a numpy array'
        assert type(x0) is np.ndarray, 'x0 must be a numpy array'
        assert type(Xm) is np.ndarray, 'xbar must be a numpy array'
        assert type(N) is int, 'N must be an int'

        # assert problem requirements
        assert A.shape[0] == self.nx, 'row A must = number of x0 states'
        assert A.shape[0] == A.shape[1], 'A must be square'
        assert B.shape[0] == self.nx, 'row B must = number of x0 states'
        assert x0.ndim == 1, 'x0 must be flat'
        assert umax is None or umax > 0, 'umax must be None or greater than 0'

        # create GEKKO object
        self.m = gekko.GEKKO()

        # initialize GEKKO Vars
        self.X = self.m.Array(self.m.Var, (self.nx, N + 1))
        self.U = self.m.Array(self.m.Var, (self.nu, N))

        # initilaize GEKKO Params
        self.x0 = self.m.Array(self.m.Param, (self.nx,))
        for i in range(self.nx):
            self.x0[i].value = x0[i]
        self.Xm = self.m.Array(self.m.Param, (self.nx, N))
        for i in range(self.nx):
            for t in range(N):
                self.Xm[i, t].value = Xm[i, t]

        # dynamic constraints
        for i in range(self.nx):
            self.m.Equation(self.X[i, 0] == self.x0[i])
        for t in range(N):
            temp = np.dot(A, self.X[:, t]) + np.dot(B, self.U[:, t])
            for i in range(self.nx):
                self.m.Equation(self.X[i, t + 1] == temp[i])

        # input constraints
        if umax is not None:
            self.umax = self.m.Param(value=umax)
            for t in range(N):
                for i in range(self.nu):
                    self.m.Equation(self.U[i, t] ** 2 <= self.umax ** 2)

        # initialize cost function
        J = 0

        # state cost
        for t in range(1, N + 1):
            J += ((self.X[0, t] - self.Xm[0, t - 1]) ** 2 + (self.X[1, t] - self.Xm[1, t - 1]) ** 2 - l ** 2) ** 2

        # input cost
        for t in range(N):
            for i in range(self.nu):
                J += self.U[i, t] ** 2

        self.m.Obj(J)

