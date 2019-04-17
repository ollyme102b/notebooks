import numpy as np
from Path_Planning.level1.circleLineIntersection import circle_line_intersection


def circle_line_intersection_path(m_pos, l, l0, l1, d_m_pos, n, f_pos=np.array([0, 0])):
    """
    :param m_pos: molly position vector
    :param l: length of constraint line between molly and folly
    :param l0: position vector of start of path line constraint
    :param l1: position vector of end of path line constraint
    :param d_m_pos: deviation of molly per time step
    :param n: path horizon
    :param f_pos: folly position vector
    :return: folly path
    """

    folly_path = np.zeros((2, n))
    xp = f_pos[0]
    yp = f_pos[1]

    for i in range(n):
        xf, yf = circle_line_intersection(m_pos[0], m_pos[1], l, l0[0], l0[1], l1[0], l1[1], xp=xp, yp=yp)

        # add time step path to output path array
        folly_path[0, i] = xf
        folly_path[1, i] = yf

        # update folly position for next iteration
        xp = xf
        yp = yf

        # update molly position for next iteration
        m_pos = m_pos + d_m_pos

    return folly_path
