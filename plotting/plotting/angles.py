import numpy as np


def unit_vec(u):
    """Scales input vector u to unit length"""
    return np.asarray(u) / np.linalg.norm(u)


def polar(r, theta):
    """Returns cartesian vector from polar input form"""
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y


def vec_angle3(u, v, n):
    """Computes signed angle between 3-element vectors u and v with reference vector n"""
    norm = np.linalg.norm
    l = norm(u) * norm(v)
    n = norm(n)
    cos_t = np.dot(u, v) / l
    sin_t = np.dot(np.cross(n, u), v) / l
    t = np.arccos(cos_t)
    if sin_t >= 0:
        return t
    else:
        return -t


def vec_angle2(u, v):
    """Computes signed angle between 2-element vectors u and v
    u, v: Vectors of shape (2,)
    """
    a = *u, 0
    b = *v, 0
    n = [0, 0, 1]
    return vec_angle3(a, b, n)
