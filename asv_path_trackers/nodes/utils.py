#!/usr/bin/env python
import time
import heapq
import numpy as np

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

class Controller(object):
    def __init__(self):
        pass

    def update(self, vobj):
        pass

    def draw(self, axes, N, fcolor, ecolor):
        pass

    def visualize(self, fig, axes, t, n):
        pass


def eucledian_path_length(path):
    """
    Calculates the Eucledian length of a 2D path.

    The input is assumed to be a numpy array.
    """
    # Calculate length of path traveled
    e = path[1:,0:2] - path[0:-1,0:2]
    # Sum of L2-norm along each row
    length = np.sum( np.sum(np.abs(e)**2,axis=-1)**(1./2) )
    return length
def normalize_angle(angle, angle_ref):
    """
    Makes 'angle' compatible with 'angle_ref' such that the numerical
    difference is at most PI
    """
    if angle_ref == np.Inf:
        return angle

    # Get angle within 2*PI of angle_ref
    diff = angle_ref - angle
    if diff > 0:
        new_angle = angle + (diff - np.fmod(diff, 2*np.pi))
    else:
        new_angle = angle + (diff + np.fmod(-diff, 2*np.pi))

    # Make sure angle is on the closest side of angle_ref
    diff = angle_ref - new_angle
    if diff > np.pi:
        new_angle += 2*np.pi
    elif diff < -np.pi:
        new_angle -= 2*np.pi

    return new_angle


def int_circles(x1, y1, r1, x2, y2, r2):
    """Tests for intersection between two circles.

    Returns:
        int = True on intersection
        coords = coordinates of intersection (numpy.array)

    http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
    """

    # Distance between circle centers
    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    if (d > r1 + r2) or (d < np.abs(r1-r2)):
        # No solution
        return False, []

    a = (r1**2 - r2**2 + d**2 ) / (2*d)

    xp = x1 + a*(x2 - x1)/d
    yp = y1 + a*(y2 - y1)/d

    if (d == r1 + r2):
        # One solution
        coords = np.array([xp, yp])
        return True, coords
    else:
        # Two solutions
        h = np.sqrt(r1**2 - a**2);
        coords = np.array([[xp+h*(y2-y1)/d, yp-h*(x2-x1)/d],
                           [xp-h*(y2-y1)/d, yp+h*(x2-x1)/d]])
    return True, coords
