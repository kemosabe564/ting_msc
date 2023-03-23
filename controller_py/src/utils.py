import math
import numpy as np
import configuration as config


def angle_vectors(vector_1, vector_2):
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return round(angle, 3)


def cart2pol(cart=np.array([0, 0])):
    """
    :param cart: [x,y]
    :return:
    """
    rho = np.sqrt(cart[0] ** 2 + cart[1] ** 2)
    phi = np.arctan2(cart[1], cart[0])
    return np.round(np.array([rho, phi]), 3)


def pol2cart(pol=np.array([0, 0])):
    """
    :param pol: [rho, phi]
    :return:
    """
    x = pol[0] * np.cos(pol[1])
    y = pol[0] * np.sin(pol[1])
    return np.round(np.array([x, y]), 3)


def renew_vec(old_vec):
    new_vec = np.concatenate((old_vec[1:], np.array([old_vec[-1]])), axis=0)
    return new_vec


def normalize(item):
    if np.linalg.norm(item) < config.step_threshold:
        return np.array([0., 0.])
    else:
        return item / np.linalg.norm(item)


def in_domain(point):
    if (point[0] > config.domain[0][0] and point[0] < config.domain[2][0] and point[1] > config.domain[0][1] and point[1] < config.domain[2][1]):
        return True
    else:
        False


def yaw_from_quaternion(x, y, z, w):
    """
    Convert quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radiant (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is roation around z in radians (counterclockwise)
    """
    # t0 = +2.0 * (w*x + y*z)
    # t1 = +1.0 -2.0*(x*x + y*y)
    # roll_x = math.atan2(t0,t1)
    #
    # t2 = +2.0 * (w*y -z *x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < 1.0 else t2
    # pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    # return roll_x, pitch_y, yaw_z #in radians
    return yaw_z  # in radians


class ObstacleAvoidance:
    def __init__(self):
        self.ref_lines_domain = np.array([
            [config.domain[0], config.domain[1]],
            [config.domain[1], config.domain[2]],
            [config.domain[2], config.domain[3]],
            [config.domain[3], config.domain[0]]
        ])
        self.ref_lines = self.ref_lines_domain
        if config.obstacles:
            print('fix dit')

    @staticmethod
    def perpendicular(a):
        b = np.empty_like(a)
        b[0] = a[1]
        b[1] = -a[0]
        return b

    @staticmethod
    def det(a,b):
        return a[0]*b[1] - a[1]*b[0]

    @staticmethod
    def check_direction_vectors(vector_1, vector_2):
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        return dot_product > 0

    def check_in_domain(self, point):
        if (point[0] > config.domain[0][0] and point[0] < config.domain[2][0] and point[1] > config.domain[0][1] and point[1] < config.domain[1][1]):
            if config.obstacles:
                print("fix dit")
            else:
                return True
        else:
            return False

    def line_intersection(self, locations):
        pot_inter = {}
        move_vec = np.round(np.array([locations[1][0] - locations[0][0],
                                      locations[1][1] - locations[0][1]]), 3)

        for count, ref_line in enumerate(self.ref_lines):
            xdiff = np.array([ref_line[0][0] - ref_line[1][0], locations[0][0] - locations[1][0]])
            ydiff = np.array([ref_line[0][1] - ref_line[1][1], locations[0][1] - locations[1][1]])

            div = self.det(xdiff, ydiff)
            if div != 0:
                d = (self.det(*ref_line), self.det(*locations))
                inter = np.array([self.det(d, xdiff) / div, self.det(d, ydiff) / div])

                inter_vec = np.round(np.array([inter[0] - locations[0][0],
                                               inter[1] - locations[0][1]]), 3)

                if self.check_direction_vectors(move_vec, inter_vec):
                    pot_inter[count] = {'inter': inter,
                                        'inter_dist': np.linalg.norm(inter_vec)}

        if pot_inter:
            key_min = min(pot_inter.keys(), key=(lambda k: pot_inter[k]['inter_dist']))
            return key_min, pot_inter[key_min]['inter']
        else:
            return None, None

    def obstacle_avoidance(self, start_point, move):
        no_obs_new_point = start_point + move

        if not self.check_in_domain(start_point + move):
            index_line, inter = self.line_intersection(np.array([start_point, start_point + move]))
            if isinstance(index_line, int):
                ref_line = self.ref_lines[index_line]
                ref_vec = np.array([ref_line[1][0] - ref_line[0][0], ref_line[1][1] - ref_line[0][1]])

                per_vec = self.perpendicular(ref_vec)
                if not per_vec[0]:
                    new_point = np.array([no_obs_new_point[0], 2 * inter[1] - move[1] - start_point[1]])
                else:
                    new_point = np.array([2 * inter[0] - move[0] - start_point[0], no_obs_new_point[1]])

                # Check if mirrored point is within region (we do not calculate double bouncing)
                if self.check_in_domain(new_point):
                    return new_point, True
                else:
                    # if mirrored point is not within region of influence, try to move the opposite direction
                    new_point_alt = start_point - move
                    if self.check_in_domain(new_point_alt):
                        return new_point_alt, True

                    # if this also doesn't result in an feasible location, stay were you are
                    else:
                        return start_point, True
            else:
                return start_point, True
        else:
            return start_point + move, True