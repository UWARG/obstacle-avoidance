import math

# https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points


def midpoint_of_line(p1, p2):
    pass


def distance__between_two_points(p1, p2):
    pass


def distance_from_point_to_line(p1, l1, r1):
    num = abs((ry - ly) * px - (rx - lx) * py + rx * ly - ry * lx)
    den = math.sqrt((rx - lx) ** 2 + (ry - ly) ** 2)
    return num / den


def length_of_line(p1, p2):
    pass
