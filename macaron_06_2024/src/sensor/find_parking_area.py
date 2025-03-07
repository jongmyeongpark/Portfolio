import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
import enum
from typing import Tuple 

def generate_random_parking_area(num_parking: int, size_range: tuple, cone_count: tuple, add_noise: bool, add_other_cones: bool):
    assert(size_range[0] < size_range[1])

    size = np.random.random((2, )) * (size_range[1] - size_range[0]) + size_range[0]
    if size[0] < size[1]:
        size[0], size[1] = size[1], size[0]
    angle = np.random.rand() * 2 * np.pi

    vector_w = np.array([np.cos(angle), np.sin(angle)]) * size[0]
    vector_h = np.array([np.cos(angle+np.pi/2), np.sin(angle+np.pi/2)]) * size[1]

    start_pos = np.random.random((2, ))
    end_pos = start_pos + vector_w * num_parking + vector_h

    parking_area_num = np.random.randint(0, num_parking)
    cones = []
    for i in range(num_parking):
        start_cone = start_pos + vector_w * i

        cones_w1 = np.linspace(start_cone, start_cone + vector_w, cone_count[0] + 2)[1:-1, :]
        cones_h1 = np.linspace(start_cone, start_cone + vector_h, cone_count[1] + 2)[1:-1, :]
        cones_w2 = np.linspace(start_cone + vector_h, start_cone + vector_h + vector_w, cone_count[0] + 2)[1:-1, :]

        if i == parking_area_num:
            cones.append(np.vstack([cones_h1, cones_w2, start_cone, start_cone + vector_h]))
        else:
            cones.append(np.vstack([cones_w1, cones_h1, cones_w2, start_cone, start_cone + vector_h]))

    cones.append(np.linspace(start_pos + vector_w * num_parking, end_pos, cone_count[1] + 2))

    cones = np.vstack(cones)

    if add_other_cones:
        cone_count = cones.shape[0]

        expand_direction = (vector_w + vector_h) / np.linalg.norm(vector_w + vector_h)
        area_min = start_pos - expand_direction
        area_max = end_pos + expand_direction

        random_cones = np.random.random((cone_count // 2, 2)) * (area_max - area_min) + area_min

        random_cones_out_rectangle = []
        rectangle = np.array([area_min, start_pos + vector_h, area_max, end_pos - vector_h])
        for cone in random_cones:
            if not check_point_in_rectangle(cone, rectangle)[0]:
                random_cones_out_rectangle.append(cone)
        
        cones = np.vstack([cones, *random_cones_out_rectangle])

    if add_noise:
        cones += np.random.random(cones.shape) / 5

    return cones, size


class ESide(enum.Enum):
    WIDTH_1 = 0
    WIDTH_2 = 1
    HEIGHT_1 = 2
    HEIGHT_2 = 3
    VERTEX = 4

    def __int__(self):
        return self.value

def draw_rectangle(plot, rectangle):
    rectangle = np.vstack([rectangle, rectangle[0]])
    plot.plot(rectangle[:, 0], rectangle[:, 1])

def get_angle_between_vectors(vec1: np.ndarray, vec2: np.ndarray) -> float:
    dot = vec1 @ vec2
    norm_vec1, norm_vec2 = np.linalg.norm(vec1), np.linalg.norm(vec2)
    theta = np.arccos(np.clip(dot / (norm_vec1 * norm_vec2), -1.0, 1.0))
    return np.rad2deg(theta)

def make_margin_rectangle(rectangle: np.ndarray, margin: float) -> Tuple[np.ndarray, np.ndarray]:
    center = np.mean([rectangle[0], rectangle[2]], axis=0)
    center_to_vertex = rectangle - center
    expand_vectors = margin * center_to_vertex / np.linalg.norm(center_to_vertex, axis=1)[:, np.newaxis]
    outer_rectangle = rectangle + expand_vectors
    inner_rectangle = rectangle - expand_vectors

    return outer_rectangle, inner_rectangle

def check_point_in_rectangle(point: np.ndarray, rectangle: np.ndarray):

    center = np.mean([rectangle[0], rectangle[2]], axis=0)

    vectors = np.vstack([center, rectangle]) - point
    vectors = np.hstack([vectors, np.zeros((5, 1))])

    # 1 h
    alpha1 = np.cross(vectors[0], vectors[1])[-1]
    alpha2 = np.cross(vectors[1], vectors[2])[-1]
    alpha3 = np.cross(vectors[2], vectors[0])[-1]
    if (alpha1 * alpha1 > 1e-6) and (alpha2 * alpha3 > 1e-6) and (alpha3 * alpha1 > 1e-6):
        return True, ESide.HEIGHT_1

    # 2 w
    alpha1 = np.cross(vectors[0], vectors[2])[-1]
    alpha2 = np.cross(vectors[2], vectors[3])[-1]
    alpha3 = np.cross(vectors[3], vectors[0])[-1]
    if (alpha1 * alpha1 > 1e-6) and (alpha2 * alpha3 > 1e-6) and (alpha3 * alpha1 > 1e-6):
        return True, ESide.WIDTH_1

    # 3 h
    alpha1 = np.cross(vectors[0], vectors[3])[-1]
    alpha2 = np.cross(vectors[3], vectors[4])[-1]
    alpha3 = np.cross(vectors[4], vectors[0])[-1]
    if (alpha1 * alpha1 > 1e-6) and (alpha2 * alpha3 > 1e-6) and (alpha3 * alpha1 > 1e-6):
        return True, ESide.HEIGHT_2

    # 4 w
    alpha1 = np.cross(vectors[0], vectors[4])[-1]
    alpha2 = np.cross(vectors[4], vectors[1])[-1]
    alpha3 = np.cross(vectors[1], vectors[0])[-1]
    if (alpha1 * alpha1 > 1e-6) and (alpha2 * alpha3 > 1e-6) and (alpha3 * alpha1 > 1e-6):
        return True, ESide.WIDTH_2
    
    return False, -1

def find_side(dist: np.ndarray):
    min_index, max_index = 0, 0
    w, h = 0, 0
    if dist[0] < dist[1]:
        if dist[0] < dist[2]:
            min_index = 0
            h = dist[0]

            if dist[1] < dist[2]:
                max_index = 2
                w = dist[1]
            else:
                max_index = 1
                w = dist[2]
        else:
            min_index = 2
            max_index = 1
            h = dist[2]
            w = dist[0]
    else:
        if dist[1] < dist[2]:
            min_index = 1
            h = dist[1]

            if dist[0] < dist[2]:
                max_index = 2
                w = dist[0]
            else:
                max_index = 0
                w = dist[2]
        else:
            min_index = 2
            max_index = 0
            h = dist[2]
            w = dist[1]

    return (min_index, max_index), (w, h)

def find_rectangle(points: np.ndarray, size: Tuple[float, float], margin, allow_angle=10):
    assert(size[0] > margin / 2)
    assert(size[1] > margin / 2)

    ret = []

    for rectangle_candidate in combinations(points, 4):
        rectangle_candidate = np.array(rectangle_candidate)

        dist = np.linalg.norm(rectangle_candidate[1:, :] - rectangle_candidate[0, :], axis=1)
        (min_index, max_index), (w, h) = find_side(dist)

        rectangle_candidate[[1+min_index, 1]] = rectangle_candidate[[1, 1+min_index]]
        rectangle_candidate[[1+max_index, 2]] = rectangle_candidate[[2, 1+max_index]]

        if (size[0] - margin <= w <= size[0] + margin) and (size[1] - margin <= h <= size[1] + margin):
            v1 = rectangle_candidate[1] - rectangle_candidate[0]
            v2 = rectangle_candidate[2] - rectangle_candidate[1]
            v3 = rectangle_candidate[3] - rectangle_candidate[2]
            v4 = rectangle_candidate[0] - rectangle_candidate[3]

            theta1 = get_angle_between_vectors(v1, v2)
            theta2 = get_angle_between_vectors(v2, v3)
            theta3 = get_angle_between_vectors(v3, v4)
            theta4 = get_angle_between_vectors(v4, v1)

            is_right1 = 90 - allow_angle < theta1 < 90 + allow_angle
            is_right2 = 90 - allow_angle < theta2 < 90 + allow_angle
            is_right3 = 90 - allow_angle < theta3 < 90 + allow_angle
            is_right4 = 90 - allow_angle < theta4 < 90 + allow_angle
            
            if is_right1 and is_right2 and is_right3 and is_right4:
                count = [0, 0, 0, 0, 4]
                points_in_rectangle = []
                outer_rectangle, inner_rectangle = make_margin_rectangle(rectangle_candidate, margin)
                for point in points:
                    is_in_outer, side = check_point_in_rectangle(point, outer_rectangle)
                    is_in_inner, _ = check_point_in_rectangle(point, inner_rectangle)
                    if is_in_outer and (not is_in_inner):
                        count[int(side)] += 1
                        points_in_rectangle.append(point)
                ret.append((rectangle_candidate, count, (w, h)))

    return ret

def find_parking_area(points: np.ndarray, size: Tuple[float, float], margin: float, allow_angle=5):
    """find parking area

    Args:
        points (np.ndarray): cones centroid
        size (tuple[w, h]): parking area size(long side is w)
        margin (float): margin
        allow_angle (int, optional): 90 +- allow_angle. Defaults to 10.

    Returns:
        np.ndarray: four vertex of parking area(rectangle)
    """
    fittest_parking_area = [None, 99999999, False]
    for parking_area, count, (w, h) in find_rectangle(points, size, margin, allow_angle=allow_angle):
        if count[int(ESide.WIDTH_2)] == 0 and count[int(ESide.WIDTH_1)] == 0:
            mse = (size[0] - w)**2 + (size[1] - h)**2
            if mse < fittest_parking_area[1]:
                fittest_parking_area = [parking_area, mse, True]

    if not fittest_parking_area[2]:
        return False, None  
    
    return True, fittest_parking_area[0]
            
# ax = plt.subplot()

# cones, size = generate_random_parking_area(3, (1, 10), (4, 2), True, False)

# ax.scatter(cones[:, 0], cones[:, 1], c='#0000FF')
# for parking_area, cone, count, (main_rectangle, outer_rectangle, inner_rectangle) in find_rectangle(cones, size, 0.7, allow_angle=3):
#     if count[int(ESide.HEIGHT_1)] > 0 or count[int(ESide.HEIGHT_2)] > 0:
#         ax.scatter(cone[:, 0], cone[:, 1], c='#FF0000')

#         if count[int(ESide.WIDTH_1)] == 0 or count[int(ESide.WIDTH_2)] == 0:
#             print(parking_area)
#             draw_rectangle(ax, main_rectangle)
        
#         draw_rectangle(ax, outer_rectangle)
#         draw_rectangle(ax, inner_rectangle)

# ax.axis('equal')
# plt.show()
