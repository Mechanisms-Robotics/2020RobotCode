import path_follower


def generate_path_from_points(points):  # tested implicitly in path follower
    '''
    Given an iterable of points ((x1, y1), (x2, y2), ...), returns a Path
    object.
    '''

    assert points is not None
    assert len(points) > 1

    path_segments = []
    last_point = points[0]

    for point in points[1:]:
        path_segments.append(
            path_follower.LinearPathSegment(last_point, point))
        last_point = point

    return path_follower.Path(path_segments)
