import pymunk
import numpy as np

EPSILON = 0.00001


# normalize an N-D vector
def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm > EPSILON:
        return vector/norm
    else:
        return 0.0*vector

# create 2D rotation matrix for angle theta CCW
def R(theta):
    th = np.radians(theta)
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s], [s, c]])

# generate hallway with incoming funnel
def genEnv(space, theta, f, rect_size, height, width):

    hallway = [
        pymunk.Segment(space.static_body, (0.8*width, height/2-0.5*f*rect_size), (width, height/2-0.5*f*rect_size), 2.0),
        pymunk.Segment(space.static_body, (0.8*width, height/2+0.5*f*rect_size), (width, height/2+0.5*f*rect_size), 2.0),
    ]

    lower_wall = np.array([[0., 0.], R(theta) @ np.array([0.75*height, 0.])])
    upper_wall = [[0., 0.], R(-theta) @ np.array([0.75*height,0.])]
    uw = [pt + np.array([0.8*width, height/2-(0.5*f*rect_size)]) for pt in upper_wall]
    lw = [pt + np.array([0.8*width, height/2+(0.5*f*rect_size)]) for pt in lower_wall]
    backwall = [uw[1], lw[1]]

    inroom = [pymunk.Segment(space.static_body, list(seg[0]), list(seg[1]), 3.0)
                for seg in [uw, lw, backwall]]

    return hallway + inroom
