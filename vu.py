import numpy as np


def getLinesAsFormulas(data):
    mP1 = np.array([-40, 0])
    mP2 = np.array([40, 0])
    print(ptsToFormula([mP1[0], mP2[0]], [mP1[1], mP2[1]]))
    print(ptsToFormula([mP2[0], data[2]], [mP2[1], data[3]]))
    print(ptsToFormula([mP1[0], data[0]], [mP1[1], data[1]]))
    print(ptsToFormula([data[0], data[4]], [data[1], data[5]]))
    print(ptsToFormula([data[2], data[6]], [data[3], data[7]]))


def ptsToFormula(x, y) -> str:
    return f'((1-t)*{x[0]}+t*{x[1]},(1-t)*{y[0]}+t*{y[1]})'


def getAngBtw(v1, v2):
    dot = v1[0] * v2[0] + v1[1] * v2[1]  # dot product between [x1, y1] and [x2, y2]
    det = v1[0] * v2[1] - v1[1] * v2[0]  # determinant
    angle = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    return angle


def getPerpVectorCW(v1):
    return normalize(np.array([v1[1], -v1[0]]))


def getPerpVectorCCW(v1):
    return normalize(np.array([-v1[1], v1[0]]))


def getCenterPt(p1, p2):
    return np.array([(p2[0] + p1[0]) / 2, (p2[1] + p1[1]) / 2])


def ang2DVector(v1, theta, magnitude):
    rel_x = v1[0] / np.linalg.norm(v1)
    rel_y = v1[1] / np.linalg.norm(v1)
    v2x = magnitude * (rel_x * np.cos(theta) - rel_y * np.sin(theta)) + v1[0]
    v2y = magnitude * (rel_y * np.cos(theta) + rel_x * np.sin(theta)) + v1[1]
    # return v1 + magnitude * normalize(vectorRot(v1, theta))
    # print(v1, theta, magnitude, v2x, v2y)
    return np.array([v2x, v2y])


def getMidPt(v1, v1r, v2, v2r):
    mv = v2 - v1
    h = 2 * getArea(np.linalg.norm(mv), v1r, v2r) / np.linalg.norm(mv)
    rot_angle = np.arcsin(h / v1r)
    return np.squeeze(np.asarray(v1 + v1r * normalize(vectorRot(mv, rot_angle))))


def getArea(l1, l2, l3):
    if not triangleIsValid(l1, l2, l3):
        raise ArithmeticError('Invalid Triangle')
    s = (l1 + l2 + l3) / 2
    return np.sqrt(s * (s - l1) * (s - l2) * (s - l3))


def vectorRot(v1, theta):
    rotMatrix = np.matrix([[np.cos(theta), -np.sin(theta)],
                           [np.sin(theta), np.cos(theta)]])
    return np.squeeze(np.asarray(np.swapaxes(rotMatrix * np.array([[v1[0]], [v1[1]]]), 0, 1)))


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        raise ArithmeticError('Zero Vector Operation')
    return v / norm


def triangleIsValid(l1, l2, l3):
    return l1 + l2 > l3 and l1 + l3 > l2 and l2 + l3 > l1
