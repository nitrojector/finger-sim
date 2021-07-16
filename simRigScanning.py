'''

Rigid model
Angle scanning estimate

'''


import numpy as np
from numpy import array
from matplotlib import pyplot as plt
import vu
import random

# Data
resX = []
resY = []
expData = [] # FORMAT ['theta1', 'theta2', 'theta3', 'mP3x', 'mP3y', 'mP4x', 'mP4y', 'mP5x', 'mP5y', 'mP6x', 'mP6y', 'mP7x', 'mP7y']


# Params
simStep = np.pi / 50
mu = 0.66
indvPlot = True
normDispLen = 20


# Set points and constraints
mP1 = array([-20, 0])
mP2 = array([20, 0])
dJ1 = 63
dJ2 = 50
objLen = 50

for theta1 in np.arange(0, np.pi / 2, simStep):
    for theta2 in np.arange(0, np.pi / 2, simStep):
        addCount = 0
        for theta3 in np.arange(-2 * np.pi / 3, 2 * np.pi / 3, simStep):
            # print(f'θ1={round(np.rad2deg(theta1), 3)} θ2={round(np.rad2deg(theta2), 3)} θ3={round(np.rad2deg(theta3), 3)}', end='\r')
            try:
                mP3 = vu.ang2DVector(mP1, theta1, dJ1)
                mP4 = vu.ang2DVector(mP2, -theta2, dJ1)
                mP5 = vu.ang2DVector(mP3, theta3, dJ2)
                mP6 = vu.getMidPt(mP5, objLen, mP4, dJ2)
                mP7 = vu.getCenterPt(mP5, mP6)

                objV = vu.normalize(mP6 - mP5)
                armLV = vu.normalize(np.array([mP5[1] - mP3[1], mP3[0] - mP5[0]]))
                armRV = vu.normalize(np.array([mP4[1] - mP6[1], mP6[0] - mP4[0]]))

                # print(f'P5 = {mP5}\tP6 = {mP6}\t\tP7 = {mP7}')
                # print(f'VL={np.dot(objV, armLV)}\tVR={np.dot(objV, armRV)}\tM=[{np.linalg.norm(armLV)}\t{np.linalg.norm(armRV)}]')
                if abs(np.arccos(np.dot(objV, armLV))) < np.tan(mu) and abs(np.arccos(np.dot(objV, armRV))) < np.tan(mu):
                    resX.append(mP7[0])
                    resY.append(mP7[1])
                    # CHANGE CHANGE CHANGE
                    expData.append(np.concatenate((theta1, theta2, theta3, mP3, mP4, mP5, mP6, mP7), axis=None))
                addCount += 1
                if indvPlot:
                    plt.plot([mP1[0], mP2[0]], [mP1[1], mP2[1]], '#000000')
                    plt.plot([mP2[0], mP4[0]], [mP2[1], mP4[1]], 'b')
                    plt.plot([mP1[0], mP3[0]], [mP1[1], mP3[1]], 'b')
                    plt.plot([mP3[0], mP5[0]], [mP3[1], mP5[1]], 'r')
                    plt.plot([mP4[0], mP6[0]], [mP4[1], mP6[1]], 'r')
                    plt.plot([mP5[0], mP6[0]], [mP5[1], mP6[1]], 'g', linestyle='dashed')
                    plt.plot(mP7[0], mP7[1], 'r^')
                    plt.plot([mP5[0], mP5[0] + normDispLen * armLV[0]], [mP5[1], mP5[1] + normDispLen * armLV[1]], '#44ddbf',
                             linestyle='dashed')
                    plt.plot([mP6[0], mP6[0] + normDispLen * armRV[0]], [mP6[1], mP6[1] + normDispLen * armRV[1]], '#44ddbf',
                             linestyle='dashed')
            except ArithmeticError:
                continue
        if indvPlot and addCount > 0:
            plt.title(f'θ1={round(np.rad2deg(theta1), 3)}° θ2={round(np.rad2deg(theta2), 3)}°')
            plt.xlim(mP1[0] - dJ1 - dJ2 / 2, mP1[1] + dJ1 + dJ2 / 2)
            plt.ylim(0, dJ1 + dJ2 + 5)
            plt.gca().set_aspect('equal', adjustable='box')
            plt.figure(dpi=1200)
            plt.show()


np.savetxt("simRigScanning_%032x.csv " % random.getrandbits(128), np.asarray(expData), delimiter=",")

plt.title(f'simStep={round(simStep, 3)} μ={round(mu, 3)} j1len={round(dJ1, 3)} j2len={round(dJ2, 3)} objLen={round(objLen, 3)}')
plt.xlim(mP1[0] - dJ1 - dJ2 / 2, mP1[1] + dJ1 + dJ2 / 2)
plt.ylim(0, dJ1 + dJ2 + 5)
plt.gca().set_aspect('equal', adjustable='box')
plt.scatter(resX, resY)
plt.show()




