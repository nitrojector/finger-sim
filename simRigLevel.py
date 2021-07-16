'''

Rigid model
Level estimate

'''


import numpy as np
from numpy import array
from matplotlib import pyplot as plt
import vu
import random

# Data
resX = []
resY = []
expData = []

# Params
simStep = np.pi / 25
mu = 0.66
cofAng = np.tan(mu)
indvPlot = True
normDispLen = 20

scanLvl = 60
tolerance = 1

# Set points and constraints
mP1 = array([-20, 0])
mP2 = array([20, 0])
dJ1 = 63
dJ2 = 50
objLen = 50


#
minPos = np.zeros((8, 2))
minP7Rec = 0
maxPos = np.zeros((8, 2))
maxP7Rec = 0

for theta1 in np.arange(np.pi / 4, np.pi, simStep):
    for theta2 in np.arange(- np.pi , np.pi, simStep):
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
                
                # CHANGE CHANGE CHANGE
                # if np.arccos(np.dot(objV, armLV)) < np.tan(mu) and np.arccos(np.dot(objV, armRV)) < np.tan(mu):

                addCount += 1
                if mP7[1] > scanLvl - tolerance and mP7[1] < scanLvl + tolerance:
                    plt.plot([mP1[0], mP2[0]], [mP1[1], mP2[1]], '#000000')
                    plt.plot([mP2[0], mP4[0]], [mP2[1], mP4[1]], 'b')
                    plt.plot([mP1[0], mP3[0]], [mP1[1], mP3[1]], 'b')
                    plt.plot([mP3[0], mP5[0]], [mP3[1], mP5[1]], 'g')
                    plt.plot([mP4[0], mP6[0]], [mP4[1], mP6[1]], 'g')
                    plt.plot([mP5[0], mP6[0]], [mP5[1], mP6[1]], 'g', linestyle='dashed')
                    plt.plot(mP7[0], mP7[1], 'r^')
                    plt.plot([mP5[0], mP5[0] + normDispLen * armLV[0]], [mP5[1], mP5[1] + normDispLen * armLV[1]],
                             '#44ddbf',
                             linestyle='dashed')
                    plt.plot([mP6[0], mP6[0] + normDispLen * armRV[0]], [mP6[1], mP6[1] + normDispLen * armRV[1]],
                             '#44ddbf',
                             linestyle='dashed')

                    plt.title(f'scanLvl={scanLvl} tolerance={tolerance}')
                    plt.xlim(mP1[0] - dJ1 - dJ2 / 2, mP1[1] + dJ1 + dJ2 / 2)
                    plt.ylim(0, dJ1 + dJ2 + 5)
                    plt.gca().set_aspect('equal', adjustable='box')
                    plt.show()
                    plt.clf()

                    print(armRV)
                    print(np.rad2deg(np.arccos(np.dot(objV, armLV))), np.rad2deg(np.pi - np.arccos(np.dot(objV, armRV))))
                    if np.arccos(np.dot(objV, armLV)) < cofAng and np.arccos(np.dot(objV, armRV)) < cofAng:
                        if mP7[0] < minP7Rec:
                          minP7Rec = mP7[0]
                          minPos[0] = [mP2[0], mP4[0]]
                          minPos[1] = [mP2[1], mP4[1]]
                          minPos[2] = [mP1[0], mP3[0]]
                          minPos[3] = [mP1[1], mP3[1]]
                          minPos[4] = [mP3[0], mP5[0]]
                          minPos[5] = [mP3[1], mP5[1]]
                          minPos[6] = [mP4[0], mP6[0]]
                          minPos[7] = [mP4[1], mP6[1]]
                          
                        if mP7[0] > maxP7Rec:
                          maxP7Rec = mP7[0]
                          maxPos[0] = [mP2[0], mP4[0]]
                          maxPos[1] = [mP2[1], mP4[1]]
                          maxPos[2] = [mP1[0], mP3[0]]
                          maxPos[3] = [mP1[1], mP3[1]]
                          maxPos[4] = [mP3[0], mP5[0]]
                          maxPos[5] = [mP3[1], mP5[1]]
                          maxPos[6] = [mP4[0], mP6[0]]
                          maxPos[7] = [mP4[1], mP6[1]]
                        
                        resX.append(mP7[0])
                        resY.append(mP7[1])
                        
                        expData.append(np.concatenate((theta1, theta2, theta3, mP3, mP4, mP5, mP6, mP7), axis=None))
            except ArithmeticError:
                continue
              
plt.title(f'scanLvl={scanLvl} tolerance={tolerance}')
plt.xlim(mP1[0] - dJ1 - dJ2 / 2, mP1[1] + dJ1 + dJ2 / 2)
plt.ylim(0, dJ1 + dJ2 + 5)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
plt.clf()


'''
plt.plot([mP2[0], mP4[0]], [mP2[1], mP4[1]], 'b')
plt.plot([mP1[0], mP3[0]], [mP1[1], mP3[1]], 'b')
plt.plot([mP3[0], mP5[0]], [mP3[1], mP5[1]], 'g')
plt.plot([mP4[0], mP6[0]], [mP4[1], mP6[1]], 'g')
'''

plt.plot(minPos[0], minPos[1], 'b')
plt.plot(minPos[2], minPos[3], 'b')
plt.plot(minPos[4], minPos[5], 'g')
plt.plot(minPos[6], minPos[7], 'g')

plt.plot(maxPos[0], maxPos[1], 'b')
plt.plot(maxPos[2], maxPos[3], 'b')
plt.plot(maxPos[4], maxPos[5], 'g')
plt.plot(maxPos[6], maxPos[7], 'g')


plt.scatter(resX, resY)
plt.title(f'scanLvl={scanLvl} tolerance={tolerance}')
plt.xlim(mP1[0] - dJ1 - dJ2 / 2, mP1[1] + dJ1 + dJ2 / 2)
plt.ylim(0, dJ1 + dJ2 + 5)
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

np.savetxt("simRigLevel_%032x.csv " % random.getrandbits(128), np.asarray(expData), delimiter=",")




