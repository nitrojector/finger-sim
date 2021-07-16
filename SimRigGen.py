"""

A complete rewrite of simRigScanning.py and simRigLevel.py
out of frustration of the chaos and mess in those two programs

Dependency:
    vu.py

"""

import numpy as np
import cv2 as cv
import sys
from numpy import array
from matplotlib import pyplot as plt
import vu
import random
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.figure import Figure

# ### Output ####
# External
scatterPts = [[0], [0]]
savesAngle = []
savesPts = []

# Internal
jointLvlRefLocMax = np.zeros(10)  # [0:mP3x, 1:mP3y, 2:mP4x, 3:mP4y, 4:mP5x, 5:mP5y, 6:mP6x, 7:mP6y, 8:mP7x, 9:mP7y]
jointLvlRefLocMin = np.zeros(10)  # [0:mP3x, 1:mP3y, 2:mP4x, 3:mP4y, 4:mP5x, 5:mP5y, 6:mP6x, 7:mP6y, 8:mP7x, 9:mP7y]

# ### Parameters (Changeable) ####

# 1: Regular scanning through θ1->θ2->θ3~>θ4 with reports for every θ1,θ2 combination
# 2: Level pass with scanLvl for motion along a line
# 3: Linear pass with motion from motionPt1 -> motionPt2
SIM_MODE = 2

simStep = np.pi / 50
mu = 0.66
t1Range = [np.pi / 4, 2 * np.pi / 3]
t2Range = [np.pi / 4, 2 * np.pi / 3]
t3Range = [-np.pi / 2, np.pi / 2]

# Global SIM_MODE Params
plotIndv = False
normDispLen = 10
VERBOSE = False
saveData = True

# SIM_MODE=2 Params
scanLvl = 90
tolerance = 0.5
prefConcave = True

# SIM_MODE=3 Params
motionPt1 = [0, 0]
motionPt2 = [30, 30]


# ### Set points and constraints ####
mP1 = array([-40, 0])
mP2 = array([40, 0])
dJoint1 = 63
dJoint2 = 35.5
objLen = 50
COF_ANGLE = np.tan(mu)


# style 1 -> Plot essentials   style 2 -> Also plot dashes
def plotStickModel(data, style=1, ang=[0]):
    global mP1, mP2, ax
    if len(ang) > 1:
        ax.title.set_text(f'mP7x={round(ang[0], 3)} '
                          f'θ1={round(ang[1], 3)}° '
                          f'θ2={round(ang[2], 3)}° '
                          f'θ3={round(ang[3], 3)}° '
                          f'θ4={round(ang[4], 3)}°')
    if style >= 1:
        ax.plot([mP1[0], mP2[0]], [mP1[1], mP2[1]], '#000000')
        ax.plot([mP2[0], data[2]], [mP2[1], data[3]], 'b')
        ax.plot([mP1[0], data[0]], [mP1[1], data[1]], 'b')
        ax.plot([data[0], data[4]], [data[1], data[5]], 'c')
        ax.plot([data[2], data[6]], [data[3], data[7]], 'c')
        ax.plot(data[8], data[9], 'r^')
    if style >= 2:
        mV3 = array([data[0], data[1]])
        mV4 = array([data[2], data[3]])
        mV5 = array([data[4], data[5]])
        mV6 = array([data[6], data[7]])
        ax.plot([data[4], data[6]], [data[5], data[7]], 'g', linestyle='dashed')
        ax.plot([data[4], data[4] + (normDispLen * vu.getPerpVectorCW(mV5 - mV3))[0]],
                [data[5], data[5] + (normDispLen * vu.getPerpVectorCW(mV5 - mV3))[1]],
                'm', linestyle='dashed')
        ax.plot([data[6], data[6] + (normDispLen * vu.getPerpVectorCCW(mV6 - mV4))[0]],
                [data[7], data[7] + (normDispLen * vu.getPerpVectorCCW(mV6 - mV4))[1]],
                'm', linestyle='dashed')
    if SIM_MODE == 2:
        ax.plot([-90, 90], [scanLvl + tolerance, scanLvl + tolerance], '--x', linestyle='dashed')
        ax.plot([-90, 90], [scanLvl - tolerance, scanLvl - tolerance], '--x', linestyle='dashed')


def showPlot(title=''):
    global mP1, dJoint1, dJoint2
    if title == '':
        plt.title(f'θ1={round(np.rad2deg(theta1), 3)}° θ2={round(np.rad2deg(theta2), 3)}°')
    else:
        plt.title(title)
    plt.xlim(mP1[0] - dJoint1 - dJoint2 / 2, mP1[1] + dJoint1 + dJoint2 / 2)
    plt.ylim(0, dJoint1 + dJoint2 + 5)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.figure(dpi=1200)
    plt.show()


def getCurrentState(precision=3):
    global mP3, mP4, mP5, mP6, mP7
    if precision == 0:
        return [mP3[0], mP3[1],
                mP4[0], mP4[1],
                mP5[0], mP5[1],
                mP6[0], mP6[1],
                mP7[0], mP7[1]]
    return np.round([mP3[0], mP3[1],
                     mP4[0], mP4[1],
                     mP5[0], mP5[1],
                     mP6[0], mP6[1],
                     mP7[0], mP7[1]], decimals=precision)


def getAngleState(precision=3):
    global theta1, theta2, theta3, theta4, mP7
    if precision == 0:
        return [mP7[0], np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4)]
    return np.round([mP7[0], np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4)],
                    decimals=precision)


def getPlotAsImage():
    ax.set_xlim([-100, 100])
    ax.set_ylim([0, dJoint1 + dJoint2 + 5])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.figure(dpi=1200)

    canvas.draw()  # draw the canvas, cache the renderer

    width, height = fig.get_size_inches() * fig.get_dpi()

    image = np.fromstring(canvas.tostring_rgb(), dtype=np.uint8)

    image.resize(int(height), int(width), 3)

    return image


# SIM_MODE = int(input('SIM_MODE='))

fig = Figure()
canvas = FigureCanvas(fig)
ax = fig.subplots()

for theta1 in np.arange(t1Range[0], t1Range[1], simStep):
    for theta2 in np.arange(t2Range[0], t2Range[1], simStep):
        plotCount = 0
        for theta3 in np.arange(t3Range[0], t3Range[1], simStep):
            plt.clf()
            try:
                # theta3 = 0
                mP3 = vu.ang2DVector(mP1, theta1, dJoint1)
                mP4 = vu.ang2DVector(mP2, -theta2, dJoint1)
                mP5 = vu.ang2DVector(mP3, theta3, dJoint2)
                mP6 = vu.getMidPt(mP5, objLen, mP4, dJoint2)
                mP7 = vu.getCenterPt(mP5, mP6)
                # Success in theoretical calculations

                # Recalculating Theta 4
                theta4 = vu.getAngBtw(mP6 - mP4, mP4 - mP2)

                # Check for alignment with cone of friction
                if abs(vu.getAngBtw(mP6 - mP5, vu.getPerpVectorCW(mP5 - mP3))) > COF_ANGLE or \
                        abs(vu.getAngBtw(mP5 - mP6, vu.getPerpVectorCCW(mP6 - mP4))) > COF_ANGLE:
                    # print('!0', end='')
                    raise ArithmeticError('Cone of friction range unfulfilled')

                # Check for level scan fulfillment
                if SIM_MODE == 2 and (mP7[1] < scanLvl - tolerance or mP7[1] > scanLvl + tolerance):
                    # print('!1', end='')
                    raise ArithmeticError('Level requirement unfulfilled')

                # Check if theta4 is too extreme
                if abs(theta4) > np.pi / 2:
                    # print('!2', end='')
                    raise ArithmeticError('Theta 4 is unrealistic')

                # Preference test for concave holding

                # if prefConcave and vu.getAngBtw(mP3, mP5 - mP3) > np.pi and mP7[0] < 0:
                #     print('!3L', end='')
                #     raise ArithmeticError('Prefer concave [Left]')
                #
                # if prefConcave and vu.getAngBtw(mP4, mP6 - mP4) < np.pi and mP7[0] > 0:
                #     print('!3L', end='')
                #     raise ArithmeticError('Prefer concave [Right]')

                # if prefConcave and theta3 < 0 > mP7[0]:
                #     # print('!3L', end='')
                #     raise ArithmeticError('Prefer concave [Left]')
                #
                # if prefConcave and theta4 > 0 < mP7[0]:
                #     # print('!3L', end='')
                #     raise ArithmeticError('Prefer concave [Right]')

                if SIM_MODE == 2 and (vu.getAngBtw(mP5 - mP3, mP3 - mP1) < 0 or vu.getAngBtw(mP6 - mP4, mP4 - mP2) > 0):
                    raise ArithmeticError('Prefer concave')

                # Record max and min position of joint for level scanning
                if SIM_MODE == 2 and mP7[0] > jointLvlRefLocMax[8]:
                    jointLvlRefLocMax = getCurrentState(0)
                if SIM_MODE == 2 and mP7[0] < jointLvlRefLocMin[8]:
                    jointLvlRefLocMin = getCurrentState(0)

                # plotStickModel(getCurrentState(0), 2)

                scatterPts[0].append(mP7[0])
                scatterPts[1].append(mP7[1])

                if SIM_MODE >= 1:
                    savesPts.append(getCurrentState())
                if SIM_MODE == 2:
                    savesAngle.append(getAngleState())
                # print('>s')

                plotCount += 1

                # cv.imshow('Plot', getPlotAsImage())
                resp = cv.waitKey(10)
                if resp & 0xFF == ord('p'):
                    cv.waitKey(0)
                elif resp & 0xFF == ord('q'):
                    exit(0)

                ax.clear()

            except ArithmeticError as e:
                if VERBOSE:
                    print(f'Skip -> '
                          f'θ1={round(np.rad2deg(theta1), 3)}° '
                          f'θ2={round(np.rad2deg(theta2), 3)} '
                          f'θ3={round(np.rad2deg(theta3), 3)}°')
                continue

        # print('-', end='')
    if plotIndv and plotCount > 0:
        showPlot()
    # print('+')
    print(f'Progress -> [{int((theta1 - t1Range[0]) / (t1Range[1] - t1Range[0]) * 50) * "+"}'
          f'{(50 - int((theta1 - t1Range[0]) / (t1Range[1] - t1Range[0]) * 50)) * "-"}]\t'
          f'{int((theta1 - t1Range[0]) / (t1Range[1] - t1Range[0]) * 100)}%', end='\r')
print('DONE')

# ax.scatter(scatterPts[0], scatterPts[1])
# plt.show()

savesAngle = sorted(savesAngle, key=lambda x: x[0])
savesPts = sorted(savesPts, key=lambda x: x[8])

# Saving data as .csv
SESS_NAME = 'simRigGen'
if SIM_MODE == 2:
    SESS_NAME += '[Lvl]'

if SIM_MODE == 3:
    SESS_NAME += '[Lin]'

if saveData:
    try:
        fid = '%032x' % random.getrandbits(128)
        np.savetxt(f"./saves/{SESS_NAME}-Pos-{fid}.csv", np.asarray(savesPts), delimiter=",", fmt='%f')
        np.savetxt(f"./saves/{SESS_NAME}-Ang-{fid}.csv", np.asarray(savesAngle), delimiter=",", fmt='%f')
        print(f'Data successfully saved with SESS_NAME: {fid}')
        if SIM_MODE == 2: print(f'PARAM_STRING: lvl{scanLvl}-SA1\'{int(np.pi/simStep)}-({fid})')
    except IOError:
        print(f'Save failed with SESS_NAME: {fid}')

if SIM_MODE == 2:
    print('Replaying simulation. [p] to pause, [q] to quit')
    while True:
        for valPts, valAng in zip(savesPts, savesAngle):
            plotStickModel(valPts, style=2, ang=valAng)
            cv.imshow('Plot', getPlotAsImage())
            resp = cv.waitKey(10)
            if resp & 0xFF == ord('p'):
                cv.waitKey(0)
            elif resp & 0xFF == ord('q'):
                sys.exit(0)
            ax.clear()
        for valPts, valAng in zip(np.flip(savesPts, axis=0), np.flip(savesAngle, axis=0)):
            plotStickModel(valPts, style=2, ang=valAng)
            cv.imshow('Plot', getPlotAsImage())
            resp = cv.waitKey(10)
            if resp & 0xFF == ord('p'):
                cv.waitKey(0)
            elif resp & 0xFF == ord('q'):
                sys.exit(0)
            ax.clear()
