import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# identity matrices
I3 = np.array([[1,0,0], [0,1,0], [0,0,1]])
I4 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])

# note that the unit of length used is centimeters
Mlist = [np.array([[1,0,0,0], [0,1,0,0], [0,0,1,1.05], [0,0,0,1]]),
         np.array([[1,0,0,0], [0,1,0,0], [0,0,1,2.65], [0,0,0,1]])]
Slist = [np.array([[0], [0], [1], [0], [0], [0]]),
         np.array([[0], [1], [0], [0], [0], [0]]),
         np.array([[0], [1], [0], [-1.05], [0], [0]]),
         np.array([[0], [0], [1], [0], [0], [0]])]

def bracket(arr):
    return np.array([[0, -arr[2,0], arr[1,0]],
                     [arr[2,0], 0, -arr[0,0]],
                     [-arr[1,0], arr[0,0], 0]])

def exp3(arr, theta):
    brackArr = bracket(arr)
    return I3 + np.sin(theta)*brackArr + (1-np.cos(theta))*(np.matmul(brackArr, brackArr))

def exp6(arr, theta):
    sw = arr[0:3]
    sv = arr[3:]
    swBrack = bracket(sw)
    exp3Arr = exp3(sw, theta)
    svArr = np.matmul((theta*I3 + (1-np.cos(theta))*swBrack + (theta-np.sin(theta))*(np.matmul(swBrack, swBrack))), sv)
    return np.array([[exp3Arr[0,0], exp3Arr[0,1], exp3Arr[0,2], svArr[0,0]],
                     [exp3Arr[1,0], exp3Arr[1,1], exp3Arr[1,2], svArr[1,0]],
                     [exp3Arr[2,0], exp3Arr[2,1], exp3Arr[2,2], svArr[2,0]],
                     [0, 0, 0, 1]])

def simulate(thetaArr):
    ret = [(0,0,0)]
    poeList = [np.matmul(exp6(Slist[0], thetaArr[0]), exp6(Slist[1], thetaArr[1])),
               np.matmul(exp6(Slist[2], thetaArr[2]), exp6(Slist[3], thetaArr[3]))]

    # initialized to the 4x4 indentity matrix
    poe = I4
    for i in range(len(poeList)):
        poe = np.matmul(poe, poeList[i])
        temp = np.matmul(poe, Mlist[i])
        ret.append((temp[0][3], temp[1][3], temp[2][3]))

    return ret

t = np.linspace(0, 1, 1000)

# we assume that the base of the arm is at (0, 0, 0)

# plots the initial position of the arm
initPnts = simulate([0 for _ in range(5)])
# print("initPnts:", initPnts, "\n")

# specifies the rotation of each joint in radians (from -pi/2 to pi/2)
shoulderRotation = np.pi/12
shoulderJoint = np.pi/12
elbowJoint = np.pi/12
wristRotation = np.pi/12

simPnts = simulate([shoulderRotation, shoulderJoint, elbowJoint, wristRotation])
# print("simPnts:", simPnts, "\n")

# plots the initial and final positions of the arm
for i in range(1, len(initPnts)):
    ax.plot(initPnts[i-1][0] + t*(initPnts[i][0]-initPnts[i-1][0]),
            initPnts[i-1][1] + t*(initPnts[i][1]-initPnts[i-1][1]),
            initPnts[i-1][2] + t*(initPnts[i][2]-initPnts[i-1][2]))
    ax.plot(simPnts[i-1][0] + t*(simPnts[i][0]-simPnts[i-1][0]),
            simPnts[i-1][1] + t*(simPnts[i][1]-simPnts[i-1][1]),
            simPnts[i-1][2] + t*(simPnts[i][2]-simPnts[i-1][2]))
    print("i: " + str(i) + ", dist from i to i-1: " +
          str(np.sqrt((simPnts[i][0]-simPnts[i-1][0])**2 + (simPnts[i][1]-simPnts[i-1][1])**2 +
                        (simPnts[i][2]-simPnts[i-1][2])**2)))

plt.show()











