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
         np.array([[1,0,0,0], [0,1,0,0], [0,0,1,1.65], [0,0,0,1]]),
         np.array([[1,0,0,0], [0,1,0,0], [0,0,1,2.65], [0,0,0,1]])]
Slist = [np.array([[0], [1], [0], [0], [0], [0]]),
         np.array([[0], [0], [1], [0], [0], [0]]),
         np.array([[0], [1], [0], [-1.05], [0], [0]]),
         np.array([[0], [1], [0], [-1.65], [0], [0]]),
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
               exp6(Slist[2], thetaArr[2]),
               np.matmul(exp6(Slist[3], thetaArr[3]), exp6(Slist[4], thetaArr[4]))]

    # initialized to the 4x4 indentity matrix
    poe = I4
    for i in range(len(poeList)):
        # if i > 0:
        poe = np.matmul(poe, poeList[i])
        # else:
        #     # Slist[2:] are not independent
        #     S4 = Slist[4].copy()
        #     S4[3] =
        temp = np.matmul(poe, Mlist[i])
        ret.append((temp[0][3], temp[1][3], temp[2][3]))

    return ret

t = np.linspace(0, 1, 1000)

# we assume that the base of the arm is at (0, 0, 0)

# plots the initial position of the arm
initPnts = simulate([0 for _ in range(5)])
print("initPnts:", initPnts, "\n")

# specifies the rotation of each joint in radians (from -pi/2 to pi/2)
shoulderJoint = np.pi/12
shoulderRotation = 0
elbowJoint = np.pi/12
wristJoint = np.pi/12
wristRotation = np.pi/12

simPnts = simulate([shoulderJoint, shoulderRotation, elbowJoint, wristJoint, wristRotation])
print("simPnts:", simPnts, "\n")

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

print("\n")
print("shoulderJoint:\n", exp6(Slist[0], shoulderJoint), "\n")
print("shoulderRotation:\n", exp6(Slist[1], shoulderRotation), "\n")
print("matmul:\n", np.matmul(exp6(Slist[0], shoulderJoint), exp6(Slist[1], shoulderRotation)), "\n")

print("test:\n", np.matmul(exp6(Slist[1], shoulderRotation), np.array([[1,0,0,1], [0,1,0,0], [0,0,1,1.05], [0,0,0,1]])), "\n")

# M0 = np.array([[1,0,0,3], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
# M1 = np.array([[1,0,0,2], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
# M2 = np.array([[1,0,0,1], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
# testSlist = [np.array([[0], [0], [1], [0], [0], [0]]),
#              np.array([[0], [0], [0], [1], [0], [0]]),
#              np.array([[0], [0], [1], [0], [-2], [0]])]
#
# result0 = np.matmul(np.matmul(np.matmul(exp6(testSlist[0], np.pi/6), exp6(testSlist[1], 0.5)), exp6(testSlist[2], np.pi/4)), M0)
# result1 = np.matmul(np.matmul(exp6(testSlist[0], np.pi/6), exp6(testSlist[1], 0.5)), M1)
# result2 = np.matmul(exp6(testSlist[0], np.pi/6), M2)
# print("result0:\n", result0, "\n")
# print("result1:\n", result1, "\n")
# print("result2:\n", result2, "\n")











