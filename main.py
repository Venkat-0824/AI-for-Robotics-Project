import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np
import sys

######################################################### Simulation Setup ############################################################################
table_path =  'data/data/table2/table.urdf'

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.stepSimulation()
planeId = p.loadURDF("plane.urdf", [0, 0, -1])
# p.loadURDF("plane.urdf",[0,0,-.98])
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("data/data/sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot
tableId = p.loadURDF(table_path, [1.1, 0., -1], p.getQuaternionFromEuler([0, 0, 1.56]))  # load table

######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# center of table is at [1.4,0, -1], adjust postion of object to put it on the table
# objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5
testing_object = sys.argv[1]
xpos = 1.10
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])
# orn = p.getQuaternionFromEuler([2*ang, 2*ang, 0])
# load your assigned random object 
random_object_path = '/data/data/random_urdfs/082/082.urdf'
# load your assigned object
object_path = '/data/data/126_5-16-ball-end-hex.urdf'

if testing_object == 'random_object':
    random_objectId = p.loadURDF(random_object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])
if testing_object == 'object':
    objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])





###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]
def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])


######################################################### detected information ##########################################################################
'''
19 b'palm'
20 b'rb1'
21 b'finger1.1a'
22 b'finger1.1b'
23 b'finger1.1c'
24 b'fingertip1'
25 b'rb2'
26 b'finger1.2a'
27 b'finger1.2b'
28 b'finger1.2c'
29 b'rb3'
30 b'finger2.1a'
31 b'finger2.1b'
32 b'finger2.1c'
33 b'fingertip2'
34 b'rb4'
35 b'finger2.2a'
36 b'finger2.2b'
37 b'finger2.2c'
38 b'rb5'
39 b'finger3.1a'
40 b'finger3.1b'
41 b'finger3.1c'
42 b'fingertip3'
43 b'rb6'
44 b'finger3.2a'
45 b'finger3.2b'
46 b'finger3.2c'
47 b'rb7'
48 b'finger4.1a'
49 b'finger4.1b'
50 b'finger4.1c'
51 b'fingertip4'
52 b'rb8'
53 b'finger4.2a'
54 b'finger4.2b'
55 b'finger4.2c'
56 b'rb9'
57 b'rb10'
58 b'thumb1'
59 b'thumb2'
60 b'rb11'
61 b'thumb3.1'
62 b'thumbtip'
63 b'rb12'
64 b'thumb3.2'
hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
[0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

'''


def info():
    palmContact = []
    thumbContact = []
    indexContact = []
    midContact = []
    ringContact = []
    pinkyContact = []
    palmLinks = [19, 20, 25, 29, 34, 38, 43, 47, 52, 56, 57]
    thumbLinks = [58, 59, 60, 61, 62, 63, 64]
    indexLinks = [48, 49, 50, 51, 53, 54, 55]
    middleLinks = [39, 40, 41, 42, 44, 45, 46]
    ringLinks = [30, 31, 32, 33, 35, 36, 37]
    pinkyLinks = [21, 22, 23, 24, 26, 27, 28]

    contact = p.getContactPoints(sawyerId, objectId)  # pubullet quick guide
    nums = len(contact)
    if (nums == 0):
        print("There are no contact points")
        return [], [], [], [], [], []

    for i in range(nums):
        temp = []
        if (contact[i][3] in palmLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            palmContact.append(temp)

        if (contact[i][3] in thumbLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            thumbContact.append(temp)

        if (contact[i][3] in indexLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            indexContact.append(temp)

        if (contact[i][3] in middleLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            midContact.append(temp)

        if (contact[i][3] in ringLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            ringContact.append(temp)

        if (contact[i][3] in pinkyLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            pinkyContact.append(temp)

    return palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact


######################################################### Simulation ##########################################################################
currentP = [0] * 65
k = 0

##################################################################### Input Value Here ##########################################################################################################


handInitial =  [0.26121505230261155, 0.17934456879275282, 1.1146831961129855, 0.26493107969000046, 0.17907812933399364, 0.3545015109713303, 0.22064881672076855, 0.5670891755018503, 1.2468723794210015, 0.18128529903142357, 0.5851992296711084, 0.17014392502411396, 0.5872846661538109, 0.7724645929203886, 0.22290265600560122, 0.6284191039153812, 0.6579502400620709, 0.17077031419079114, 1.1386768699572838, 1.0894212302872914, 0.19751315819872878, 1.1428490439060197, 1.0377958048053253, 0.1714625969253612, 1.5689115835924374, 0.34084727936496095, 0.3406543690380435]
grasp_orientation = [1.571832769393921, 2.837588395277343, 0.8169429206848145]
grasp_palmPosition = [1.1, -0.135, -0.02]
handClose =   [0.9518602488025174, 0.9525470182946450, 1.5474074983539879, 0.9360509968123250, 0.9350156258164670, 0.22392050064941071, 0.59446326661024560, 0.5793631323758750, 1.16989640098739240, 0.5854447548559532, 0.5821846276001461, 0.17047160298302721, 0.8805923372517215, 0.3838697556762420, 0.22621806265375471, 0.9384265748909741, 0.2984993223313540, 0.17173215163837462, 1.2230835819966671, 0.22276218388972482, 0.17234180343293864, 1.2271037262924911, 0.24140473736653115, 0.17141295627521990, 1.5777646827634441, 0.5456018336264685, 0.5480414005648171]
pu_palmPosition =  [0.94, 0.0, 0.25]
pu_orientation  =  [1.2892775535683597, 2.827588395376452, 1.2237756253388918]
final_palmPosition = [0.9110235933478783, 0.17, 0.25]
final_orientation = [0.727097749711193, 2.2984782021237885, 0.4631795893178812]
handOpen = [0.23791776350578354, 0.1757544910936571, 0.29764745486837116, 0.23754362279773994, 0.16999597089686969, 0.3775939322409635, 0.18168474927512803, 0.1758926369336866, 0.1845598545607197, 0.17030196791446582, 0.17011452285795407, 0.1700102825769791, 0.1717451657212141, 0.17208485272719378, 0.17001139107791998, 0.17009669284461805, 0.17393682045571487, 0.16999999988999997, 0.170107095758437, 0.26215783072686536, 0.16999960893597742, 0.17000115527206383, 0.25230788515373766, 0.1700390287977374, 0.8499999370367525, 0.3401734466845466, 0.3400368734419835]
##################################################################################################################################################################################################
initial_palmPosition = [0.94, 0.0, 0.2]
initial_orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

initial_palmPosition = [initial_palmPosition[0]-0.1,initial_palmPosition[1]-0.05,initial_palmPosition[2]]
# open_hand = [0.17490632355314772, 0.17860926478782999, 0.22650570370961434, 0.1761960796906339, 0.16999998123489446, 0.24349545190199115, 0.17706970814364162, 0.1781320748502043, 0.21614202049281497, 0.1703162987263699, 0.1701053810095854, 0.1700094742145387, 0.17173149540753777, 0.1718829432142787, 0.17000876625470537, 0.17007884093167489, 0.1742054185106175, 0.16999999999999985, 0.1700353815401814, 0.18103022110163855, 0.16999559680906137, 0.169999450133347, 0.17043300417766252, 0.17008861424515928, 0.8510846113965783, 0.34063687465125625, 0.34047793205464855]
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=-150, cameraTargetPosition=[0.8,0.5,0.2])

p.setGravity(0, 0, -10)




##################################################################################################################################################################################################

startPos = [handInitial[24], handInitial[25], handInitial[18], handInitial[19], handInitial[12], handInitial[13],
            handInitial[6],
            handInitial[7], handInitial[0], handInitial[1]]

"""
final is the list of the fingers in this order:
0---> thumb low
1---> thumb middle
2---> index low
3---> index middle
4---> middle low
5---> middle middle
6---> ring low
7---> ring middle
8---> pinky low
9---> pinky middle

Same order for open hand.
"""
final = np.zeros(10)
for i in range(10):
    final[i] = startPos[i]


####################################################################Write the code for the simulation##############################################################################################################################

p.setGravity(0, 0, -10)

# move palm to initial postion
k = 0
while 1:
    k = k + 1

    # move palm to target postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]],
                         p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
        time.sleep(0.05)
        p.stepSimulation()

        if (i == 100):
            print('Robot reached initial position')
            break  
        
 



    # moving palm to target postion where the object located
    i = 0
    while 1:
        i += 1
        #p.stepSimulation()
        currentP = palmP([1.0, -0.005, -0.12],
                         p.getQuaternionFromEuler([1.4292775535583496, grasp_orientation[1], grasp_orientation[2]]))
        
        
       
                
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print(' Robot about grasp object ')
            break  
        #handclose after grasping the object
    i = 0
    while 1:
        i+=1
        thumb(1.5, 0.5)
        indexF(1.5, 1.57)
        midF(0.5, 1.57)
        ringF(0.5,1.57)
        pinkyF(0.9, 1.57)
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('robot grasped object suceefully')
            break 
        # picking up the object
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([pu_palmPosition[0], pu_palmPosition[1], pu_palmPosition[2]],
                         p.getQuaternionFromEuler([pu_orientation[0], pu_orientation[1], pu_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('picking up object')
            break
    time.sleep(12)
    print("Object grasped successfully five seconds")
    break
        

p.disconnect()
print("disconnected")







