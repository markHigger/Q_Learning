#!/usr/bin/env python
# license removed for brevity

from tarfile import RECORDSIZE
import rospy
import rospkg
import numpy as np
import random
import math
import pickle
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

#regions gathered from subscriber to lidarscans 
currentRegions = [np.nan,np.nan,np.nan,np.nan, np.nan]
hasData = 0


#Thresholds for forward, right, left, angled (forward right), and crash-state regions
THRESH = {'ffar': 0.75, \
        'fveryfar': 1.3,\
        'rclose': 0.7, \
        'rfar':0.8, \
        'rveryfar': 1.3, \
        'lfar': 0.4, \
        'afar':0.9, \
        'abort': 0.14}

#Callback function for lidar data subscriber
#Reads lidar data and catagorizes data to find current regions
def getLiData(liData):
    region = [0,0,0,0,0]
    liScans = np.array(list(liData.ranges))
    # rospy.loginfo(liScans.shape)
    #Seperate data into 4 even quadrants for forward/right/back/left/right
    for idx in range(0,4):
        regionData = []
        rsize = (1.0/8.0)
        regionStart = int((idx * len(liScans)/4) - (len(liScans)*rsize/2))
        # regionStart = (idx * len(liScans)/4) - (len(liScans)/16)
        #for the first quadrant the first li index will be negative
        if regionStart >=0:
            regionEnd = int(regionStart + len(liScans)*rsize)
            # regionEnd = regionStart + len(liScans)/8
            regionData.append(liScans[regionStart:regionEnd])
        if regionStart < 0:
            regionStart += len(liScans)
            regionEnd = int(regionStart + len(liScans)*rsize - len(liScans))
            # regionStart += len(liScans)
            # regionEnd = regionStart + len(liScans)/4 - len(liScans)
            # regionEnd = regionStart + len(liScans)/8 - len(liScans)
            # rospy.loginfo([regionStart , regionEnd])
            regionData.append(liScans[regionStart:-1])
            np.append(regionData, liScans[0:regionEnd])
        
        #Take min of all scans in region
        
        region[idx] = np.min(regionData)

        # region[idx] = liScans[regionStart:regionEnd]
        
        # rospy.loginfo([regionStart , regionEnd])
        # rospy.loginfo(region[idx])
        # rospy.loginfo(liScans[regionStart])
    regionStart = int((7 * len(liScans)/8) - (len(liScans)*rsize/2))
    regionEnd = int(regionStart + len(liScans)*rsize)
    regionData = []

    regionData.append(liScans[regionStart:regionEnd])
    # rospy.loginfo([regionStart , regionEnd])
    
    region[4] = np.min(regionData)
    

    # rospy.loginfo('regionData: %s',np.array2string(np.array(region)))
    #Determain region based off of thresholds
    if region[0] < THRESH['ffar']:
        currentRegions[0] = 0
    elif region[0] < THRESH['fveryfar']:
        currentRegions[0] = 1
    else:
        currentRegions[0] = 2

    if region[3] < THRESH['rclose']:
        currentRegions[1] = 0
    elif region[3] < THRESH['rfar']:
        currentRegions[1] = 1
    elif region[3] < THRESH['rveryfar']:
        currentRegions[1] = 2
    else:
        currentRegions[1] = 3

    if region[1] < THRESH['lfar']:
        currentRegions[2] = 0
    else:
        currentRegions[2] = 1

    if region[4] < THRESH['afar']:
        currentRegions[3] = 0
    else:
        currentRegions[3] = 1

    if np.min(liScans) > THRESH['abort'] and not math.isinf(np.max(liScans)):
        # rospy.loginfo(np.min(liScans))
        currentRegions[4] = 0
    else:
        currentRegions[4] = 1
    
    # rospy.loginfo([region[0], region[3]])
    # rospy.loginfo('Current Regions: [%d, %d]', currentRegions[0],currentRegions[1])


def getState(regions):
    #set state to hitting wall if any lascans are too close
    if regions[4]:
        state = '-1'
    else:
        #f*1 + r*10 + l*100 + a*1000
        state = (1e0*regions[0]) + \
                (1e1*regions[1]) + \
                (1e2*regions[2]) + \
                (1e3*regions[3])
        state = str(int(state))
    return state

def getAction(Qarr,eps):
    cmd_vel = Pose2D()
    randval = random.random()

    if randval < eps:
        idx = random.randint(0,len(Qarr)-1)
    else:
        idx = np.argmax(Qarr)

    #Hard Left Turn
    if idx == 0:
        cmd_vel.x = 0.1
        cmd_vel.y = 0
        cmd_vel.theta = 0.6
    #soft left turn
    elif idx == 1:
        cmd_vel.x = 0.2
        cmd_vel.y = 0
        cmd_vel.theta = 0.15
    #straight
    elif idx == 2:
        cmd_vel.x = 0.3
        cmd_vel.y = 0
        cmd_vel.theta = 0
    #soft right turn
    elif idx == 3:
        cmd_vel.x = 0.2
        cmd_vel.y = 0
        cmd_vel.theta = -0.15
    #Hard right Turn
    if idx == 4:
        cmd_vel.x = 0.1
        cmd_vel.y = 0
        cmd_vel.theta = -0.6
    return cmd_vel, idx
def printQ(Q):
    printval = ''
    for key in sorted(list(map(int,(Q.keys())))):
        printval = printval + '\n' + str(key) + ': '
        vals = ['%.2f' % i for i in Q[str(key)]]
        printval = printval + str(vals)
    if not rospy.is_shutdown():
        rospy.loginfo(printval)
def saveQ(Q):
    filename = './Q_sarsa.pickle'
    with open(filename, 'wb') as handle:
        pickle.dump(Q, handle, protocol=pickle.HIGHEST_PROTOCOL)

def loadQ():
    rospack = rospkg.RosPack()
    pkgdir = rospack.get_path('wall_follower')
    filename = pkgdir + '/Q_sarsa.pickle'
    with open(filename, 'rb') as handle:
        return pickle.load(handle)

def controlBot():

    while not rospy.is_shutdown():
        modelPos = ModelState()
        modelPos.model_name = 'triton_lidar'
        Q = loadQ()
        eps = 0
        epIdx = 1
        ittIdx = 0
        nextState = '-1'
        while(~np.isnan(currentRegions[0])):
            Q = loadQ()
            # Get state at k
            if nextState == '-1':
                state = getState(currentRegions)
            else:
                state = nextState

            # add state to Q and R matrix if it does not exist
            if not Q.has_key(state):
                Q[state] = [0, 0, 0, 0, 0]
                rospy.loginfo('Unidentified state')
            
            #get action at k
            
            rospy.loginfo(nextState)
            if nextState == '-1':
                [cmd_vel, action] = getAction(Q[state],eps)
            else:
                action = nextAction

            #Terminate when bot is too close or 5 min has passed
            if state == '-1':
                rospy.loginfo('Resetting Model')
                #Generate random start point/angle
                xstart = random.uniform(-3.5,3.5)
                ystart = random.uniform(-3.5,3.5)
                thetastart = random.uniform(0,6.2)
                # xstart = 0
                # ystart = 0
                # thetastart=0
                modelPos.pose.position.x = xstart
                modelPos.pose.position.y = ystart
                modelPos.pose.orientation.z = thetastart
                rospy.wait_for_service('/gazebo/set_model_state')
                try:
                    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                    resp = set_state(modelPos)
                except rospy.ServiceException:
                    print("Service call failed")
                epIdx = epIdx + 1
                ittIdx = 0
                rate.sleep()
                continue


            # rospy.loginfo(state)
            # rospy.loginfo(Q[state])


            # rospy.loginfo(cmd_vel)
            pub.publish(cmd_vel)
            ittIdx = ittIdx + 1
            rate.sleep()

            #Learn
            #get state at k+1
            nextState = getState(currentRegions)
            if not Q.has_key(nextState):
                Q[nextState] = [0, 0, 0, 0, 0]
                rospy.loginfo('Unidentified state')

            [cmd_vel, nextAction] = getAction(Q[nextState],eps)

            printQ(Q)
            rospy.loginfo("\n state: %s, eps: %f, action: %d, Ep: %d", state, eps, action, epIdx)

if __name__ == '__main__':
    try:
        #Setup Node and Publisher
        pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
        rospy.init_node('line_follower', anonymous=True)
        rate = rospy.Rate(3) # 3hz
        rospy.Subscriber('/scan', LaserScan, getLiData)
        cmd_vel = Pose2D()
        cmd_vel.y = 0
        controlBot()
    except rospy.ROSInterruptException:
        pass