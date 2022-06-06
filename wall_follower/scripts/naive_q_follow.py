#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan

#regions gathered from subscriber to lidarscans 
currentRegions = [np.nan,np.nan]
hasData = 0

#Thresholds for regions
THRESH = {'forward': 0.75, \
        'rightclose': 0.7, \
        'rightfar':0.8}

#Callback function for lidar data subscriber
#Reads lidar data and catagorizes data to find current regions
def getLiData(liData):
    region = [0,0,0,0]
    liScans = np.array(list(liData.ranges))
    # rospy.loginfo(liScans.shape)
    #Seperate data into 4 even quadrants for forward/right/back/left/right
    for idx in range(0,4):
        regionData = []
        regionStart = (idx * len(liScans)/4) - (len(liScans)/8)
        # regionStart = (idx * len(liScans)/4) - (len(liScans)/16)
        #for the first quadrant the first li index will be negative
        if regionStart >=0:
            regionEnd = regionStart + len(liScans)/4
            # regionEnd = regionStart + len(liScans)/8
            regionData.append(liScans[regionStart:regionEnd])
        if regionStart < 0:
            regionStart += len(liScans)
            regionEnd = regionStart + len(liScans)/4 - len(liScans)
            # regionEnd = regionStart + len(liScans)/8 - len(liScans)
            regionData.append(liScans[regionStart:-1])
            np.append(regionData, liScans[0:regionEnd])
        
        #Take simple average of all scans in region
        region[idx] = np.min(regionData)

        # region[idx] = liScans[regionStart:regionEnd]
        
        # rospy.loginfo([regionStart , regionEnd])
        # rospy.loginfo(region[idx])
        # rospy.loginfo(liScans[regionStart])
    # rospy.loginfo('regionData: %s',np.array2string(np.array(region)))
    #Determain region based off of thresholds
    if region[0] < THRESH['forward']:
        currentRegions[0] = 0
    else:
        currentRegions[0] = 1

    if region[3] < THRESH['rightclose']:
        currentRegions[1] = 0
    elif region[3] < THRESH['rightfar']:
        currentRegions[1] = 1
    else:
        currentRegions[1] = 2
    # rospy.loginfo([region[0], region[3]])
    # rospy.loginfo('Current Regions: [%d, %d]', currentRegions[0],currentRegions[1])

def getState(regions):
    state = (1e0*(regions[0])) + \
            (1e1*(regions[1]))
    state = str(int(state))
    return state

def getAction(Qarr):
    cmd_vel = Pose2D()
    idx = np.argmax(Qarr)
    if idx == 0:
        cmd_vel.x = 0.3
        cmd_vel.y = 0
        cmd_vel.theta = 0
    elif idx == 2:
        cmd_vel.x = 0.2
        cmd_vel.y = 0
        cmd_vel.theta = -0.25
    else:
        cmd_vel.x = 0.2
        cmd_vel.y = 0
        cmd_vel.theta = 0.5
    return cmd_vel
def controlBot():
    while not rospy.is_shutdown():
        Q = {'0': [0, 1, 0], \
             '1': [0, 1, 0], \
             '10': [0, 1, 0], \
             '11': [1, 0, 0], \
             '20': [0, 1, 0], \
             '21': [0, 0, 1]}
        if(~np.isnan(currentRegions[0])):
            state = getState(currentRegions)
            # rospy.loginfo(state)
            # rospy.loginfo(Q[state])
            cmd_vel = getAction(Q[state])
            # rospy.loginfo(cmd_vel)
            pub.publish(cmd_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        #Setup Node and Publisher
        pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
        rospy.init_node('line_follower', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        rospy.Subscriber('/scan', LaserScan, getLiData)
        cmd_vel = Pose2D()
        cmd_vel.y = 0
        controlBot()
    except rospy.ROSInterruptException:
        pass