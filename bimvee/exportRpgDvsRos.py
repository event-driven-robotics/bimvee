# -*- coding: utf-8 -*-
"""
Created on Thu Sep 24 11:29:15 2020

@author: sbamford
code contributions from Henri Rebecq
"""


import sys
sys.path.append('/usr/local/lib/python2.7/site-packages')
sys.path.append('/opt/ros/kinetic/lib')
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/home/sim/catkin_ws/devel/lib/python2.7/dist-packages')
import numpy as np
#import cv2
import rosbag
#from dvs_msgs.msg import Event, EventArray
#from sensor_msgs.msg import CameraInfo
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge #, CvBridgeError
import rospy
from tqdm import tqdm

def exportPose6q(inDict, topicName, bagFile):
    point = inDict['point']
    rotation = inDict['rotation']
    for poseIdx in tqdm(range(inDict['ts'].shape[0])):
        poseMsg = PoseStamped()
        ts = rospy.Time(secs=inDict['ts'][poseIdx])        
        poseMsg.header.stamp = ts
        poseMsg.pose.position.x = point[poseIdx, 0]
        poseMsg.pose.position.y = point[poseIdx, 1]
        poseMsg.pose.position.z = point[poseIdx, 2]
        poseMsg.pose.orientation.w = rotation[poseIdx, 0]
        poseMsg.pose.orientation.x = rotation[poseIdx, 1]
        poseMsg.pose.orientation.y = rotation[poseIdx, 2]
        poseMsg.pose.orientation.z = rotation[poseIdx, 3]
        bagFile.write(topic=topicName, msg=poseMsg, t=ts)

    
def exportFrame(inDict, topicName, bagFile):
    bridge = CvBridge()
    for frameIdx in tqdm(range(inDict['ts'].shape[0])):
        img = inDict['frames'][frameIdx]
        assert img.dtype == np.uint8 # TODO: Handle type conversion here
        # For example by doing right shifts from 10-bit encoding to 8 bits: img = np.right_shift(img, 2)
        img = img.astype('uint8')
        ts = rospy.Time(secs=inDict['ts'][frameIdx])
        imgMsg = bridge.cv2_to_imgmsg(img, 'mono8')
        imgMsg.header.stamp = ts
        bagFile.write(topic=topicName, msg=imgMsg, t=ts)
    
def exportRpgDvsRos(inDict, **kwargs):
    # Open bag
    exportFilePathAndName = kwargs.get('exportFilePathAndName', './')
    bagFile = rosbag.Bag(exportFilePathAndName, 'w')

    #Descend data hierarchy, looking for dataTypes to export
    for channelName in inDict['data']:
        for dataTypeName in inDict['data'][channelName]:
            dataTypeDict = inDict['data'][channelName][dataTypeName]
            topicName = '/' + channelName + '/' + dataTypeName
            if dataTypeName == 'pose6q':
                # For now, we'll make an arbitrary decision which ros message type to use
                exportPose6q(dataTypeDict, topicName, bagFile)
            elif dataTypeName == 'frame':
                # For now, we'll make an arbitrary decision which ros message type to use
                exportFrame(dataTypeDict, topicName, bagFile)
            else:
                print('Skipping dataType "' + dataTypeName + '" from channel "' + channelName + '"')
    bagFile.close()
    
'''
Legacy code from AedatTools repo
#%% DVS
    
    # Put several events into an array in a single ros message, for efficiency     
    
    if 'polarity' in aedat['data'] \
        and ('dataTypes' not in aedat['info'] or 'polarity' in aedat['info']['dataTypes']): 
    legacy code by Henri Rebecq, from AedatTools repo
        countMsgs = 0
        numEventsPerArray = 25000 # Could be a parameter
        numEvents = aedat['data']['polarity']['numEvents']
        numArrays = - (- numEvents / numEventsPerArray) # The subtraction allows rounding up
    
        # Construct the event array object - a definition from rpg_dvs_ros
        # Use this repeatedly for each message        
        eventArrayObject = EventArray()
        # The following properties don't change
        eventArrayObject.width = 240 # HARDCODED CONSTANT - RESOLVE ON A RAINY DAY
        eventArrayObject.height = 180 # HARDCODED CONSTANT - RESOLVE ON A RAINY DAY
        # Use the following object array repeatedly to construct the contents 
        # of each ros message
        eventArray = np.empty(-(-numEventsPerArray), 'object')
        # Outer loop over arrays or ros messages
        for startPointer in range(0, numEvents, numEventsPerArray):         
            countMsgs = countMsgs + 1        
            print 'Writing event array message', countMsgs, 'of', numArrays, ' ...'
            endPointer = min(startPointer + numEventsPerArray, numEvents)            
            # Break the data vectors out of the dict for efficiency, 
            # but do this message by message to avoid memory problems
            arrayX        = aedat['data']['polarity']['x'][startPointer : endPointer] 
            arrayY        = aedat['data']['polarity']['y'][startPointer : endPointer] 
            arrayPolarity = aedat['data']['polarity']['polarity'][startPointer : endPointer]
            # Convert timestamps to seconds (ros, however, stores timestamps to ns precision)            
            arrayTimeStamp = aedat['data']['polarity']['timeStamp'][startPointer : endPointer]/1000000.0 

            # Iterate through all the events in the intended event array
            for eventIndex in range (0, endPointer - startPointer):
                # The Event object definition comes from rpg_dvs_ros
                e = Event()
                e.x = 239 - arrayX[eventIndex] # Flip X - I don't know why this is necessary
                e.y = arrayY[eventIndex]
                e.ts = rospy.Time(arrayTimeStamp[eventIndex])
                e.polarity = arrayPolarity[eventIndex]
                eventArray[eventIndex] = e;
            # The last array may be smaller than numEventsPerArray, so clip the object array
            if endPointer == numEvents:
                eventArray = eventArray[0 : endPointer - startPointer]
            # Assume that the ros message is sent at the time of the last event in the message
            eventArrayObject.header.stamp = e.ts
            eventArrayObject.events = eventArray
            bag.write(topic='/dvs/events', msg=eventArrayObject, t=e.ts)
            
    #%% IMU6
    
    # Put several events into an array in a single ros message, for efficiency     
    if 'imu6' in aedat['data'] \
        and ('dataTypes' not in aedat['info'] or 'imu6' in aedat['info']['dataTypes']): 
        # Break the IMU events out of the dict, for efficiency
        # Accel is imported as g; we want m/s^2
        arrayAccelX = aedat['data']['imu6']['accelX'] * 9.8
        arrayAccelY = aedat['data']['imu6']['accelY'] * 9.8
        arrayAccelZ = aedat['data']['imu6']['accelZ'] * 9.8
        # Angular velocity is imported as deg/s; we want rad/s
        arrayGyroX = aedat['data']['imu6']['gyroX'] * 0.01745
        arrayGyroY = aedat['data']['imu6']['gyroY'] * 0.01745
        arrayGyroZ = aedat['data']['imu6']['gyroZ'] * 0.01745
            # Convert timestamps to seconds (ros, however, stores timestamps to ns precision)            
        arrayTimeStamp = aedat['data']['imu6']['timeStamp']/1000000.0 
        numEvents = aedat['data']['imu6']['numEvents']
        # Use the following containers repeatedly during the export
        imuMsg = Imu()
        accel = Vector3()
        gyro = Vector3()
        # I guess these assignments only need to be made once
        imuMsg.linear_acceleration = accel
        imuMsg.angular_velocity = gyro
        for eventIndex in range(0, numEvents):         
            imuMsg.header.stamp = rospy.Time(arrayTimeStamp[eventIndex])            
            accel.x = arrayAccelX[eventIndex]
            accel.y = arrayAccelY[eventIndex]
            accel.z = arrayAccelZ[eventIndex]
            gyro.x = arrayGyroX[eventIndex]
            gyro.y = arrayGyroY[eventIndex]
            gyro.z = arrayGyroZ[eventIndex]
            bag.write(topic='/dvs/imu', msg=imuMsg, t=imuMsg.header.stamp)
    '''
