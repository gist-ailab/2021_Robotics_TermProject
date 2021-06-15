#!/usr/bin/env python

#----------------------------------------------------------------------------------
# Title: "UWB two tags ROS publish Node"
# Affiliation: Institute of Integrated Technology (IIT), 
#              Center for Healthcare Robotics, GIST.
#----------------------------------------------------------------------------------

import paho.mqtt.client as mqtt
from binascii import hexlify, a2b_base64, unhexlify #, b64encode
import base64
import sys
import codecs

import json
import numpy as np
import math
from math import sin, cos, pi, sqrt, acos, asin # 

import binascii
import base64 

import time
from datetime import datetime

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import String
import std_msgs.msg

global_px = []
global_py = []
UWB_FRAME = "uwb_frame"


def on_message1(client, userdata, msg):
  global global_px, global_py
  strRead = str(msg.payload.decode())
  json_data = json.loads(strRead)
  JsonLocation = json.dumps(json_data, indent=4, sort_keys=True)
  to_python = json.loads(JsonLocation)
  data1=[]
  data2 = []
  px = []
  py = []
  pz = []
  posQ = []

  for i,j in to_python['position'].iteritems():
     data1.append ((i,j))
     data2.append(j)
     if i == 'x':
        px = j
     if i == 'y':
        py = j
     if i == 'z':
        pz = j
     if i == 'quality':
        posQ = j
        #print ("Tag1: %0.2f, %0.2f, %0.2f, %d" %(float(px), float(py), float(pz), int(posQ)))
      #   global_px.append(px)
      #   global_py.append(py)
      #   if len(global_px) < 100:
      #     print(len(global_px))   
      #   else:
      #     print(sum(global_px)/len(global_px))
      #     print(sum(global_py)/len(global_py))
      #     exit()
        loc_msg = PoseStamped()
        loc_msg.header.frame_id = UWB_FRAME
        loc_msg.header.stamp = rospy.Time.now()
        loc_msg.pose.position.x = float(px)
        loc_msg.pose.position.y = float(py)      
        pub_tag1.publish(loc_msg)
     

def on_message2(client, userdata, msg):
  strRead = str(msg.payload.decode())
  json_data = json.loads(strRead)
  JsonLocation = json.dumps(json_data, indent=4, sort_keys=True)
  to_python = json.loads(JsonLocation)
  data1=[]
  data2 = []
  px = []
  py = []
  pz = []
  posQ = []
  for i,j in to_python['position'].iteritems():
     data1.append ((i,j))
     data2.append(j)
     if i == 'x':
        px = j
     if i == 'y':
        py = j
     if i == 'z':
        pz = j
     if i == 'quality':
        posQ = j
        # print ("Tag2: %0.2f, %0.2f, %0.2f, %d" %(float(px), float(py), float(pz), int(posQ)) )    
        loc_msg = PoseStamped()
        loc_msg.header.frame_id = UWB_FRAME
        loc_msg.header.stamp = rospy.Time.now()
        loc_msg.pose.position.x = float(px)
        loc_msg.pose.position.y = float(py)      
        pub_tag2.publish(loc_msg)





if __name__=="__main__":
   try:
      rospy.init_node('UWB_two_tags_pub_node', anonymous=True)
      pub_tag1 = rospy.Publisher("UWBTag1", PoseStamped, queue_size=1)
      pub_tag2 = rospy.Publisher("UWBTag2", PoseStamped, queue_size=1)

      #---- Setup MQTT Client--------------  
      client = mqtt.Client()

      client.message_callback_add("dwm/node/0e97/uplink/location", on_message1) # tag1
      client.message_callback_add("dwm/node/4827/uplink/location", on_message2) # tag2

      client.connect("192.168.0.35",1883,60) #Dasan 513

      client.subscribe("dwm/node/#", 0)
      
      client.loop_forever()
      #---- Setup MQTT Client--------------  


      #----for calculate mean x, y about UWB---
      
   except KeyboardInterrupt:
      exit()