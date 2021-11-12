#!/usr/bin/env python

import rospy
from primitive_motion_level_msgs.msg import PrimitiveState, PrimitiveStateArray, PrimitiveStateArrayArray

#     - name: <name>
#       parent_link_name: <linkname for URDF>
#       local_pose: [x, y, z, x, y, z, w] # default 0,0,0,0,0,0,1
#       is_wrenchC_global: <bool> # default false
#       wrenchC: [<double>, <double>, <double>, <double>, <double>, <double>,
#                 <double>, <double>, <double>, <double>, <double>, <double> ...] # default []
#       wrenchld: [<double>,
#                  <double> ...] # default []
#       wrenchud: [<double>,
#                  <double> ...] # default []
#       pose_follow_gain: [<double>, <double>, <double>, <double>, <double>, <double>] # default 1.0
#       wrench_follow_gain: [<double>, <double>, <double>, <double>, <double>, <double>] # default 0.0
#       support_com: <bool> #default false
#       time: <double> # default 0.1
#       M: [x ,y, z, rx, ry, rz] # default 10, 10, 10, 5, 5, 5
#       D: [x ,y, z, rx, ry, rz] # default 200, 200, 200, 100, 100, 100
#       K: [x ,y, z, rx, ry, rz] # default 400, 400, 400, 200, 200, 200

def updatePrimitiveStateFromParam(primitiveState, param):
    primitiveState.name = param["name"]
    if primitiveState.name != "com":
        primitiveState.parent_link_name = param["parent_link_name"]
    else:
        primitiveState.parent_link_name = "com"
    if "local_pose" in param:
        primitiveState.local_pose.position.x = param["local_pose"][0]
        primitiveState.local_pose.position.y = param["local_pose"][1]
        primitiveState.local_pose.position.z = param["local_pose"][2]
        primitiveState.local_pose.orientation.x = param["local_pose"][3]
        primitiveState.local_pose.orientation.y = param["local_pose"][4]
        primitiveState.local_pose.orientation.z = param["local_pose"][5]
        primitiveState.local_pose.orientation.w = param["local_pose"][6]
    else:
        primitiveState.local_pose.position.x = 0.0
        primitiveState.local_pose.position.y = 0.0
        primitiveState.local_pose.position.z = 0.0
        primitiveState.local_pose.orientation.x = 0.0
        primitiveState.local_pose.orientation.y = 0.0
        primitiveState.local_pose.orientation.z = 0.0
        primitiveState.local_pose.orientation.w = 1.0
    if "support_com" in param:
        primitiveState.support_com = param["support_com"]
    else:
        primitiveState.support_com = False
    if "time" in param:
        primitiveState.time = param["time"]
    else:
        primitiveState.time = 0.1
    if "is_wrenchC_global" in param:
        primitiveState.is_wrenchC_global = param["is_wrenchC_global"]
    else:
        primitiveState.is_wrenchC_global = False
    if "wrenchC" in param:
        primitiveState.wrenchC = map(lambda x: float(x), param["wrenchC"])
    else:
        primitiveState.wrenchC = []
    if "wrenchld" in param:
        primitiveState.wrenchld = map(lambda x: float(x), param["wrenchld"])
    else:
        primitiveState.wrenchld = []
    if "wrenchud" in param:
        primitiveState.wrenchud = map(lambda x: float(x), param["wrenchud"])
    else:
        primitiveState.wrenchud = []
    if "pose_follow_gain" in param:
        primitiveState.pose_follow_gain = param["pose_follow_gain"]
    else:
        primitiveState.pose_follow_gain = [1.0]*6
    if "wrench_follow_gain" in param:
        primitiveState.wrench_follow_gain = param["wrench_follow_gain"]
    else:
        primitiveState.wrench_follow_gain = [0.0]*6
    if "M" in param:
        primitiveState.M = map(lambda x: float(x), param["M"])
    else:
        primitiveState.M = [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
    if "D" in param:
        primitiveState.D = map(lambda x: float(x), param["D"])
    else:
        primitiveState.D = [200.0, 200.0, 200.0, 100.0, 100.0, 100.0]
    if "K" in param:
        primitiveState.K = map(lambda x: float(x), param["K"])
    else:
        primitiveState.K = [400.0, 400.0, 400.0, 200.0, 200.0, 200.0]

