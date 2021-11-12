#!/usr/bin/env python

# rosparams
#   end_effectors:
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

# publish
#   ~output: primitive_motion_level_msgs/PrimitiveStateArray (latch)

import rospy

from primitive_motion_level_msgs.msg import PrimitiveState, PrimitiveStateArray
from primitive_motion_level_tools.primitive_state import updatePrimitiveStateFromParam

if __name__ == "__main__":
    rospy.init_node("primitive_state_publisher")

    latch = False
    if rospy.has_param("~latch"):
        latch = bool(rospy.get_param("~latch"))

    pub = rospy.Publisher('~output', PrimitiveStateArray, queue_size=1, latch=latch)
    msg = PrimitiveStateArray()

    if rospy.has_param("~end_effectors"):
        end_effector_params = rospy.get_param("~end_effectors")
        for end_effector_param in end_effector_params:
            state = PrimitiveState()
            updatePrimitiveStateFromParam(state,end_effector_param)
            msg.primitive_state.append(state)

    if latch:
        pub.publish(msg)
        rospy.spin()
    else:
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()

