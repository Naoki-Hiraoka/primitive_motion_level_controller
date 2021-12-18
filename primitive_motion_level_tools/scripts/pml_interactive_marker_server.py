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
#       cared: <bool> # default True
#  preview_time: <double> # default 1.0

# publish
#   ~command: primitive_motion_level_msgs/PrimitiveStateArray

# server
#   ~activate: std_srvs/SetBool

import copy

import rospy

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
import tf
from tf import TransformListener, transformations
from primitive_motion_level_msgs.msg import PrimitiveState, PrimitiveStateArray, PrimitiveStateArrayArray
from primitive_motion_level_tools.primitive_state import updatePrimitiveStateFromParam
from geometry_msgs.msg import Pose
from std_srvs.srv import SetBool, SetBoolResponse

tf_prefix = ""

class EndEffector:
    server = None
    menu_handler = None
    tfl = None
    int_marker = None
    state = None
    preview_support_com = None
    is_active = False
    is_cared = True
    cared_s = None

    def __init__(self, interactiveMarkerServer, tfListener, param):
        self.server = interactiveMarkerServer
        self.tfl = tfListener
        self.menu_handler = MenuHandler()

        self.state = PrimitiveState()
        updatePrimitiveStateFromParam(self.state,param)
        self.preview_support_com = copy.deepcopy(self.state.support_com)
        if "cared" in param:
            self.is_cared = param["cared"]

        self.int_marker = InteractiveMarker()
        self.int_marker.name = self.state.name
        self.int_marker.scale = 0.4

        self.cared_s = rospy.Service('~'+self.state.name+'_cared', SetBool, self.handle_cared)

    def handle_cared(self, req):
        if req.data == self.is_cared:
            return SetBoolResponse(True, "")
        self.is_cared = req.data
        if req.data:
            if self.is_active:
                self.start()
        else:
            if self.is_active:
                self.stop()
        return SetBoolResponse(True, "")

    def start(self):
        self.int_marker.description = self.int_marker.name;
        self.int_marker.header.frame_id = tf_prefix + "odom"

        try:
            (trans,rot) = self.tfl.lookupTransform(self.int_marker.header.frame_id, tf_prefix + self.state.parent_link_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("failed to lookup transform between "+self.int_marker.header.frame_id+" and "+tf_prefix + self.state.parent_link_name)
        scale, shear, angles, translation, persp = tf.transformations.decompose_matrix(tf.transformations.compose_matrix(translate=trans,angles=tf.transformations.euler_from_quaternion(rot)).dot(tf.transformations.compose_matrix(translate=[self.state.local_pose.position.x,self.state.local_pose.position.y,self.state.local_pose.position.z],angles=tf.transformations.euler_from_quaternion([self.state.local_pose.orientation.x,self.state.local_pose.orientation.y,self.state.local_pose.orientation.z,self.state.local_pose.orientation.w]))))
        self.state.pose.position.x = translation[0]
        self.state.pose.position.y = translation[1]
        self.state.pose.position.z = translation[2]
        self.state.pose.orientation.x, self.state.pose.orientation.y, self.state.pose.orientation.z, self.state.pose.orientation.w = tf.transformations.quaternion_from_euler(angles[0],angles[1],angles[2])
        self.int_marker.pose = copy.deepcopy(self.state.pose)

        self.int_marker.controls = []
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))

        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.int_marker.controls.append(copy.deepcopy(control))

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append( Marker() )
        control.markers[0].type = Marker.CUBE
        control.markers[0].scale.x = 0.08
        control.markers[0].scale.y = 0.08
        control.markers[0].scale.z = 0.08
        control.markers[0].color.r = 0.8
        control.markers[0].color.g = 0.8
        control.markers[0].color.b = 0.8
        control.markers[0].color.a = 0.5
        self.int_marker.controls.append(control)

        self.server.insert(self.int_marker, self.processFeedback)

        self.menu_handler = MenuHandler()
        entry = self.menu_handler.insert( "Support COM", callback=self.supportCOMCb )
        if self.state.support_com:
            self.menu_handler.setCheckState( entry, MenuHandler.CHECKED )
        else:
            self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED )
        entry = self.menu_handler.insert( "Preview Support COM", callback=self.previewSupportCOMCb )
        if self.preview_support_com:
            self.menu_handler.setCheckState( entry, MenuHandler.CHECKED )
        else:
            self.menu_handler.setCheckState( entry, MenuHandler.UNCHECKED )
        self.menu_handler.apply(self.server, self.state.name)

        self.server.applyChanges()

    def stop(self):
        self.server.erase(self.int_marker.name)
        self.server.applyChanges()

    def activate(self):
        if self.is_active:
            return True
        if self.is_cared:
            self.start()
        self.is_active = True

    def deactivate(self):
        if not self.is_active:
            return True
        if self.is_cared:
            self.stop()
        self.is_active = False

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.state.pose = copy.deepcopy(feedback.pose)

    def supportCOMCb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self.state.support_com = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self.state.support_com = True
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()

    def previewSupportCOMCb(self, feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            self.preview_support_com = False
        else:
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            self.preview_support_com = True
        self.menu_handler.reApply( self.server )
        self.server.applyChanges()

    def getState(self):
        return copy.deepcopy(self.state)
    def getPreviewState(self):
        previewState = copy.deepcopy(self.state)
        previewState.support_com = copy.deepcopy(self.preview_support_com)
        return previewState

if __name__ == "__main__":
    rospy.init_node("pml_interactive_marker_server")

    server = InteractiveMarkerServer("pml_interactive_marker_server")
    tfl = TransformListener()

    if rospy.has_param("~tf_prefix"):
        tf_prefix = rospy.get_param("~tf_prefix")
        if tf_prefix is not "":
            tf_prefix = "/" + tf_prefix + "/"
    else:
        tf_prefix = ""

    preview_time = 1.0
    if rospy.has_param("~preview_time"):
        preview_time = float(rospy.get_param("~preview_time"))
    end_effectors = []
    if rospy.has_param("~end_effectors"):
        end_effector_params = rospy.get_param("~end_effectors")
        for end_effector_param in end_effector_params:
            end_effectors.append(EndEffector(server,tfl,end_effector_param))

    pub = rospy.Publisher('~command', PrimitiveStateArray, queue_size=10)
    previewPub = rospy.Publisher('~previewCommand', PrimitiveStateArrayArray, queue_size=10)

    is_active = False
    def handle_activate(req):
        global is_active
        for end_effector in end_effectors:
            if req.data:
                end_effector.activate()
            else:
                end_effector.deactivate()
        is_active = req.data
        return SetBoolResponse(True, "")
    s = rospy.Service('~activate', SetBool, handle_activate)

    r = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            if is_active:
                msg = PrimitiveStateArray()
                for end_effector in end_effectors:
                    if end_effector.is_cared:
                        msg.primitive_state.append(end_effector.getState())
                pub.publish(msg)
                msg = PrimitiveStateArrayArray()
                primitive_states = PrimitiveStateArray()
                primitive_states.header.stamp = rospy.Time(preview_time)
                for end_effector in end_effectors:
                    if end_effector.is_cared:
                        primitive_states.primitive_state.append(end_effector.getPreviewState())
                msg.primitive_states.append(primitive_states)
                previewPub.publish(msg)
            r.sleep()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown('finish')
        rospy.spin()
