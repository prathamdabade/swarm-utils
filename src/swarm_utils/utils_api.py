# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import State
from math import radians, cos, sin
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.srv import CommandBool, CommandTOL, CommandBoolRequest, CommandTOLRequest, SetMode, SetModeRequest

class utils_api():
    def __init__(self):
        self.current_waypoint = PoseStamped()
        self.ns = rospy.get_namespace()
        self.current_state = State()
        #State Subscriber
        self.state_sub = rospy.Subscriber("{}mavros/state".format(self.ns),State,queue_size=10,callback=self.state_cb)

        #Arming Client
        rospy.wait_for_service("{}mavros/cmd/arming".format(self.ns))
        self.arming_client = rospy.ServiceProxy("{}mavros/cmd/arming".format(self.ns), service_class=CommandBool)

        #Setmode Client
        rospy.wait_for_service("{}mavros/set_mode".format(self.ns))
        self.setmode_client = rospy.ServiceProxy("{}mavros/set_mode".format(self.ns), service_class=SetMode)

        #Takeoff Client
        rospy.wait_for_service("{}mavros/cmd/takeoff".format(self.ns))
        self.takeoff_client = rospy.ServiceProxy("{}mavros/cmd/takeoff".format(self.ns), service_class=CommandTOL)

        #Landing Client
        rospy.wait_for_service("{}mavros/cmd/land".format(self.ns))
        self.land_client = rospy.ServiceProxy("{}mavros/cmd/land".format(self.ns), service_class=CommandTOL)

        #Local pose publisher
        self.local_pos_pub = rospy.Publisher('{}mavros/setpoint_position/local'.format(self.ns), PoseStamped, queue_size=10)
    #Check for FCU connection
    def connect(self):
        rospy.loginfo('waiting for FCU connection...')
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state.connected:
                rospy.loginfo('FCU connection established')
                return 0
            else:
                rospy.logerr('Error in connecting to the FCU')
                return -1

    #State Callback
    def state_cb(self,msg):
        self.current_state = msg

    #change mode
    def set_mode(self,mode):
        setmode_srv = SetModeRequest(0,mode)
        response = self.setmode_client(setmode_srv)
        if response.mode_sent:
            rospy.loginfo('Set mode {}'.format(mode))
            return 0
        else:
            rospy.logerr('failed to set mode')
            return -1

    #Arm function
    def arm(self):
        rospy.loginfo('Arming....')
        arm_req = CommandBoolRequest(True)
        while not rospy.is_shutdown() and not self.current_state.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_req)
        else:
            if response.success:
                rospy.loginfo('Armed Successfully')
                return 0
            else:
                rospy.logerr('Failed to arm')
                return -1

    #Takeoff function
    def takeoff(self,altitude):
        self.arm()
        takeoff_srv = CommandTOLRequest(0,0,0,0, altitude)
        response = self.takeoff_client(takeoff_srv)
        rospy.sleep(1)
        if response.success:
            rospy.loginfo('Takeoff successful')
            return 0
        else:
            rospy.logerr('Takeoff failed')
            return -1

    #Landing Function
    def land(self):
        land_srv = CommandTOLRequest(0,0,0,0,0)
        response = self.land_client(land_srv)
        if response.success:
            rospy.loginfo('Landing initiated')
            return 0
        else:
            rospy.logerr('Failed to land')
            return -1

    # waypoint and heading
    def set_heading(self, phi):
        phi = radians(phi)
        qw = cos(phi)
        qx = 0
        qy = 0
        qz = sin(phi)
        self.current_waypoint.pose.orientation = Quaternion(qx,qy,qz,qw)
        self.local_pos_pub(self.current_waypoint)
