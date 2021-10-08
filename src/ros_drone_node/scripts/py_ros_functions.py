# -*- coding: utf-8 -*-
from PrintColours import *
import rospy
import threading
import cv2
from cv2.cv2 import aruco
from math import atan2, pow, sqrt, degrees, radians, sin, cos
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Twist, TwistStamped, Vector3
from geographic_msgs.msg import GeoPose, GeoPoseStamped, GeoPoint
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, LaserScan, Image
from mavros_msgs.msg import State
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget
from mavros_msgs.srv import WaypointClear, WaypointClearRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import CommandLong, CommandLongRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from cv_bridge import CvBridge, CvBridgeError

"""Control Functions
	This module is designed to make high level control programming simple.
"""

class eft_api:
    def __init__(self):
        """This function is called at the beginning of a program and will start of the communication links to the FCU.
        """
        self.current_state_g = State()
        self.current_pose_g = Odometry()
        self.current_pose_gg = NavSatFix()
        self.correction_vector_g = Pose()
        self.correction_vector_gg = GeoPose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()
        self.waypoint_vg = PositionTarget()
        self.waypoint_gg = GeoPoseStamped()
        self.waypoint_vgg = GlobalPositionTarget()
        self.vel_g = TwistStamped()
        self.my_lock = threading.Lock()
        self.bridge = CvBridge()
        self.id_aruco = String()

        self.current_heading_g = 0.0
        self.local_offset_g = 0.0
        self.correction_heading_g = 0.0
        self.local_desired_heading_g = 0.0
        self.alt_offset = 0.0

        self.ns = rospy.get_namespace()
        if self.ns == "/":
            rospy.loginfo(CBLUE2 + "Using default namespace" + CEND)
        else:
            rospy.loginfo(CBLUE2 + "Using {} namespace".format(self.ns) + CEND)

        # Publishers
        self.local_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_position/local".format(self.ns),
            data_class=PoseStamped,
            queue_size=10,
        )

        self.local_vel_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_raw/local".format(self.ns),
            data_class=PositionTarget,
            queue_size=10,
        )

        self.global_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_position/global".format(self.ns),
            data_class=GeoPoseStamped,
            queue_size=10,
        )

        self.global_vel_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_raw/global".format(self.ns),
            data_class=GlobalPositionTarget,
            queue_size=10,
        )

        self.vel_pos_pub = rospy.Publisher(
            name="{}mavros/setpoint_velocity/cmd_vel".format(self.ns),
            data_class=TwistStamped,
            queue_size=10,
        )

        self.image_pub = rospy.Publisher(
            name="/detected_markers".format(self.ns),
            data_class=Image, 
            queue_size=1,
        )

        self.id_pub = rospy.Publisher(
            name="/arudo_ID", 
            data_class=String, 
            queue_size=1,
        )

        # Subscribers
        self.currentPos = rospy.Subscriber(
            name="{}mavros/global_position/local".format(self.ns),
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb,
        )

        self.currentGlobalPos = rospy.Subscriber(
            name="{}mavros/global_position/global".format(self.ns),
            data_class=NavSatFix,
            queue_size=10,
            callback=self.pose_global_cb,
        )

        self.state_sub = rospy.Subscriber(
            name="{}mavros/state".format(self.ns),
            data_class=State,
            queue_size=10,
            callback=self.state_cb,
        )

        self.laser_h = rospy.Subscriber(
            name="/spur/laser_h/scan",
            data_class=LaserScan,
            queue_size= 1,
            buff_size= 52428800,
            callback=self.laser_cb_h,
        )

        self.image_sub = rospy.Subscriber(
            name="/webcam/image_raw", 
            data_class=Image, 
            callback=self.aruco_cb
        )

        self.id_sub = rospy.Subscriber(
            name="/arudo_ID", 
            data_class=String, 
            queue_size=1,
            callback= self.id_cb
        )

        # Service Proxy
        rospy.wait_for_service("{}mavros/cmd/arming".format(self.ns))

        self.arming_client = rospy.ServiceProxy(
            name="{}mavros/cmd/arming".format(self.ns), service_class=CommandBool
        )

        rospy.wait_for_service("{}mavros/cmd/land".format(self.ns))

        self.land_client = rospy.ServiceProxy(
            name="{}mavros/cmd/land".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/cmd/takeoff".format(self.ns))

        self.takeoff_client = rospy.ServiceProxy(
            name="{}mavros/cmd/takeoff".format(self.ns), service_class=CommandTOL
        )

        rospy.wait_for_service("{}mavros/set_mode".format(self.ns))

        self.set_mode_client = rospy.ServiceProxy(
            name="{}mavros/set_mode".format(self.ns), service_class=SetMode
        )

        rospy.wait_for_service("{}mavros/cmd/command".format(self.ns))

        self.command_client = rospy.ServiceProxy(
            name="{}mavros/cmd/command".format(self.ns), service_class=CommandLong
        )

        rospy.wait_for_service("{}mavros/mission/clear".format(self.ns))

        self.clear_client = rospy.ServiceProxy(
            name="{}mavros/mission/clear".format(self.ns), service_class=WaypointClear
        )
        rospy.loginfo(CBOLD + CGREEN2 + "Initialization Complete." + CEND)

    def state_cb(self, message):
        self.current_state_g = message

    def pose_cb(self, msg):
        """Gets the raw pose of the drone and processes it for use in control.
        Args:
                msg (geometry_msgs/Pose): Raw pose of the drone.
        """
        self.current_pose_g = msg
        self.enu_2_local()

        q0, q1, q2, q3 = (
            self.current_pose_g.pose.pose.orientation.w,
            self.current_pose_g.pose.pose.orientation.x,
            self.current_pose_g.pose.pose.orientation.y,
            self.current_pose_g.pose.pose.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

        self.current_heading_g = degrees(psi) - self.local_offset_g

    def pose_global_cb(self, msg):
        """Gets the raw pose of the drone and processes it for use in control.
        Args:
                msg (geographic_msgs/GeoPose): Raw pose of the drone.
        """
        self.current_pose_gg = msg
        self.enu()
        '''
        q0, q1, q2, q3 = (
            self.current_pose_gg.orientation.w,
            self.current_pose_gg.orientation.x,
            self.current_pose_gg.orientation.y,
            self.current_pose_gg.orientation.z,
        )

        psi = atan2((2 * (q0 * q3 + q1 * q2)),
                    (1 - 2 * (pow(q2, 2) + pow(q3, 2))))
        
        self.current_heading_g = degrees(psi) - self.local_offset_g
        '''

    def enu_2_local(self):
        x, y, z = (
            self.current_pose_g.pose.pose.position.x,
            self.current_pose_g.pose.pose.position.y,
            self.current_pose_g.pose.pose.position.z,
        )

        current_pos_local = Point()

        current_pos_local.x = x * cos(radians((self.local_offset_g - 90))) - y * sin(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.y = x * sin(radians((self.local_offset_g - 90))) + y * cos(
            radians((self.local_offset_g - 90))
        )

        current_pos_local.z = z

        return current_pos_local
    
    def enu(self):
        lat, lon, alt = (
            self.current_pose_gg.latitude,
            self.current_pose_gg.longitude,
            self.current_pose_gg.altitude,
        )

        current_pos_global = GeoPoint()

        current_pos_global.latitude = lat
        current_pos_global.longitude = lon
        current_pos_global.altitude = alt

        return current_pos_global

    def get_current_heading(self):
        """Returns the current heading of the drone.
        Returns:
            Heading (Float): θ in is degrees.
        """
        return self.current_heading_g

    def get_current_location(self):
        """Returns the current position of the drone.
        Returns:
            Position (geometry_msgs.Point()): Returns position of type geometry_msgs.Point().
        """
        return self.enu_2_local()

    def get_current_global_location(self):
        """Returns the current position of the drone.
        Returns:
            Position (geographic_msgs.GeoPoint()): Returns position of type geographic_msgs.GeoPoint().
        """
        return self.enu()

    def land(self):
        """The function changes the mode of the drone to LAND.
        Returns:
                0 (int): LAND successful
                -1 (int): LAND unsuccessful.
        """
        srv_land = CommandTOLRequest(0, 0, 0, 0, 0)
        response = self.land_client(srv_land)
        if response.success:
            rospy.loginfo(
                CGREEN2 + "Land Sent {}".format(str(response.success)) + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Landing failed" + CEND)
            return -1

    def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.
        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo(CYELLOW2 + "Waiting for FCU connection" + CEND)
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo(CGREEN2 + "FCU connected" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error connecting to drone's FCU" + CEND)
                return -1

    def wait4start(self):
        """This function will hold the program until the user signals the FCU to mode enter GUIDED mode. This is typically done from a switch on the safety pilot's remote or from the Ground Control Station.
        Returns:
                0 (int): Mission started successfully.
                -1 (int): Failed to start mission.
        """
        rospy.loginfo(CYELLOW2 +
                      "Waiting for user to set mode to GUIDED" + CEND)
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo(
                    CGREEN2 + "Mode set to GUIDED. Starting Mission..." + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Error startting mission" + CEND)
                return -1

    def set_mode(self, mode):
        """This function changes the mode of the drone to a user specified mode. This takes the mode as a string. Ex. set_mode("GUIDED").
        Args:
                mode (String): Can be set to modes given in https://ardupilot.org/copter/docs/flight-modes.html
        Returns:
                0 (int): Mode Set successful.
                -1 (int): Mode Set unsuccessful.
        """
        SetMode_srv = SetModeRequest(0, mode)
        response = self.set_mode_client(SetMode_srv)
        if response.mode_sent:
            rospy.loginfo(CGREEN2 + "SetMode Was successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "SetMode has failed" + CEND)
            return -1

    def set_speed(self, speed_mps):
        """This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.
        Args:
                speed_mps (Float): Speed in m/s.
        Returns:
                0 (int): Speed set successful.
                -1 (int): Speed set unsuccessful.
        """
        speed_cmd = CommandLongRequest()
        speed_cmd.command = 178
        speed_cmd.param1 = 1
        speed_cmd.param2 = speed_mps
        speed_cmd.param3 = -1
        speed_cmd.param4 = 0

        rospy.loginfo(
            CBLUE2 + "Setting speed to {}m/s".format(str(speed_mps)) + CEND)
        response = self.command_client(speed_cmd)

        if response.success:
            rospy.loginfo(
                CGREEN2 + "Speed set successfully with code {}".format(str(response.success)) + CEND)
            rospy.loginfo(
                CGREEN2 + "Change Speed result was {}".format(str(response.result)) + CEND)
            return 0
        else:
            rospy.logerr(
                CRED2 + "Speed set failed with code {}".format(str(response.success)) + CEND)
            rospy.logerr(
                CRED2 + "Speed set result was {}".format(str(response.result)) + CEND)
            return -1

    def set_heading(self, heading):
        """This function is used to specify the drone's heading in the local reference frame. Psi is a counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                heading (Float): θ(degree) Heading angle of the drone.
        """
        self.local_desired_heading_g = heading
        heading = heading + self.correction_heading_g + self.local_offset_g

        rospy.loginfo("The desired heading is {}".format(
            self.local_desired_heading_g))

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)

        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)

        qw = cy * cr * cp + sy * sr * sp
        qx = cy * sr * cp - sy * cr * sp
        qy = cy * cr * sp + sy * sr * cp
        qz = sy * cr * cp - cy * sr * sp

        self.waypoint_g.pose.orientation = Quaternion(qx, qy, qz, qw)

    def set_destination(self, x, y, z, psi = None):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                x (Float): x(m) Distance with respect to your local frame.
                y (Float): y(m) Distance with respect to your local frame.
                z (Float): z(m) Distance with respect to your local frame.
                psi (Float): θ(degree) Heading angle of the drone.
        """
        if psi is None:
            psi = 90 - degrees(atan2(y, x))
            self.set_heading(-psi)
        else: 
            self.set_heading(-psi)

        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x

        y = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y

        z = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        rospy.loginfo("Destination set to x:{} y:{} z:{} origin frame".format(x, y, z))
        rospy.loginfo("Heading to {}".format(-psi))

        self.waypoint_g.pose.position = Point(x, y, z)

        self.local_pos_pub.publish(self.waypoint_g)

    def set_destination_raw(self, x, y, z, vel, psi = None):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                x (Float): x(m) Distance with respect to your local frame.
                y (Float): y(m) Distance with respect to your local frame.
                z (Float): z(m) Distance with respect to your local frame.
                psi (Float): θ(degree) Heading angle of the drone.
                vel (Floar): vel(m/s) Velocity of the drone
        """
        self.waypoint_vg.type_mask = self.waypoint_vg.IGNORE_AFX + self.waypoint_vg.IGNORE_AFY + \
            self.waypoint_vg.IGNORE_AFZ + self.waypoint_vg.IGNORE_YAW + self.waypoint_vg.IGNORE_YAW_RATE
        if psi is None:
            psi = 90 - degrees(atan2(y, x))
            self.set_heading(-psi)
        else: 
            self.set_heading(-psi)

        theta = radians((self.correction_heading_g + self.local_offset_g - 90))

        Xlocal = x * cos(theta) - y * sin(theta)
        Ylocal = x * sin(theta) + y * cos(theta)
        Zlocal = z

        x = Xlocal + self.correction_vector_g.position.x + self.local_offset_pose_g.x

        y = Ylocal + self.correction_vector_g.position.y + self.local_offset_pose_g.y

        z = Zlocal + self.correction_vector_g.position.z + self.local_offset_pose_g.z

        rospy.loginfo("Destination set to x:{} y:{} z:{} origin frame".format(x, y, z))
        rospy.loginfo("Heading to {}".format(-psi))

        self.waypoint_vg.position = Point(x, y, z)
        self.waypoint_vg.velocity = Vector3(vel, vel, vel)

        self.local_vel_pos_pub.publish(self.waypoint_vg)

    def get_alt_offset(self):
        return self.current_pose_gg.altitude

    def set_destination_global(self, lat, lon, alt, psi = None):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the global reference frame. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                lat (Float): Coordinates of latitude
                lon (Float): Coordinates of longitude
                alt (Float): relative altitude
                psi (Float): θ(degree) Heading angle of the drone.
        """
        x = lat - self.current_pose_gg.latitude
        y = lon - self.current_pose_gg.longitude

        if psi is None:
            psi = 90 - degrees(atan2(y, x))
            self.set_heading(-psi)
        else: 
            self.set_heading(-psi)

        alt_g = self.alt_offset + alt
        rospy.loginfo("Destination set to lat:{} lon:{} alt:{} origin frame".format(lat, lon, alt_g))
        rospy.loginfo("Heading to {}".format(-psi))

        self.waypoint_gg.pose.position = GeoPoint(lat, lon, alt_g)

        self.global_pos_pub.publish(self.waypoint_gg)

    def set_destination_global_raw(self, lat, lon, alt, vel, psi = None):
        """This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the global reference frame. Psi is counter clockwise rotation following the drone's reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone.
        Args:
                lat (Float): Coordinates of latitude
                lon (Float): Coordinates of longitude
                alt (Float): relative altitude
                psi (Float): θ(degree) Heading angle of the drone.
                vel (Floar): vel(m/s) Velocity of the drone
        """
        stamp = rospy.get_rostime()
        self.waypoint_vgg.type_mask = self.waypoint_vgg.IGNORE_AFX + self.waypoint_vgg.IGNORE_AFY + \
            self.waypoint_vgg.IGNORE_AFZ + self.waypoint_vgg.IGNORE_YAW + self.waypoint_vgg.IGNORE_YAW_RATE
        self.waypoint_vgg.header.stamp = stamp
        self.waypoint_vgg.coordinate_frame = self.waypoint_vgg.FRAME_GLOBAL_INT
        x = lat - self.current_pose_gg.latitude
        y = lon - self.current_pose_gg.longitude

        if psi is None:
            psi = 90 - degrees(atan2(y, x))
            self.set_heading(-psi)
        else: 
            self.set_heading(-psi)

        alt_g = self.alt_offset + alt
        rospy.loginfo("Destination set to lat:{} lon:{} alt:{} origin frame".format(lat, lon, alt_g))
        rospy.loginfo("Heading to {}".format(-psi))

        self.waypoint_vgg.latitude = lat
        self.waypoint_vgg.longitude = lon
        self.waypoint_vgg.altitude = alt_g
        self.waypoint_vgg.velocity = Vector3(vel, vel, vel)

        self.global_vel_pos_pub.publish(self.waypoint_vgg)

    def arm(self):
        """Arms the drone for takeoff.
        Returns:
                0 (int): Arming successful.
                -1 (int): Arming unsuccessful.
        """
        self.set_destination(0, 0, 0)
        self.alt_offset = self.get_alt_offset() - 19.5
        for _ in range(100):
            self.local_pos_pub.publish(self.waypoint_g)
            rospy.sleep(0.01)

        rospy.loginfo(CBLUE2 + "Arming Drone" + CEND)

        arm_request = CommandBoolRequest(True)

        while not rospy.is_shutdown() and not self.current_state_g.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_pos_pub.publish(self.waypoint_g)
        else:
            if response.success:
                rospy.loginfo(CGREEN2 + "Arming successful" + CEND)
                return 0
            else:
                rospy.logerr(CRED2 + "Arming failed" + CEND)
                return -1

    def takeoff(self, takeoff_alt):
        """The takeoff function will arm the drone and put the drone in a hover above the initial position.
        Args:
                takeoff_alt (Float): The altitude at which the drone should hover.
        Returns:
                0 (int): Takeoff successful.
                -1 (int): Takeoff unsuccessful.
        """
        self.arm()
        rospy.loginfo(CGREEN2 + "Taking off..." + CEND)
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, takeoff_alt)
        response = self.takeoff_client(takeoff_srv)
        rospy.sleep(10)
        if response.success:
            rospy.loginfo(CGREEN2 + "Takeoff successful" + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Takeoff failed" + CEND)
            return -1

    def initialize_local_frame(self):
        """This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to."""
        self.local_offset_g = 0.0

        for i in range(30):
            rospy.sleep(0.1)

            q0, q1, q2, q3 = (
                self.current_pose_g.pose.pose.orientation.w,
                self.current_pose_g.pose.pose.orientation.x,
                self.current_pose_g.pose.pose.orientation.y,
                self.current_pose_g.pose.pose.orientation.z,
            )

            psi = atan2((2 * (q0 * q3 + q1 * q2)),
                        (1 - 2 * (pow(q2, 2) + pow(q3, 2))))

            self.local_offset_g += degrees(psi)
            self.local_offset_pose_g.x += self.current_pose_g.pose.pose.position.x
            self.local_offset_pose_g.y += self.current_pose_g.pose.pose.position.y
            self.local_offset_pose_g.z += self.current_pose_g.pose.pose.position.z

        self.local_offset_pose_g.x /= 30.0
        self.local_offset_pose_g.y /= 30.0
        self.local_offset_pose_g.z /= 30.0
        self.local_offset_g /= 30.0

        rospy.loginfo(CBLUE2 + "Coordinate offset set" + CEND)
        rospy.loginfo(
            CGREEN2 + "The X-Axis is facing: {}".format(self.local_offset_g) + CEND)

    def check_waypoint_reached(self, pos_tol=0.5, head_tol=0.01):
        """This function checks if the waypoint is reached within given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.
        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.
        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """
        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(
            self.waypoint_g.pose.position.x - self.current_pose_g.pose.pose.position.x
        )
        dy = abs(
            self.waypoint_g.pose.position.y - self.current_pose_g.pose.pose.position.y
        )
        dz = abs(
            self.waypoint_g.pose.position.z - self.current_pose_g.pose.pose.position.z
        )

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
        rospy.loginfo("Distance left: {}".format(dMag))

        cosErr = cos(radians(self.current_heading_g)) - cos(
            radians(self.local_desired_heading_g)
        )

        sinErr = sin(radians(self.current_heading_g)) - sin(
            radians(self.local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return 1
        else:
            return 0

    def check_global_waypoint_reached(self, pos_tol=0.05, head_tol=0.01):
        """This function checks if the waypoint is reached within globally given tolerance and returns an int of 1 or 0. This function can be used to check when to request the next waypoint in the mission.
        Args:
                pos_tol (float, optional): Position tolerance under which the drone must be with respect to its position in space. Defaults to 0.3.
                head_tol (float, optional): Heading or angle tolerance under which the drone must be with respect to its orientation in space. Defaults to 0.01.
        Returns:
                1 (int): Waypoint reached successfully.
                0 (int): Failed to reach Waypoint.
        """
        self.global_pos_pub.publish(self.waypoint_gg)

        dx = abs(
            self.waypoint_gg.pose.position.latitude - self.current_pose_gg.latitude
        ) * 400
        dy = abs(
            self.waypoint_gg.pose.position.longitude - self.current_pose_gg.longitude
        ) * 400
        dz = abs(
            self.waypoint_gg.pose.position.altitude - self.current_pose_gg.altitude
        ) -19.5

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))
        rospy.loginfo("Distance left: {}".format(dMag))
        
        cosErr = cos(radians(self.current_heading_g)) - cos(
            radians(self.local_desired_heading_g)
        )

        sinErr = sin(radians(self.current_heading_g)) - sin(
            radians(self.local_desired_heading_g)
        )

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol: #and dHead < head_tol:
            return 1
        else:
            return 0
    
    def clear_waypoints(self):
        """The function clears the waypoint list.
        Returns:
                0 (int): Clear successful
                -1 (int): Clear unsuccessful.
        """
        srv_clear = WaypointClearRequest()
        response = self.clear_client(srv_clear)
        if response.success:
            rospy.loginfo(
                CGREEN2 + "Clear Sent {}".format(str(response.success)) + CEND)
            return 0
        else:
            rospy.logerr(CRED2 + "Clear failed" + CEND)
            return -1
    
    def set_velocity(self, vel_mps):
        """This function is used to change the speed of the vehicle in guided mode. It takes the speed in meters per second as a float as the input.
        Args:
                speed_mps (Float): Speed in m/s.
        """
        self.vel_g.twist.linear = Vector3(vel_mps, vel_mps, vel_mps)
        self.vel_pos_pub.publish(self.vel_g)
        rospy.loginfo("Velocity changed")

    def laser_cb_h(self, msg):
        #with self.my_lock:
        cr_scan = LaserScan()
        cr_scan = msg
        avoid_x = 0.0
        avoid_y = 0.0
        avoid = False
        
        for i in range(1, len(cr_scan.ranges)):
            d0 = 6
            k = 1
            if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 5:
                avoid = True
                rospy.loginfo("Distance to object: {}".format(cr_scan.ranges[i]))
                x = cos(cr_scan.angle_increment * i)
                y = sin(cr_scan.angle_increment * i)
                u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))

                avoid_x += (x*u)
                avoid_y += (y*u)

        # Getting the current_heading of the drone and converting it to radians.
        cr_heading = radians(self.get_current_heading())
        avoid_x = (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading))
        avoid_y = (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading))

        if avoid:
            dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))

            if dist > 3:
                avoid_x = (3 * (avoid_x/dist))
                avoid_y = (3 * (avoid_y/dist))

            cur_pose = Point()
            cur_g_pose = GeoPoint()
            # Getting the current location from the drone.
            cur_pose = self.get_current_location()
            cur_g_pose = self.get_current_global_location()
            avoid_goal =[3 + cur_pose.x, 3 + cur_pose.y, 1 + cur_pose.z]
            self.waypoint_gg.pose.position.altitude += 1
            # Sending the goal
            self.clear_waypoints()
            '''
            while True:
                self.set_destination_global(cur_g_pose.latitude, cur_g_pose.longitude, cur_g_pose.altitude - self.alt_offset - 19.5, 0)
                rospy.loginfo(CRED+ "Stopping collision" + CEND)
                if self.check_global_waypoint_reached():
                    break
            '''
            while True:
                self.set_destination(avoid_goal[0], avoid_goal[1], avoid_goal[2], 0)
                rospy.loginfo(CYELLOW + "Avoiding collision" + CEND)
                if self.check_waypoint_reached():
                    break
            self.laser_h.unregister()

    def detect_aruco(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        params = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = params)
        output = aruco.drawDetectedMarkers(img, corners, ids)
        return output, ids

    def aruco_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as x:
            print("")

        markers_img, ids_list = self.detect_aruco(cv_image)

        if ids_list is None:
            self.id_pub.publish(ids_list)
        else:
            ids_str = ''.join(str(x) for x in ids_list)
            self.id_pub.publish(ids_str)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(markers_img, "bgr8"))
        except CvBridgeError as x:
            print("")
    
    def id_cb(self, msg):
        self.id_aruco = msg.data