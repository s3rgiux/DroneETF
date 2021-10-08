#! /usr/bin/env python
# Import ROS
import rospy
# Import threading control
import threading
# Import the API
from py_ros_functions import *
# To print colours
from PrintColours import *

# Create an object for the API
drone = eft_api()
to_alt = 7
speed = 0.1

def main():
    # Initializing ROS node.
    rospy.init_node("drone_EFT", anonymous=True)
    # Wait for FCU connection
    drone.wait4connect()
    # Set the drone to GUIDED mode
    drone.set_mode("GUIDED")
    # Wait for the mode
    drone.wait4start()
    # Create local reference frame
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 3m
    drone.takeoff(to_alt)
    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(3)

    # Several local waypoints
    '''
    goals = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
             [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
    i = 0

    while i < len(goals):
        drone.set_destination(
            x=goals[i][0], y=goals[i][1], z=goals[i][2], psi=goals[i][3])
        rate.sleep()
        if drone.check_waypoint_reached():
            i += 1
    '''

    # One global line
    goals = [-35.36294112, 149.16517736, to_alt]

    while True:
        #with drone.my_lock:
        drone.set_destination_global(goals[0], goals[1], goals[2])
        rate.sleep()
        rospy.loginfo(CBLUE + "Heading to target..." + CEND)
        if drone.check_global_waypoint_reached() and drone.id_aruco == "[0]":
            break
        '''
        drone.laser_h = rospy.Subscriber(
            name="/spur/laser_h/scan",
            data_class=LaserScan,
            queue_size=1,
            buff_size= 52428800,
            callback=drone.laser_cb_h)
        '''
    # Land
    drone.land()
    rospy.loginfo(CGREEN2 + "Target Detected... Landing now." + CEND)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()