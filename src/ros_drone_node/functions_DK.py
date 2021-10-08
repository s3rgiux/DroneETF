# Dependencies
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command, APIException
from pymavlink import mavutil
import time
import socket
#import exceptions
import math
import argparse

# Functions
def connectCopter():
	"""Connect the vehicle to QGC

	Returns:
		vehicle (obejct): vehicle used
	"""

	global vehicle, sitl
	parser = argparse.ArgumentParser(description= 'commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect

	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()

	print("Connecting to vehicle on: %s" %connection_string)
	vehicle = connect(connection_string, wait_ready= True)

	return vehicle

def arm_and_takeoff(TargetAltitude):
	"""Arm, GUIDED mode and take off of the vehicle

	Args:
		TargetAltitude (double): Height vehicle would reach
	"""
	global vehicle
	while not vehicle.is_armable:
		print(" Waiting for vehicle to initialise...")
		time.sleep(1)

	print("Arming motors")
	vehicle.mode = VehicleMode("GUIDED")
	time.sleep(4)
	vehicle.armed = True

	while not vehicle.armed:
		print (" Waiting for arming...")
		time.sleep(1)

	print ("Taking off")
	vehicle.simple_takeoff(TargetAltitude)

	while True:
		print(" Altitude: %f" %vehicle.location.global_relative_frame.alt)
		if vehicle.location.global_relative_frame.alt >= TargetAltitude*0.95:
		    print("Reached target altitude")
		    break
		time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
    """
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
	
	Args:
		original_location (property): current locatio 
		dNorth (double): meters to North
		dEast (double): meters to East

	Returns:
		targetlocation (object): go-to location 
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles.

	Args:
		aLocation1 (object): First location
		aLocation2 (object): Second location

	Returns:
		distance (float): distance between the two points
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def goto_position_target_local_ned(north, east, down):
	"""
	Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

	Args:
		north (float): meters to move North
		east (float): meters to move East
		down (float): meters to move Down
	"""
	global vehicle
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
	vehicle.send_mavlink(msg)

def goto(dNorth, dEast, gotoFunction):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.

	Args:
		dNorth (float): meters to move North
		dEast (float): meters to move East
		gotoFunction (function): function for moving
    """
    global vehicle
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)

def set_roi(location):
	"""
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
    specified region of interest (LocationGlobal).
    The vehicle may also turn to face the ROI.

	Args:
		location (object): location to look at
	"""
	global vehicle
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
        0, #confirmation
        0, 0, 0, 0, #params 1-4
        location.lat,
        location.lon,
        location.alt
        )
	vehicle.send_mavlink(msg)

def condition_yaw(heading, relative=False):
	"""
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour

	Args:
		heading (float): yaw in degrees
		relative (boolean): is relative to the vehicle
    """
	global vehicle
	if relative:
		is_relative = 1 #yaw relative to direction of travel
	else:
		is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
	vehicle.send_mavlink(msg)
