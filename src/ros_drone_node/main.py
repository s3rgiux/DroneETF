from functions_DK import *

# Main
sitl = None
vehicle = connectCopter()
vehicle.gimbal
arm_and_takeoff(5)

print("Set airspeed to 10m/s (max).")
vehicle.airspeed = 10

print("Going towards the first point...")
goto_position_target_local_ned(50, -50, -15)
time.sleep(10)

print("Going towards the second point...")
goto(0, 100, vehicle.simple_goto)

print("Going towards the third point...")
goto_position_target_local_ned(0, 0, -10)
time.sleep(15)

print("Landing...")
vehicle.mode = VehicleMode("LAND")
while True:
	if vehicle.location.global_relative_frame.alt < 1:
		print("Completed!")
		break
	time.sleep(1)

# Closure
vehicle.close()
if sitl is not None:
	sitl.stop()

