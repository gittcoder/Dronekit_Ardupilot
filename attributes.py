from dronekit import connect,VehicleMode,LocationGlobalRelative,LocationGlobal
import time
vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)

# ATTRIBUTES

# vehicle is an instance of the Vehicle class
print("Autopilot Firmware version: %s",vehicle.version)
print("Autopilot capabilities (supports ftp): %s",vehicle.capabilities.ftp)
print("Global Location: %s",vehicle.location.global_frame)
print("Global Location (relative altitude): %s",vehicle.location.global_relative_frame)
print("Local Location: %s",vehicle.location.local_frame)
print("Attitude: %s",vehicle.attitude)
print("Velocity: %s",vehicle.velocity)
print("GPS: %s",vehicle.gps_0)
print("Groundspeed: %s",vehicle.groundspeed)
print("Airspeed: %s",vehicle.airspeed)
print("Gimbal status: %s",vehicle.gimbal)
print("Battery: %s",vehicle.battery)
print("EKF OK?: %s",vehicle.ekf_ok)
print("Last Heartbeat: %s",vehicle.last_heartbeat)
print("Rangefinder: %s",vehicle.rangefinder)
print("Rangefinder distance: %s",vehicle.rangefinder.distance)
print("Rangefinder voltage: %s",vehicle.rangefinder.voltage)
print("Heading: %s",vehicle.heading)
print("Is Armable?: %s",vehicle.is_armable)
print("System status: %s",vehicle.system_status.state)
print("Mode: %s",vehicle.mode.name)
print("Armed: %s",+++vehicle.armed)
#disarm the vehicle
vehicle.armed = False

#set the default groundspeed to be used in movement commands
vehicle.groundspeed = 3.2

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name=='GUIDED' and not vehicle.armed :
    print(" Getting ready to take off ...")
    time.sleep(1)
#Point the gimbal straight down
vehicle.gimbal.rotate(-90, 0, 0)
time.sleep(10)

#Set the camera to track the current home position.
vehicle.gimbal.target_location(vehicle.home_location)
time.sleep(10)

# HOME LOCATION

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
print(" Home Location: %s",vehicle.home_location)

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")

# We have a home location.
print("\n Home location: %s",vehicle.home_location)

#PARAMETERS


# Print the value of the THR_MIN parameter.
print("Param: %s" % vehicle.parameters['THR_MIN'])


# Change the parameter value (Copter, Rover)
vehicle.parameters['THR_MIN']=100

print("\nPrint all parameters (iterate `vehicle.parameters`):")
for key, value in vehicle.parameters.iteritems():
    print(" Key:%s Value:%s" % (key,value))





























































































































































































