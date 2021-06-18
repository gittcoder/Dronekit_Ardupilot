from logging import FATAL
from dronekit import connect,VehicleMode,LocationGlobalRelative,LocationGlobal
import time
from pymavlink import mavutil
from threading import Thread
import pygame
key=""
vehicle=connect("udp:127.0.0.1:14550",wait_ready=True)
alti=50

def move(x,y,z,duration):
    msg=vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0,0,0,
        x,y,z,
        0,0,0,
        0,0
    )
    for i in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    return True

def arm_and_takeoff(altitude):
    print("basic checkup")
    while not vehicle.is_armable:
        print("waiting to initialize")
        time.sleep(1)
    print("arming motors")
    vehicle.mode=VehicleMode("GUIDED")
    vehicle.armed=True

    while not vehicle.armed:
        print("waiting to be armed")
        time.sleep(1)
    print(vehicle.home_location)
    print("taking off")
    vehicle.simple_takeoff(altitude)

    while True:
        print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
        
        if vehicle.location.global_relative_frame.alt>=altitude*0.95:
            print("target altitude reached")
            break
        time.sleep(1)
    vehicle.airspeed = 15 
    vehicle.groundspeed = 15 



def travel():
    alti=50
    north=10 #velocity for y axis
    east=10 #velocity for x axis
    down=0 # velcity for z axis
    count=0 # duration for the message to be sent
    while True:
        key=input()
        if(key=="W" or key=='w'):
            move(10,0,0,1)
            time.sleep(1)
        elif(key=="S" or key=="s"):
            move(-10,0,0,1)
            time.sleep(1)
        elif(key=="A" or key=="a"):
            move(0,-10,0,1)
            
            time.sleep(1)
        elif(key=="D" or key=="d"):
            move(0,10,0,1)
            time.sleep(1)
        elif(key=="L" or key=="l") :
            if(vehicle.location.global_relative_frame.alt>=20.0):
                move(0,0,10,1)
                alti-=20
                time.sleep(10)
                print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
        elif(key=="H"or key=="h"):
            move(0,0,-10,1)
            time.sleep(10)
            alti+=20
            print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
        elif(key=="E" or key=="e"):
            print("Returning to home location....\n")
            location=LocationGlobalRelative(-35.36333366, 149.16527792, 100)
            vehicle.airspeed = 15
            vehicle.groundspeed = 7.5 
            vehicle.simple_goto(location, groundspeed=10)
            time.sleep(20)
            print("Landing.....\n")
            move(0,0,alti,1)
            time.sleep(20)
            print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
            print("Exiting.....")
            break
        else:
            print("\nInvalid Input !!\n")

    



def main():
    arm_and_takeoff(50)
    travel()

    

if __name__=="__main__":
    main()