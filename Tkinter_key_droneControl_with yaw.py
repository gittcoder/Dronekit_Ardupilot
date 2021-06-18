import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time
from threading import Thread
import tkinter as tk



vehicle = connect('udp:127.0.0.1:14551')

#-- Set the  flying speed
gnd_speed = 5


def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%alt)
      if alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)

def condition_yaw(heading, relative=False):
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


      
def move(x,y,z,duration,yaw,d):
    if yaw:
        condition_yaw(d,False)
    else:
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

class Threader(Thread):
    def __init__(self,north,east,down,time,yaw,degrees)->None:
        Thread.__init__(self)
        self.north=north
        self.east=east
        self.down=down
        self.time=time
        self.yaw=yaw
        self.degrees=degrees
    def run(self) -> None:
        move(self.north,self.east,self.down,self.time,self.yaw,self.degrees)
        time.sleep(self.time+2.5)
   
    
def move_key(e):
    if e.char == e.keysym:
        if e.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
            
    else: 
        if e.keysym == 'Up':
            move(vehicle, gnd_speed, 0, 0,False,0)
        elif e.keysym == 'Down':
            move(vehicle,-gnd_speed, 0, 0,False,0)
        elif e.keysym == 'Left':
            move(vehicle, 0, -gnd_speed, 0,False,0)
        elif e.keysym == 'Right':
            move(vehicle, 0, gnd_speed, 0,False,0)
        elif e.keysym=='w':
            move(vehicle,0,0,-5,0,False,0)
        elif e.keysym=='s':
            move(vehicle,0,0,5,0,False,0)
        elif e.keysym=='a':
            move(vehicle,0,0,-5,0,True,10)
        elif e.keysym=='d':
            move(vehicle,0,0,-5,0,True,-10)
        elif e.keysym=='h':
            alt = vehicle.location.global_relative_frame.alt
            print(">> Altitude = %.1f m"%alt)
            
            
def main():
    arm_and_takeoff(10)
    r = tk.Tk()
    print(">> Control the drone with the arrow keys. Press r for RTL mode")
    r.bind_all('<Key>',move_key)
    r.mainloop()

if __name__=="__main__":
    main()