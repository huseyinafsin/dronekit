from ast import For
from tokenize import String
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from numpy import take
from pymavlink import mavutil  # Needed for command message definitions
import time
import math

from sympy import false, true
from tomlkit import boolean
#--------------------------------------------------------->
vehicle=connect('/dev/ttyUSB0', baud=57600)
vehicle.mode = VehicleMode("STABILIZE")
print("Mode:",vehicle.mode)

#--------------------------------------------------------->
irtifa_degeri=1
inis_degeri=0.4
home = vehicle.location.global_relative_frame
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,
    0, 0,
    mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000111111000111,  # -- BITMASK -> Consider only the velocities
    0, 0, 0,  # -- POSITION
    vx, vy, vz,  # -- VELOCITY
    0, 0, 0,  # -- ACCELERATIONS
    0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def arm():
    try:
        while not vehicle.armed:
            print( "Waiting for vehicle to initialise...")
            vehicle.arm()
            time.sleep(1)
        while  vehicle.armed:
            print(vehicle.mode)
            print( "Arming motor")
            time.sleep(5)
            battery_check()
            kontrol_degeri=takeoff(irtifa_degeri)
            print("kontrol degeri--->",kontrol_degeri)
            if kontrol_degeri>=irtifa_degeri:
                print("you are aTargetAltitude")
                print("Havada bekliyorum")
                vehicle.location.global_relative_frame.alt=irtifa_degeri
                print(vehicle.airspeed)
                vehicle.airspeed=10
                temp_land()
                vehicle.close()
                
    except KeyboardInterrupt:
        print('arm-stopped')
        vehicle.mode = VehicleMode("LAND")
        print(vehicle.mode)
        vehicle.disarm()
        print("disarming")
def goto_location(waypoint):
    vehicle.simple_goto(waypoint)
    time.sleep(2)
    reached = 0
    while(not reached):
        time.sleep(1)
        a = vehicle.velocity
        if (abs(a[1])< 0.2 and abs(a[2])< 0.2 and abs(a[0])< 0.2):
            reached = 1
    print ("Waypoint reached!")
def temp_land():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.location.global_relative_frame.alt==0:
        print("alt:",vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle,0,0,0.1)
        print ("Vehicle in AUTO mode")
    vehicle.mode = VehicleMode("AUTO")
def land():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.location.global_relative_frame.alt==0:
        print("land->mode",vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle,0,0,0.1)
    #vehicle.armed = False
    #vehicle.close()
def about_battery():
    print("*******************************")
    print ("About Battery: %s" % vehicle.battery)
    batterycheck=int(vehicle.battery.level)
    if batterycheck <= 25:
        print("Battery is loww!!! not started")
        vehicle.disarm()
        print("disarming")
    print("*****************************")
def battery_check():
    batterycheck=float(vehicle.battery.voltage)
    if batterycheck <= 12.0:
        print("Battery is loww!!! go home")
    else:
        print ("Battery: %s" % vehicle.battery.level)
def range_with_floats(start, stop, step):
    while stop > start:
        yield start
        start += step
def takeoff(aTargetAltitude):
    try:
        vehicle.airspeed=0.001
        vehicle.groundspeed=0.001
        vehicle.mode = VehicleMode("GUIDED")
        print("mod:",vehicle.mode)
        vehicle.armed = True
        vehicle.simple_takeoff(aTargetAltitude)
        vehicle.airspeed=0.0001
        vehicle.groundspeed=0.0001
        time.sleep(2)
        print("graoundspeed:",vehicle.groundspeed,"  airspeed:",vehicle.airspeed)
        for i in range_with_floats(0.00, 0.1, 0.01):
            print(i)
            vehicle.airspeed-=i
            vehicle.groundspeed-=i
            time.sleep(3)      
            print("graoundspeed:",vehicle.groundspeed,"  airspeed:",vehicle.airspeed)
        
        while True:
                print(vehicle.location.global_relative_frame.alt,"---->",aTargetAltitude * 0.95)
                if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                    print("Reached target altitude")
                    if aTargetAltitude * 0.95>0:
                        print("graoundspeed:",vehicle.groundspeed,"  airspeed:",vehicle.airspeed)
                        return aTargetAltitude
                time.sleep(1)     
    except KeyboardInterrupt:
        print('takeoff-stopped')
        vehicle.mode = VehicleMode("STABILIZE")
        print(vehicle.mode)
        vehicle.disarm()
############# POINTS ###############

print (" Type: %s" % vehicle._vehicle_type)
print (" System status: %s" % vehicle.system_status.state)
print (" GPS: %s" % vehicle.gps_0)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
if vehicle.location.global_relative_frame.alt is None:
    print("GPS is None")
else: 
    home = vehicle.location.global_relative_frame
    about_battery()  
    arm()
    
