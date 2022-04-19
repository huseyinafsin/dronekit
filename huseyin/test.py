from tokenize import String
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from numpy import take
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
from serial import Serial
from sympy import false, true
from tomlkit import boolean
#--------------------------------------------------------->
vehicle=connect('/dev/ttyUSB0', baud=57600)
vehicle.mode = VehicleMode("STABILIZE")
print("Mode:",vehicle.mode)
#--------------------------------------------------------->
def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)


def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    currentLocation=vehicle.location.global_relative_frame  
    targetLocation=get_location_metres(currentLocation, dNorth, dEast)
    targetDistance=get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        print ("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print( "Reached target")
            break;
        time.sleep(2)


def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)
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
   
    except KeyboardInterrupt:
        print('arm-stopped')
        vehicle.mode = VehicleMode("LAND")
        print(vehicle.mode)
        vehicle.disarm()
        print("disarming")
    
def land():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.location.global_relative_frame.alt==0:
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle,0,0,0.1)
    vehicle.armed = False
    vehicle.close()
def about_battery():
    print("***********")
    print ("About Battery: %s" % vehicle.battery)
    batterycheck=float(vehicle.battery.voltage)
    print(batterycheck)
    if batterycheck <= 12.0:
        print("Battery is loww!!! not started")
        vehicle.disarm()
        print("disarming")
    print("***********")
def battery_check():
    batterycheck=int(vehicle.battery.level)
    if batterycheck <= 25:
        print("Battery is loww!!! go home")
    else:
        print ("Battery: %s" % vehicle.battery.level)

def takeoff(aTargetAltitude):
    try:
        vehicle.mode = VehicleMode("GUIDED")
        print(vehicle.mode)
        vehicle.armed = True
        vehicle.simple_takeoff(aTargetAltitude)
        while True:
                print(vehicle.location.global_relative_frame.alt,"---->",aTargetAltitude * 0.95)
                if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                    print("Reached target altitude")
                    if aTargetAltitude * 0.95>0:
                        print(vehicle.mode)
                        time.sleep(3)
                        return aTargetAltitude
                time.sleep(1)     
    except KeyboardInterrupt:
        print('takeoff-stopped')
        vehicle.mode = VehicleMode("LAND")
        print(vehicle.mode)
        vehicle.disarm()
############# POINTS ###############

# Display basic vehicle state
print(30*'$')
# print (" BATARYA DURUMU %s" % vehicle.battery.level)
print (" GPS DURUMU GLOBAL RELATIVE %s" % vehicle.location.global_relative_frame)
print (" GPS DURUMU GLOBAL %s" % vehicle.location.global_frame)
print (" GPS DURUMU LOCAL %s" % vehicle.location.local_frame)
print(30*'$')

print (" Type: %s" % vehicle._vehicle_type)
print (" System status: %s" % vehicle.system_status.state)
print (" GPS: %s" % vehicle.gps_0)
print (" Alt: %s" % vehicle.location.global_relative_frame.alt)
time.sleep(1)
home = vehicle.location.global_relative_frame
about_battery()  
arm()
#goto_position_target_global_int(5)