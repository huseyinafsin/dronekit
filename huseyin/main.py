from tokenize import String
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from numpy import take
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
from serial import Serial
from sympy import false, true
from tomlkit import boolean

# --------------------------------------------------------->
vehicle = connect('/dev/ttyUSB0', baud=57600)
vehicle.mode = VehicleMode("STABILIZE")
print("Mode:", vehicle.mode)
# --------------------------------------------------------->
irtifa_degeri = 0.8


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
            print("Waiting for vehicle to initialise...")
            vehicle.arm()
            time.sleep(1)
        while vehicle.armed:
            print(vehicle.mode)
            print("Arming motor")
            time.sleep(5)
            battery_check()
            kontrol_degeri = takeoff(irtifa_degeri)
            print("kontrol degeri--->", kontrol_degeri)
            if kontrol_degeri >= irtifa_degeri:
                print("you are aTargetAltitude")
                kontrol_degeri = takeoff(0.2)
                print("new aTargetAltitude--->", kontrol_degeri)
                print("lan modu geciliyor")
                land()
    except KeyboardInterrupt:
        print('arm-stopped')
        vehicle.mode = VehicleMode("LAND")
        print(vehicle.mode)
        vehicle.disarm()
        print("disarming")


def land():
    print("Vehicle in LAND mode")
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.location.global_relative_frame.alt == 0:
        if vehicle.location.global_relative_frame.alt < 2:
            set_velocity_body(vehicle, 0, 0, 0.1)
    vehicle.armed = False
    vehicle.close()


def about_battery():
    print("***********")
    print("About Battery: %s" % vehicle.battery)
    batterycheck = int(vehicle.battery.level)
    if batterycheck <= 25:
        print("Battery is loww!!! not started")
        vehicle.disarm()
        print("disarming")
    print("***********")


def battery_check():
    batterycheck = int(vehicle.battery.level)
    if batterycheck <= 25:
        print("Battery is loww!!! go home")
    else:
        print("Battery: %s" % vehicle.battery.level)


def takeoff(aTargetAltitude):
    try:
        vehicle.mode = VehicleMode("GUIDED")
        print(vehicle.mode)
        vehicle.armed = True
        vehicle.simple_takeoff(aTargetAltitude)
        while True:
            print(vehicle.location.global_relative_frame.alt, "---->", aTargetAltitude * 0.95)
            if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                if aTargetAltitude * 0.95 > 0:
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
print(" Type: %s" % vehicle._vehicle_type)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)
print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
time.sleep(1)
home = vehicle.location.global_relative_frame
about_battery()
arm()
