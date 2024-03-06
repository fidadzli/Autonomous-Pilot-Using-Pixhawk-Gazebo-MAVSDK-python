from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions

import time
import math
import csv
import serial

global S0_array, S1_array, S2_array, S3_array

S0_array = []
S1_array = []
S2_array = []
S3_array = []
a=0
b=0
c=0
d=0
# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect('127.0.0.1:14550', wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At the time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on a 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


# Arm and take off to an altitude of 1 meter
arm_and_takeoff(1)


def condition_yaw(heading, relative=False):
    if relative:
        is_relative = 1  # yaw relative to the direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        heading,  # param 1, yaw in degrees
        0,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame)
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


vehicle.groundspeed = 1
print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
DURATION = 10  # Set duration for each segment.

# Define the Arduino's serial port and baud rate
arduino_port = '/dev/ttyACM0'  # Replace with your Arduino's port
baud_rate = 9600
values = []
# Open the serial connection
ser = serial.Serial(arduino_port, baud_rate)

# Create a CSV file and write the header
# Replace with your column headers


    # Read and write data until interrupted
for x in range(5):
        csv_file = open('sensor_values.csv', 'w')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['Value1', 'Value2', 'Value3', 'Value4'])
        # Read a line of data from the Arduino
        data = ser.readline().decode(encoding='UTF-8').strip()
        # Split the data into time and value
        values = data.split('x')

        if len(values) >= 4:
            # Write the data to the CSV file
            csv_writer.writerow([values[0], values[1], values[2], values[3]])
            # ser.close()
            csv_file.close()

            with open('sensor_values.csv', 'r') as file:
                reader = csv.reader(file)
                S0_array.clear()
                S1_array.clear()
                S2_array.clear()
                S3_array.clear()

                for row in reader:
                    S0_array.append(row[0])
                    S1_array.append(row[1])
                    S2_array.append(row[2])
                    S3_array.append(row[3])

                    if len(S0_array) > 0 and len(S1_array) > 0 and len(S2_array) > 0 and len(S3_array) > 0:
                        plato = 0
                        if S0_array[-1] > S1_array[-1] and S0_array[-1] > S2_array[-1] and S0_array[-1] > S3_array[-1]:
                            plato = 1
                        elif S1_array[-1] > S0_array[-1] and S1_array[-1] > S2_array[-1] and S1_array[-1] > S3_array[
                            -1]:
                            plato = 2
                        elif S2_array[-1] > S0_array[-1] and S2_array[-1] > S1_array[-1] and S2_array[-1] > S3_array[
                            -1]:
                            plato = 3
                        elif S3_array[-1] > S0_array[-1] and S3_array[-1] > S1_array[-1] and S3_array[-1] > S2_array[
                            -1]:
                            plato = 4
                        print(plato)

                        if plato == 1:
                            print('north')
                            send_ned_velocity(3, 0, 0, 3)
                            send_ned_velocity(0, 0, 0, DURATION)

                        elif plato == 2:
                            print('east')
                            send_ned_velocity(0, 3, 0, 3)
                            send_ned_velocity(0, 0, 0, DURATION)

                        elif plato == 3:
                            print('south')
                            send_ned_velocity(-3, 0, 0, 3)
                            send_ned_velocity(0, 0, 0, DURATION)

                        elif plato == 4:
                            print('west')
                            send_ned_velocity(0, -3, 0, 3)
                            send_ned_velocity(0, 0, 0, DURATION)
                        
                        elif plato == 0:
                            print('static')    
                            
            csv_file.close()       
                            
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")

vehicle.armed = False
# Close vehicle object before exiting the script
print("Close vehicle object")
vehicle.close()
