#!/usr/bin/env python3

# ros imports
# from mavsdk.offboard_pb2 import AccelerationNed, VelocityNedYaw
# from numpy.core.numeric import roll

import rospy
from sensor_msgs.msg import Image
import ros_numpy as rnp # to convert numpy array top ros msg and vise versa

# computer vision imports
# import cv2
# import cv2.aruco as aruco
import numpy as np

import asyncio
import math

import time
from mavsdk import System
from mavsdk.offboard import (Attitude, OffboardError)

from lib.aruco_tracker_lib import *

# VARIABLES -----------------------------------------------------------------------------

# ros variables
_aruco_img_pub = None
_cam_topic_sub = None 

# drone position variables
_global_position = None
_yaw_deg = _pitch_deg = _roll_deg = 0

_y_error_rad = _x_error_rad = 0
_x_error_deg = _y_error_deg = 0

_x_error = 0
_y_error = 0 

_takeoff_altitude = 0

_previous_land_velocity_state = -1
_previous_land_velocity = 0

# conversiton veriable
deg2rad = math.pi/180
rad2deg = 1/deg2rad


# camera parameters
_camera_matrix = np.array([[1.28779477e+03, 0.00000000e+00, 6.59268320e+02],
                        [0.00000000e+00, 1.28791404e+03, 3.64676675e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
_camera_distortion = np.array([0.15085, 0.0317, -0.00131, 0.00199, -0.99343])
_aruco_tracker = ArucoTracker(_camera_matrix, _camera_distortion)

horizontal_len = 1280 # px
virticle_len = 720    # px

horizontal_fov = 62.2 # degrees
vertical_fov = 48.8 # degrees
 
id_marker_size = 40
id_to_find = 162

# mode variable
PRESITION_LAND = False

# Functions ---------------------------------------------------

def get_location_metres(original_lat, original_lon, dNorth, dEast):
    """
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth

    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_lat/180))
    
    # print ("dlat, dlon", dLat, dLon)

    #New position in decimal degrees
    newlat = original_lat + (dLat * 180/math.pi)
    newlon = original_lon + (dLon * 180/math.pi)
    return(newlat, newlon)

def camera_msg_callback(message):
    global PRESITION_LAND, _x_error, _y_error, _y_error_rad, _x_error_rad, _x_error_deg, _y_error_deg
    if _aruco_img_pub is not None:

        if PRESITION_LAND and _global_position is not None:
            np_img = rnp.numpify(message)
            marker_found, img, x_m, y_m, x_rad, y_rad, corners = _aruco_tracker.track(np_img, id_to_find, id_marker_size, _global_position.relative_altitude_m)

            if marker_found:

                _x_error, _y_error = x_m, y_m
                _x_error_rad, _y_error_rad = x_rad, y_rad
                _x_error_deg, _y_error_deg = x_rad*rad2deg,  y_rad*rad2deg

                _aruco_img_pub.publish(rnp.msgify(Image, img, encoding='rgb8'))

                # print(f"from callback {_x_error}, {_y_error}")
            else:
                _x_error, _y_error = 0, 0
                _x_error_rad, _y_error_rad = 0, 0
                if(get_altitude() < 0.5):
                    _x_error_deg, _y_error_deg = 0, 0
                
async def subscribe_to_ros_topic(topic):
    global _cam_topic_sub
    try:
        rospy.init_node('drone_node',anonymous=False)
        _cam_topic_sub = rospy.Subscriber(topic, Image, camera_msg_callback)

    except rospy.ROSInterruptException:
        pass
        
async def init_ros():
    global _aruco_img_pub
    #--- ROS topic subscribing and starting for camera feed and aruco output respectively
    print("-- Starting new topic to publish aruco camera stuff")
    _aruco_img_pub = rospy.Publisher('/landing/color/aruco_image', Image, queue_size=10)

    print("-- Subscribing to camera topic")
    await subscribe_to_ros_topic('/solo/pi_cam_v2/image_raw')

async def close_ros():
    global _cam_topic_sub
    _cam_topic_sub.unregister()

async def init_drone(system_address):
    
    drone = System()
 
    #--- connecting to the drone 
    await drone.connect(system_address=system_address)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break
    
    return drone

async def arm_drone(drone):
    #--- checking if armed, if not - arming and taking off the drone, else - skip.
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            await drone.action.arm() 
            print("-- Armed")
        break

async def disarm_drone(drone):
    #--- waiting for disarm 
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-- Disarmed")
            break

async def simple_takeoff(drone):
    takeoff_altitude = await get_takeoff_altitude() # meters

    #--- checking if already in air, it so, just go forward
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            #-- taking off to a set altitude
            await drone.action.set_takeoff_altitude(takeoff_altitude)
            await drone.action.takeoff()
            print("-- Taking off")
        break

    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= (takeoff_altitude-0.5):
            print(f"reached takeoff altitude {position.relative_altitude_m}")
            break 

async def get_takeoff_altitude():
    return _takeoff_altitude

async def calc_roll_pitch(x_error_deg, y_error_deg, drone_pitch, drone_roll):
    # TODO: use PID controller in future to calculate pitch and roll.

    # correcting the error angle as the drone is also pitching 
    # and rolling while correcting its position. 
    y_deg = round(y_error_deg - drone_pitch, 3)
    x_deg = round(x_error_deg - drone_roll, 3)
    
    # changing the angle threshold to reduce the swing/overshoot while landing.
    # TODO: use PID controller in future to get pitch and roll.
    if get_altitude() > 5: angle_threshold = 4
    else: angle_threshold = 2 

    pitch = 0
    roll = 0
    
    if(abs(x_deg) < angle_threshold): roll = x_deg
    else: roll = angle_threshold*(x_deg//abs(x_deg))
    
    if(abs(y_deg) < angle_threshold): pitch = y_deg
    else: pitch = angle_threshold*(y_deg//abs(y_deg))

    return roll, pitch

async def get_landing_velocity():
    global _previous_land_velocity_state, _previous_land_velocity
    current_alt = get_altitude()
            
    if    current_alt < 1: state = 1 # slow speed
    elif  current_alt < 5: state = 2 # medium speed
    else                 : state = 3 # fast speed

    if abs(_x_error_deg) > 5 or abs(_y_error_deg) > 5:
        return 0

    velocity = _previous_land_velocity
    if state is not _previous_land_velocity_state:
        if state == 1:
            print("--- Alt < 1m; landing speed is -0.2m/s")
            velocity = -0.3

        elif state == 2:
            print("--- Alt < 5m; landing speed is -0.6m/s")
            velocity = -0.6 

        else:
            print("--- Alt > 5m; landing speed is -1.0m/s")
            velocity = -1.0

        # storing current state    
        _previous_land_velocity_state = state
        _previous_land_velocity = velocity
    
    return velocity

def       get_altitude():
    return _global_position.relative_altitude_m

async def set_attitude(drone):
    global _yaw_deg, _pitch_deg, _roll_deg
    async for euler in drone.telemetry.attitude_euler():
           _yaw_deg = euler.yaw_deg
           _pitch_deg = euler.pitch_deg
           _roll_deg = euler.roll_deg
           break

async def set_position(drone):
    global _global_position
    async for position in drone.telemetry.position():
            _global_position = position
            break

async def set_markers():
    # this function sets the target marker based on the altitude
    global id_to_find, id_marker_size
    if(get_altitude() < 5):
        id_to_find = 2
        id_marker_size = 15
            
    else:
        id_to_find = 162
        id_marker_size = 40

async def set_offboard_mode(drone, state):
    if state:
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")

    else:
        print("-- Stopping offboard")
        try:
            await drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: {error._result.result}")

async def set_takeoff_altitude(alt):
    global _takeoff_altitude
    _takeoff_altitude = alt  

async def precision_land(drone, verbose=False):
    global PRESITION_LAND

    #--- precision landing
    PRESITION_LAND = True

    thrust = 0.50 #%
    Kp = 0.001
    Kd = 0.05

    #--- we need to use off board mode to controll the yaw, pitch roll 
    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 90, thrust))
    await set_offboard_mode(drone, True)

    #--- updating current positions and attitude.
    await set_position(drone)
    await set_attitude(drone)

    # recording time and velocity to calculate velocity later
    previous_time = time.time()
    previous_altitude = get_altitude()

    # recording delta velocity/ error for PID 
    previous_delta_velocity = 0 # m/sec    
    
    while True:
        # update position and attitude variables.
        await set_position(drone)
        await set_attitude(drone)
        landing_velocity = await get_landing_velocity()
        
        if get_altitude() < 0.5:
            print("-- reached simple land altitude")
            await set_offboard_mode(drone, True)
            PRESITION_LAND = False
            break 
        else:
            
            # TODO: need to add a way to change the large and small marker setype
            await set_markers()
            
            delta_time = time.time()   - previous_time
            delta_alt = get_altitude() - previous_altitude
            if verbose:
                print(f"delta_time: {delta_time}")
                print(f"delta_alt : {delta_alt}")
            descent_speed = round(delta_alt/delta_time, 3)

            
            previous_time = time.time()
            previous_altitude = get_altitude()
            delta_velocity = descent_speed - landing_velocity
            # if delta_velocity is -ve, descending faster
            # if delta_velocity is +ve, descending slower or going up
            
            # PID
            error = delta_velocity*Kp + (delta_velocity-previous_delta_velocity)*Kd
            previous_delta_velocity = delta_velocity
            thrust -= error

            # limiting the thrust as the value ranges from 0 to 1            
            if thrust > 1: thrust = 1
            elif thrust < 0: thrust = 0

            #
            roll, pitch = await calc_roll_pitch(_x_error_deg, _y_error_deg, _pitch_deg, _roll_deg)

            if verbose:
                print(f"landing_velocity: {descent_speed}")
                print(f"          thrust: {thrust}%")
                print(f"    error angles: {_x_error_rad*rad2deg}, {_y_error_rad*rad2deg}")
                print(f"--   att: {roll}, {pitch}")

            await drone.offboard.set_attitude(Attitude(roll, pitch, 90.0, thrust))
            await asyncio.sleep(0.05)

    print("-- landing")
    await drone.action.land()

async def run():
    

    # drone = await init_drone("udp://:14540")
    drone = await init_drone("serial:////dev/ttyACM0")
    
    await init_ros()

    await arm_drone(drone)
    
    await set_takeoff_altitude(20)

    await simple_takeoff(drone)

    await precision_land(drone)

    await close_ros()
    print("-- Mission Complete")



if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
