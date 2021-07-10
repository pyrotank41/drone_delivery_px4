#!/usr/bin/env python3

# ros imports
from mavsdk import mission_raw
from mavsdk.mission_pb2 import MissionPlan
from mavsdk.offboard_pb2 import AccelerationNed, VelocityNedYaw
from numpy.core.numeric import roll
import rospy
from sensor_msgs.msg import Image
import ros_numpy as rnp # to convert numpy array top ros msg and vise versa

# computer vision imports
import cv2
import cv2.aruco as aruco
import numpy as np

import asyncio
import math

from mavsdk import System
from mavsdk.offboard import (Attitude, OffboardError, PositionNedYaw, AccelerationNed)
from mavsdk.mission_raw import *

from pymavlink import mavutil

from aruco_tracker_lib import *


_alt = 0

_aruco_img_pub = None
_cam_topic_sub = None 
_global_position = None

_landing_lat = None
_landing_lon = None

_yaw_deg = None

_y_error_rad = _x_error_rad = 0
x_deg = y_deg = 0

_x_error = 0
_y_error = 0 

_altitude = 0

horizontal_len = 1280
virticle_len = 720

horizontal_fov = 62.2 # degrees
vertical_fov = 48.8 # degrees

id_marker_size = 40
id_to_find = 162

deg2rad = (math.pi/180)
rad2deg = 1/deg2rad

PRESITION_LAND = False
VEHICLE_MODE = ""

_camera_matrix = np.array([[1.28779477e+03, 0.00000000e+00, 6.59268320e+02],
                        [0.00000000e+00, 1.28791404e+03, 3.64676675e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
_camera_distortion = np.array([0.15085, 0.0317, -0.00131, 0.00199, -0.99343])
_aruco_tracker = ArucoTracker(_camera_matrix, _camera_distortion)


def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    y_uav =-y_cam
    x_uav = x_cam
    return(x_uav, y_uav)
    
def uav_to_ne(x_uav, y_uav, yaw_rad):
    c       = math.cos(yaw_rad)
    s       = math.sin(yaw_rad)
    
    north   = x_uav*c - y_uav*s
    east    = x_uav*s + y_uav*c 
    return(north, east)
    
def check_angle_descend(angle_x, angle_y, angle_desc):
    return(math.sqrt(angle_x**2 + angle_y**2) <= angle_desc)



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
    global _aruco_img_pub, PRESITION_LAND, _global_position, _landing_lat, _landing_lon, _x_error, _y_error, _altitude, _yaw_deg, _y_error_rad, _x_error_rad, x_deg, y_deg
    if _aruco_img_pub is not None:

        if PRESITION_LAND and _global_position is not None:
            np_img = rnp.numpify(message)
            marker_found, img, x_m, y_m, x_rad, y_rad, corners = _aruco_tracker.track(np_img, id_to_find, id_marker_size, _global_position.relative_altitude_m)

            if marker_found:
                # x_m, y_m =  camera_to_uav(x_m, y_m)
                z_m = _global_position.relative_altitude_m

                # north, east = uav_to_ne(x_m, y_m, _yaw_deg)
                
                # marker_lat, marker_lon = get_location_metres(_global_position.latitude_deg, _global_position.longitude_deg, north, east)
                # _landing_lon = marker_lat
                # _landing_lat = marker_lon

                _x_error = x_m 
                _y_error = y_m 

                _y_error_rad = y_rad
                _x_error_rad = x_rad

                _aruco_img_pub.publish(rnp.msgify(Image, img, encoding='rgb8'))


                # _altitude =  _global_position.relative_altitude_m - 0.3  


                # print(f"from callback {_x_error}, {_y_error}")
            else:
                _x_error = 0
                _y_error = 0
                _y_error_rad = 0
                _x_error_rad = 0
                x_deg = 0
                y_deg = 0
                
        


def subscribe_to_ros_topic(topic):
    global _cam_topic_sub
    try:
        rospy.init_node('drone_node',anonymous=False)
        _cam_topic_sub = rospy.Subscriber(topic, Image, camera_msg_callback)

    except rospy.ROSInterruptException:
        pass
        
def init_ros():
    global _aruco_img_pub
    #--- ROS topic subscribing and starting for camera feed and aruco output respectively
    print("-- Starting new topic to publish aruco camera stuff")
    _aruco_img_pub = rospy.Publisher('/landing/color/aruco_image', Image, queue_size=10)

    print("-- Subscribing to camera topic")
    subscribe_to_ros_topic('/solo/pi_cam_v2/image_raw')

def close_ros():
    global _cam_topic_sub
    _cam_topic_sub.unregister()

async def init_drone(system_address):
    global drone
    drone = System()
 
    #--- connecting to the drone 
    await drone.connect(system_address=system_address)

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

async def set_markers(pos):
    global id_to_find, id_marker_size
    if(pos < 5):
                id_to_find = 2
                id_marker_size = 15
            
    else:
        id_to_find = 162
        id_marker_size = 40


async def run():
    global _aruco_img_pub, _global_position, PRESITION_LAND, _x_error, _y_error, _altitude, _yaw_deg

    await init_drone("udp://:14540")
    init_ros()

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break      

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    #--- arming and taking off the drone
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("-- Arming")
            await drone.action.arm() 
        break

    take_off_altitude = 15 # meters

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            #-- taking off to a set altitude
            await drone.action.set_takeoff_altitude(take_off_altitude)
            print("-- Taking off")
            await drone.action.takeoff()
        break

    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= (take_off_altitude-0.5):
            print(f"reached takeoff altitude {position.absolute_altitude_m}")
            break    
    
    await asyncio.sleep(1)



    #--- precision landing
    PRESITION_LAND = True

    await drone.action.set_maximum_speed(0.05)
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -take_off_altitude, 0.0))

    print("-- Starting offboard mode")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        return
    count = take_off_altitude//2
    
    while True:

        async for position in drone.telemetry.position():
            _global_position = position
            break
        
        print(f"Current alt {_global_position.relative_altitude_m} {_global_position.absolute_altitude_m}")
        

        if _global_position.relative_altitude_m < 0.5:
            print("reached land altitude")
            PRESITION_LAND = False
            break 
        else:
            await set_markers(_global_position.relative_altitude_m)

            distance_threshold = 1 #m
            x = 0
            y = 0
            
            if(abs(_x_error) < distance_threshold): x = _x_error
            else: x = distance_threshold*(_x_error//abs(_x_error))
            
            if(abs(_y_error) < distance_threshold): y = _y_error
            else: y = distance_threshold*(_y_error//abs(_y_error))

            await drone.offboard.set_position_ned(PositionNedYaw(y, x, -(take_off_altitude) - count, 00.0))
            count -= 1
            await asyncio.sleep(2)
            
            
        
        

    print("-- landing")
    await drone.action.land()

    #--- waiting for disarm 
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            close_ros()
            print("-- mission completed")
            break

    

async def get_thrust(current_alt, desired_alt, verbose=False):

    error = current_alt - desired_alt
    print(error)
    # if error +ve reduce thrust
    # if error -ve increase thrust
    # simple p implementation of pid 
    thrust = 0.465 - error*0.3
    if thrust > 0.6: thrust = 0.6 
    elif thrust < 0.4: thrust = 0.4

    if verbose: print(f"Thrust is:{thrust}")
    return thrust

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
