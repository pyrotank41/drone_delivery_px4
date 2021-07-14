#!/usr/bin/env python3

import asyncio
import connection_param as param

from lib.delivery_lib import *


delivery_lat = 47.39768958423579
delivery_lon = 8.545531845401872

async def mission():
    try:
        # initializing drone
        drone = await init_drone(system_address=param.connection_string)
        await drone.action.set_maximum_speed(param.maximum_speed_with_respect_to_land) # m/sec
        # initializing ros node (and aruco publisher) and subscribing to the camera topic
        await init_ros(camera_topic=param.ros_camera_topic)

        # test specific code...
        # deliver test long distance
        await deliver(drone, delivery_lat, delivery_lon, location_offset_meters=False, alt=5, verbose=True)
        print("-- Done")

    except Exception as e:
        print(e)

    await close_ros()

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(mission())
