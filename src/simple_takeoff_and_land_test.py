#!/usr/bin/env python3

import asyncio
import connection_param as param

from lib.delivery_lib import *


async def mission():
    
    try:
        # initializing drone
        drone = await init_drone(system_address=param.connection_string)
        await drone.action.set_maximum_speed(param.maximum_speed_with_respect_to_land) # m/sec
        # initializing ros node (and aruco publisher) and subscribing to the camera topic
        await init_ros(camera_topic=param.ros_camera_topic)

        # test specific code...
        await simple_takeoff_and_land(drone, alt=1)
        print("-- Done")

    except Exception as e:
        print(e)

    await close_ros()

if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(mission())
