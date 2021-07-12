#!/usr/bin/env python3

import asyncio

from lib.delivery_lib import *
from connection_param import *

async def mission():
    
    drone = await init_drone(connection_string)
    # drone = await init_drone("serial:////dev/ttyACM0") # for connecting to actual hardware.
    await drone.action.set_maximum_speed(maximum_land_speed) # m/sec
    
    await init_ros(camera_topic=ros_camera_topic)

    await simple_takeoff_and_land(drone, alt=2)

    await close_ros()
    print("-- Done")



if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(mission())
