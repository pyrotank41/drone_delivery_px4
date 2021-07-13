#!/usr/bin/env python3

import asyncio

from lib.delivery_lib import *
from connection_param import *

async def mission():
    
    drone = await init_drone(connection_string, mavsdk_server_address=mav_server_address, port=mav_server_port)

    await drone.action.set_maximum_speed(maximum_land_speed) # m/sec
    
    await init_ros(camera_topic=ros_camera_topic)

    await print_drone_info(drone)
    
    await arm_drone(drone)
    # await simple_takeoff_and_land(drone, alt=2)
    await asyncio.sleep(1)

    await disarm_drone(drone)

    await close_ros()
    print("-- Done")



if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(mission())
