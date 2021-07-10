#!/usr/bin/env python3

import asyncio

from lib.delivery_lib import *

async def mission():
    
    drone = await init_drone("udp://:14540")
    # drone = await init_drone("serial:////dev/ttyACM0")
    await drone.action.set_maximum_speed(10) # m/sec
    
    await init_ros(camera_topic='/solo/pi_cam_v2/image_raw')

    await simple_takeoff_and_land(drone, alt=2)

    await close_ros()
    print("-- Done")



if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    loop.run_until_complete(mission())
