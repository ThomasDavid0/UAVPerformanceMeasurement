#!/home/td6834/mambaforge/envs/dint/bin/python
'''This script is used to run the autotuner in sitl. It will take off, and then autotune each channel in turn'''
#sh run_simulation.sh -p ./autotuner.py
from droneinterface import Vehicle, mavlink, enable_logging, logger, AwaitCondition
from droneinterface.messages import RCOverride
import geometry as g
from time import sleep


enable_logging('INFO')

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, timeout=1, retries=20)

vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)

vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_TAKEOFF)

home: g.GPS = vehicle.get_HomePosition(None, None).home

vehicle.wait_for_test(lambda : (vehicle.next_GlobalPositionInt(None).position - home).z[0] < -49 )
logger.info('takeoff complete')

for channel in [0, 1, 3]:
    vehicle.set_mode(mavlink.PLANE_MODE_GUIDED)
    vehicle.nav_waypoint(1, home.offset(g.PZ(-100)))
    mi = vehicle.next_MissionItemReached(None)

    logger.info(f'Autotune channel {channel}')
    vehicle.set_mode(mavlink.PLANE_MODE_AUTOTUNE)
    
    ac = AwaitCondition(lambda : ": Finished" in vehicle.next_StatusText(None).text)

    val = 1100
    while not ac.result:
        val = 1900 if val == 1100 else 1100
        vehicle.send_message(RCOverride.set_channels(vehicle.sysid, vehicle.compid,
            channels={
                0: val if channel == 0 else 1500,
                1: val if channel == 1 else 1500,
                2: 1700,
                3: val if channel == 3 else 1500,
                4: 1500,
                5: 1500
            }
        ))
        sleep(0.75)
        

    
vehicle.set_mode(mavlink.PLANE_MODE_LOITER)