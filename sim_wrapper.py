#!/home/td6834/mambaforge/envs/dint/bin/python
from droneinterface import Vehicle, mavlink

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, retries=20)
vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)
vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_AUTO)


