#!/home/td6834/mambaforge/envs/dint/bin/python
from droneinterface import Vehicle, mavlink
from time import sleep


while True:
    try:
        vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False)
        break
    except Exception:
        pass

sleep(2)

vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_AUTO)


