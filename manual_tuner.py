#!/home/td6834/mambaforge/envs/dint/bin/python
from droneinterface import Vehicle, mavlink, Watcher
import livematplot as lm

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, retries=20)
vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)
vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_AUTO)

vehicle.get_custompidstate(None)

def read_data():
    while True:
        msg = vehicle.next_custompidstate(0.5)
        if msg.name == 'TSPD':
            return msg.data

watcher = Watcher(read_data, 5000, None)

def get_data():
    if min(len(watcher.data), len(watcher.times)) > 0:
        if watcher.last_result_age() > 0.5:
            watcher.reset()
    return watcher.dataframe(columns=['controller', 'input', 'target', 'dt', 'FF', 'P', 'I', 'D'])

lm.create_live_plot(
    source = get_data,
    tri = [
        lm.TraceInfo('input', 0),
        lm.TraceInfo('target', 0),
        lm.TraceInfo('FF', 1),
        lm.TraceInfo('P', 1),
        lm.TraceInfo('I', 1),
        lm.TraceInfo('D', 1),
    ],
    axi = [
        lm.AxesInfo('signal'),
        lm.AxesInfo('control'),
    ]
)


