#!/home/td6834/mambaforge/envs/dint/bin/python
from droneinterface import Vehicle, mavlink, Watcher
from time import sleep
import livematplot as lm
import numpy as np
import pandas as pd

vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, retries=20)

vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)
vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_AUTO)


#vehicle.set_parameter('SIM_WIND_DIR', 180)
#vehicle.set_parameter('SIM_WIND_SPD', 10)

watcher = Watcher(lambda : vehicle.next_namedvaluefloat(0.5).data, 500, None)


def get_data():
    return watcher.dataframe(columns=['t', 'name', 'value']).set_index('t').groupby('name').value

sleep(2)
print(get_data())

lm.create_live_plot(
    source=get_data,
    tri = [
        lm.TraceInfo('st_arspd', 0, 'red'),
        lm.TraceInfo('st_flow', 0, 'blue'),
        lm.TraceInfo('st_w_spd', 0, 'green')
    ],
    axi = [lm.AxesInfo('arspd', None)]
)
pass
#df.to_csv('temp.csv')