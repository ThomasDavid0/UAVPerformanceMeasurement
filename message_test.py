#!/home/tom/mambaforge/envs/droneinterface/bin/python
from droneinterface import Vehicle, mavlink, Watcher, enable_logging
from time import sleep
import livematplot as lm
import numpy as np
import pandas as pd

enable_logging('INFO')

vehicle = Vehicle.connect('com7', 1, input=False, retries=20)


watcher = Watcher(lambda : vehicle.next_namedvaluefloat(0.5).data, 500, None)


def get_data():
    return watcher.dataframe(columns=['t', 'name', 'value']).set_index('t').groupby('name').value

sleep(2)

print(get_data())

lm.create_live_plot(
    source=get_data,
    tri = [
        lm.TraceInfo('pitch_angle', 0, 'red'),
        lm.TraceInfo('roll_angle', 0, 'blue'),
    ],
    axi = [lm.AxesInfo('angle', None)]
)
pass
#df.to_csv('temp.csv')