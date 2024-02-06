from flightplotting import plotsec, plot_regions
from flightdata import Flight, State
from pathlib import Path
from ardupilot_log_reader import Ardupilot
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
import pandas as pd
import geometry as g
np.set_printoptions(suppress=True)

file = sorted(list(Path('sitl/logs').glob('*.BIN')))[-1]


make_controller_field = lambda name:  {f'{name}_{v}': v for v in 'Targ,Curr,err,dt,P,I,D,Total'.split(',')}
make_q_field = lambda name:  {f'{name}_{v}': v for v in 'wxyz'}

fl = Flight.from_log(file, ['TSPD','TINF', 'TRLL', 'TRST', 'TRQT', 'TRQ0'], 
    TSPD = make_controller_field('tspd'),
    TRLL = make_controller_field('trll'),
    TRQT = make_q_field('trqt'),
    TRQ0 = make_q_field('trq0'),
    TINF = dict(
        tinf_id='id',
        tinf_cmd='cmd',
        tinf_loadfactor='loadfactor'
    ),
    TRST = dict(
        trst_arsd='arspd',
        trst_roll='roll',
        trst_pitch='pitch',
        trst_yaw='yaw',
    )
)

def plot_controller(name, df):
    go.Figure(
        data = [
            go.Scatter(x=df.index, y=df[f'{name}_Targ'], name='Targ', line=dict(dash='solid'), yaxis='y'),
            go.Scatter(x=df.index, y=df[f'{name}_Curr'], name='Curr', line=dict(dash='solid'), yaxis='y'),
            go.Scatter(x=df.index, y=df[f'{name}_err'], name='err', line=dict(dash='solid'), yaxis='y'),
            go.Scatter(x=df.index, y=df[f'{name}_P'], name='P', line=dict(dash='dash'), yaxis='y2'),
            go.Scatter(x=df.index, y=df[f'{name}_I'], name='I', line=dict(dash='dash'), yaxis='y2'),
            go.Scatter(x=df.index, y=df[f'{name}_D'], name='D', line=dict(dash='dash'), yaxis='y2'),
            go.Scatter(x=df.index, y=df[f'{name}_Total'], name='Total', line=dict(dash='dash'), yaxis='y2'),
        ],
        layout=dict(
            title=name,
            yaxis=dict(title='value'),
            yaxis2=dict(title='controller', overlaying='y', side='right')
        )
    ).show()
    

st = State.from_flight(fl).label(turn=fl.tinf_id.fillna(0).astype(int).map(lambda x: f'turn_{x}'))

plot_regions(st, 'turn').show()

stt = st.get_label_subset(turn='turn_1')
flt = fl.slice_time_flight(slice(stt.t[0],stt.t[-1]))

plotsec(stt, nmodels=20, scale=1).show()


plot_controller('tspd', flt.tspd)
plot_controller('trll', flt.trll)



go.Figure(
    data=[
        go.Scatter(x=stt.data.index,  y=-stt.dw / 9.81, name='load factor'),
        go.Scatter(x=stt.data.index,  y=flt.air_speed, name='airspeed', yaxis='y2'),
        go.Scatter(x=stt.data.index,  y=abs(stt.vel), name='ground speed', yaxis='y2'),
    ],
    layout=dict(
        title='turn info',
        yaxis=dict(title='load factor, g'),
        yaxis2=dict(title='speed, m/s', overlaying='y', side='right'),
    )
).show()



#        plotter(ts, turn)
        


