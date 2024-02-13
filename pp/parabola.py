from flightplotting import plotsec, plot_regions, plotdtw
from flightdata import Flight, State
from pathlib import Path
from ardupilot_log_reader import Ardupilot
import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
import pandas as pd
import geometry as g
from plots import plot_controller
from parsing import make_controller_field, get_last_log

np.set_printoptions(suppress=True)

fl = Flight.from_log(get_last_log(), ['TSPD', 'TRLL', 'TPIT', 'TYAW', 'PINF'], 
    TRLL = make_controller_field('trll'),
    TPIT = make_controller_field('tpit'),
    PINF = dict(
        pinf_id='id',
        pinf_cmd='cmd',
        pinf_stage='stage'
    )
)


st = State.from_flight(fl) \
    .label(stage=fl.pinf_stage.fillna('auto')) \
        .label(mode=fl.pinf_id.fillna(0).astype(int).map(lambda x: f'para_{x}'))



stt = st.get_label_subset(mode='para_1')
flt = fl.slice_time_flight(slice(stt.t[0],stt.t[-1]))


def plot_para(_fl, _st):
    return go.Figure(
        data=[
            go.Scatter(x=_st.data.index,  y=-_st.dw / 9.81, name='load factor'),
            #go.Scatter(x=_fl.data.index,  y=-_fl.data.tpit_Curr / 9.81 , name='current acc'),
            go.Scatter(x=_st.data.index,  y=_st.z , name='height', yaxis='y2'),
            go.Scatter(x=_st.data.index,  y=_fl.air_speed , name='air speed', yaxis='y3'),
            #go.Scatter(x=_fl.data.index,  y=_fl.data.tpit_Total , name='pitch_demand', yaxis='y4'),
        ],
        layout=dict(
            title='parabola info',
            yaxis=dict(title='load factor, g'),
            yaxis2=dict(title='height, m', overlaying='y', anchor='free', autoshift=True),
            yaxis3=dict(title='speed, m/s', overlaying='y', anchor='free', autoshift=True),
            yaxis4=dict(title='pitch', overlaying='y', anchor='free', autoshift=True),
        )
    )



plot_regions(st, 'mode').show()
#plotsec(stt, nmodels=20, scale=1).show()

sts = []
for i, _st in  st.get_label_subset(stage='para').data.groupby('mode'):
    sts.append(State(_st))

colors = px.colors.qualitative.Plotly

fig = go.Figure()
for i, _st in enumerate(sts):
    bx = _st.att.transform_point(g.PX())
    pitch = np.arcsin(bx.z / abs(bx))
    fig.add_trace(go.Scatter(x=st.t, y=_st.att.inverse().transform_point(_st.acc).z / 9.81, line=dict(color=colors[i], dash='solid'), name='g force', showlegend=i==0))
    fig.add_trace(go.Scatter(x=st.t, y=pitch, line=dict(color=colors[i], dash='dash'), yaxis='y2', name='pitch angle', showlegend=i==0))
    fig.add_trace(go.Scatter(x=st.t, y=abs(_st.vel), line=dict(color=colors[i], dash='dot'), yaxis='y3', name='speed', showlegend=i==0))
    
    
fig.update_layout(
    yaxis=dict(title='g force'),
    yaxis2=dict(title='pitch, rad', overlaying='y', anchor='free', autoshift=True),
    yaxis3=dict(title='speed, m/s', overlaying='y', anchor='free', autoshift=True),
)
fig.show()


#plot_controller('trll', flt.trll).show()
#plot_controller('tpit', flt.tpit).show()

#stp = stt.get_label_subset(stage='para')
#flp = fl.slice_time_flight(slice(stp.t[0],stp.t[-1]))
#
#plot_para(flp, stp).show()