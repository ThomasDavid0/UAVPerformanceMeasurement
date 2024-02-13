from flightplotting import plotsec, plot_regions
from flightdata import Flight, State
import plotly.express as px
import plotly.graph_objects as go
import numpy as np
from plots import plot_controller
from parsing import make_controller_field, get_last_log

np.set_printoptions(suppress=True)

fl = Flight.from_log(get_last_log(), 
    ['TSPD', 'TRAN', 'TRAL', 'TPIT', 'TPIG', 'TYAW', 'TINF'], 
    TSPD = make_controller_field('tspd'),
    TRAN = make_controller_field('tran'),
    TRAL = make_controller_field('tral'),
    TPIT = make_controller_field('tpit'),
    TPIG = make_controller_field('tpig'),
    TYAW = make_controller_field('tyaw'),
    TINF = dict(
        tinf_id='id',
        tinf_cmd='cmd',
        tinf_loadfactor='loadfactor',
        tinf_stage='stage'
    )
)

st = State.from_flight(fl).label(
    turn=fl.tinf_id.fillna(0).astype(int).map(lambda x: f'turn_{x}'),
    stage=fl.tinf_stage.fillna(99).astype(int).map(lambda x: f'turn_{x}'),
    )

#plot_regions(st, 'turn').show()

stt = st.get_label_subset(turn='turn_1')

#plot_regions(st, 'stage').show()

flt = fl.slice_time_flight(slice(stt.t[0],stt.t[-1]))

plotsec(stt, nmodels=20, scale=1).show()

#plot_controller('tspd', flt.tspd).show()
plot_controller('tran', flt.tran).show()
plot_controller('tral', flt.tral).show()
plot_controller('tpit', flt.tpit).show()
plot_controller('tpig', flt.tpig).show()
plot_controller('tyaw', flt.tyaw).show()


#plot_controller('tyaw', flt.tyaw)

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




