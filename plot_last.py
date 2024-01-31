from flightplotting import plotsec
from flightdata import Flight, State
from pathlib import Path
from ardupilot_log_reader import Ardupilot
import plotly.express as px
import plotly.graph_objects as go

file = sorted(list(Path('logs').glob('*.BIN')))[-1]


log = Ardupilot(file, types=Flight.ardupilot_types + ['TRN1'])

turns = log.TRN1.groupby('id')

fl = Flight.from_log(log)


for _turn in turns:
    turn = _turn[1]
    ts = turn.timestamp - fl.time_actual[0]
    flt = fl[ts.iloc[0]:ts.iloc[-1]]
    
    st = State.from_flight(flt)    
    
    plotsec(st, nmodels=20).show()

    
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=ts, y=-turn['alt'], name='alt', yaxis='y'))
    fig.add_trace(go.Scatter(x=ts, y=turn['despitch'], name='pitch', yaxis='y2'))
    fig.update_layout(
        yaxis=dict(title='Altitude (m)'),
        yaxis2=dict(title='Pitch (deg)', side='right', overlaying='y')
    )
    fig.show()
    
    pass


