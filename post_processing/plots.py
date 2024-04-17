
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np

def controller_responses(name, df, targ_args={}, curr_args={}, **kwargs):
    return [
        go.Scatter(x=df.index, y=df[f'{name}_Targ'], name='target', **targ_args, **kwargs),
        go.Scatter(x=df.index, y=df[f'{name}_Curr'], name='current', **curr_args, **kwargs),
    ]

def controller_internals(name, df, **kwargs):
    return [
        go.Scatter(x=df.index, y=df[f'{name}_P'], name='P', **kwargs),
        go.Scatter(x=df.index, y=df[f'{name}_I'], name='I', **kwargs),
        go.Scatter(x=df.index, y=df[f'{name}_D'], name='D', **kwargs),
        go.Scatter(x=df.index, y=df[f'{name}_Total'], name='Total', **kwargs),
    ]



def plot_controller(name, df):
    return go.Figure(
        data = controller_responses(name, df, line=dict(dash='solid'), yaxis='y') + \
            controller_internals(name, df, line=dict(dash='dash'), yaxis='y2'),
        layout=dict(
            title=name,
            yaxis=dict(title='value'),
            yaxis2=dict(title='controller', overlaying='y', side='right')
        )
    )


def plot_controllers(names: list[str], df):
    names = [n for n in names if f'{n.lower()}_Targ' in df]
    fig = make_subplots(
        rows=len(names), cols=1, shared_xaxes=True,
        vertical_spacing=(0.2 / (len(names) - 1)),
        subplot_titles=names
    ).update_layout(
        template='simple_white',
        font=dict(family="Rockwell", size=24),
        height=200*len(names),
    )
    for i, name in enumerate(names):
        fig.add_traces(
            controller_responses(
                name.lower(), df,
                dict(line=dict(color='red', width=1)),
                dict(line=dict(color='blue', width=1)),
                showlegend=i==0,

            ), 
            rows=[i+1 for _ in range(2)], 
            cols=[1 for _ in range(2)]
        )
    return fig