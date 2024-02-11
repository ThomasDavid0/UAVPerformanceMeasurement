
import plotly.graph_objects as go



def plot_controller(name, df):
    return go.Figure(
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
    )
    