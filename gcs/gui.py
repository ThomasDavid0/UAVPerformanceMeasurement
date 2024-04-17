#!/home/td6834/mambaforge/envs/dint/bin/python
from matplotlib import use as use_agg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import PySimpleGUI as sg
import livematplot as lm
from droneinterface import Vehicle, mavlink, Watcher, Timeout
from loguru import logger
import argparse
import numpy as np
import sys

logger.remove()
logger.add(sys.stderr, level='DEBUG')

controllers = ['TSPD', 'TRAN', 'TPAN', 'TPALT', 'TYAW', 'TPG', 'TRCLB', 'TSPA']
experiments = list(range(10))
parameters = ['kFF', 'kP', 'kI', 'kD', 'max', 'min']

parser = argparse.ArgumentParser(description='gui')
#for mission planner forwarding: udp:10.42.0.1:14560
#for sim: tcp:0.0.0.0:5762
parser.add_argument('-m', '--master', type=str, default='udp:10.42.0.1:14560', help='Mavlink connection string')
parser.add_argument('--takeoff', default=False, action=argparse.BooleanOptionalAction, help='command vehicle to takeoff when connected')
parser.add_argument('--timeout', type=int, help='droneinterface connection timeout')
args = parser.parse_args()

logger.debug("Connecting to vehicle")
vehicle = Vehicle.connect(args.master, 1, retries=20, timeout=args.timeout)
vehicle.request_parameters()

if args.takeoff:
    logger.info("Taking off")
    vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)
    vehicle.arm()
    vehicle.set_mode(mavlink.PLANE_MODE_AUTO)

logger.debug("Setting up watcher")
vehicle.request_message(228)
watcher = Watcher(lambda : vehicle.next_custompidstate(0.5).data, 5000, None)


def get_data(controller):
    if min(len(watcher.data), len(watcher.times)) > 0:
        if watcher.last_result_age() > 0.5:
            watcher.reset()
    df = watcher.dataframe(columns=['controller', 'input', 'target', 'dt', 'FF', 'P', 'I', 'D'])
    return df.loc[df.controller==controller]

def pack_figure(graph, figure):
    logger.debug('Packing Figure')
    canvas = FigureCanvasTkAgg(figure, graph.Widget)
    plot_widget = canvas.get_tk_widget()
    plot_widget.pack(side='top', fill='both', expand=1)
    return plot_widget

use_agg('TkAgg')


def gain_input(param: str):
    return [
       sg.Text(param, size=(5,1)),
       sg.Input(key=f'value_{param}', size=(5,1),default_text='-', disabled=True), 
       sg.Button('>',key=f'write_{param}', size=(1,1), disabled=True) 
    ]

def create_gain_table_data():
    return [[c] + [np.round(vehicle.get_parameter(f'SCR{c}_{p}')[0], 2) for p in parameters] for c in controllers ]

cont = 'TPAN'
layout = [
    [sg.Graph((640, 480), (0, 0), (640, 480), key='Graph1')],
    [sg.Radio(n, group_id=1, key=f'plot_{n.lower()}', default=n=='TSPD') for n in controllers],
    [sg.Radio(i, group_id=2, key=f'exp_{i}', default=i==vehicle.get_parameter('SCRTEXP_EXPID', 10)[0], enable_events=True) for i in experiments],
    [sg.Text(p, size=(7,1)) for p in parameters],
    [
        sg.Table(
            create_gain_table_data(),
            headings = ['Cont'] + parameters, expand_x=True, enable_events=True, enable_click_events=True,
            key='gain_table'
        ),
        sg.Column([[sg.Text('na', key='active_controller')]]+[gain_input(p) for p in parameters])
    ],
    [sg.Button('Exit')]
]

def get_plot_controller(values):
    for c in controllers:
        if values[f'plot_{c.lower()}']:
            return c
    return controllers[0]

def set_experiment(eid):
    current_exp = vehicle.get_parameter('SCRTEXP_EXPID')[0]
    if current_exp != eid:
        logger.info(f"Setting experiment from {current_exp} to {eid}")
        vehicle.set_parameter('SCRTEXP_EXPID', eid)

logger.debug("building window")

window = sg.Window('Matplotlib', layout, finalize=True)
graph1 = window['Graph1']

plt.ioff()
fig1 = plt.figure(1)

axi = [
    lm.AxesInfo('signal'),
    lm.AxesInfo('control'),
]
tri = [
    lm.TraceInfo('input', 0),
    lm.TraceInfo('target', 0),
    lm.TraceInfo('FF', 1),
    lm.TraceInfo('P', 1),
    lm.TraceInfo('I', 1),
    lm.TraceInfo('D', 1),
]

axs = lm.create_axes(fig1, axi)
pack_figure(graph1, fig1)

logger.debug("creating traces")

lines = lm.create_traces(axs, tri, get_data('TPAN'))
lm.create_legends(axs, tri)


fig1.canvas.draw()
logger.debug("starting main loop")
while True: #vehicle.conn.is_alive():
    event, values = window.read(timeout=100)
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    elif event == sg.TIMEOUT_EVENT:
        data = get_data(get_plot_controller(values))
        if len(data) > 0:
            lm.update_traces(lines, axs, axi, tri, data)
            fig1.canvas.draw()
    elif isinstance(event, str):
        if event.startswith('exp_'):
            set_experiment(int(event[4:]))
        elif event.startswith('write_'):
            controller = window['active_controller'].DisplayText
            parameter = event[6:]
            new_value = float(values[event.replace('write_', 'value_')])
            old_value = vehicle.get_parameter(f'SCR{controller}_{parameter}')[0]
            if new_value != old_value:
                logger.info(f"Setting {controller} {parameter} from {old_value} to {new_value}")
                try:
                    vehicle.set_parameter(f'SCR{controller}_{parameter}', new_value)
                except Timeout:
                    logger.info("Timeout setting parameter")
                window["gain_table"].update(create_gain_table_data())
    elif isinstance(event, tuple):
        if event[0] == 'gain_table':
            if event[1] == '+CLICKED+':
                if event[2][0]:
                    controller = controllers[event[2][0]]
                    window['active_controller'].update(controller)
                    for p in parameters:
                        window[f'value_{p}'].update(
                            vehicle.get_parameter(f'SCR{controller}_{p}')[0],
                            disabled=False
                        )
                        window[f'write_{p}'].update(disabled=False)
                else:   
                    window['active_controller'].update('na')
                    for p in parameters:
                        window[f'value_{p}'].update('-', disabled=True)
                        window[f'write_{p}'].update(disabled=True)



window.close()