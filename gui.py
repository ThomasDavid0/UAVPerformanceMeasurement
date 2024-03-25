#!/home/tom/mambaforge/envs/droneinterface/bin/python
from matplotlib import use as use_agg
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import PySimpleGUI as sg
import livematplot as lm
from droneinterface import Vehicle, mavlink, Watcher


vehicle = Vehicle.connect('tcp:127.0.0.1:5762', 1, input=False, retries=20)
vehicle.wait_for_test(lambda : vehicle.get_SysStatus().can_arm)
vehicle.arm()
vehicle.set_mode(mavlink.PLANE_MODE_AUTO)

vehicle.request_message(228)
watcher = Watcher(lambda : vehicle.next_custompidstate(0.5).data, 5000, None)

def get_data(controller):
    if min(len(watcher.data), len(watcher.times)) > 0:
        if watcher.last_result_age() > 0.5:
            watcher.reset()
    df = watcher.dataframe(columns=['controller', 'input', 'target', 'dt', 'FF', 'P', 'I', 'D'])
    return df.loc[df.controller==controller]

def pack_figure(graph, figure):
    canvas = FigureCanvasTkAgg(figure, graph.Widget)
    plot_widget = canvas.get_tk_widget()
    plot_widget.pack(side='top', fill='both', expand=1)
    return plot_widget

use_agg('TkAgg')

cont = 'TPAN'
layout = [
    [sg.Graph((640, 480), (0, 0), (640, 480), key='Graph1')],
    [
        sg.Combo(
            ['TPAN', 'TPAL', 'TRAN', 'TSPD', 'TYAW'],
            readonly=True, 
            enable_events=True, 
            key='controller', 
            default_value=cont
        ), 
        sg.Button('Exit')
    ] 
]

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


lines = lm.create_traces(axs, tri, get_data('TPAN'))
fig1.canvas.draw()

while True:
    event, values = window.read(timeout=100)
    if event == sg.WIN_CLOSED or event == 'Exit':
        break
    elif event == sg.TIMEOUT_EVENT:
        data = get_data(values['controller'])
        if len(data) > 0:
            lm.update_traces(lines, axs, axi, tri, data)
            fig1.canvas.draw()

window.close()