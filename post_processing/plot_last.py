from parsing import get_last_log, make_controller_field
from plots import plot_controllers
from flightdata import Flight, State
from flightplotting import plot_regions
from pathlib import Path
import numpy as np
import pandas as pd
import argparse

parser = argparse.ArgumentParser(description='plot logs')
parser.add_argument('log', type=int, nargs='?', default=None)
args = parser.parse_args()

if args.log is None:
    file = get_last_log()
else:
    file = sorted(list(Path('flight_logs').rglob(f'*{args.log}.BIN')))[-1]

jsonfile = Path(str(file).replace('.BIN', '.json'))
controllers = ['TSPD', 'TRAN', 'TPAN', 'TPAL', 'TYAW', 'TPG', 'TRCL']
if jsonfile.exists():
    fl = Flight.from_json(jsonfile)
else:
    
    fl = Flight.from_log(file, 
        controllers,
        **{cnt: make_controller_field(cnt.lower()) for cnt in controllers} 
    )
    fl.to_json(jsonfile)


labels = pd.Series(data=np.full(len(fl.data), 'base'), index=fl.data.index)
for cnt in controllers:
    col = f'{cnt.lower()}_Targ'
    if col not in fl.data.columns: 
        continue
    labids = np.isfinite(fl.data.loc[:, f'{cnt.lower()}_Targ'])
    labels[labids] = labels[labids] + f'_{cnt.lower()}'
    pass

st = State.from_flight(fl).label(controller=labels)

plot_regions(st, 'controller').show()

plot_controllers(controllers, fl.data).show()
#for cnt in controllers:
#    plot_controller(cnt.lower(), getattr(fl, cnt.lower())).show()
