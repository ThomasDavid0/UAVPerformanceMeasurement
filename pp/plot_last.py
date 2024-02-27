from parsing import get_last_log, make_controller_field
from plots import plot_controller
from flightdata import Flight, State
from flightplotting import plotsec, plot_regions

import numpy as np
import pandas as pd

fl = Flight.from_log(get_last_log(), 
    ['TSPD', 'TRAN', 'TRAL', 'TPIT', 'TPIG', 'TYAW', 'TINF'], 
    TSPD = make_controller_field('tspd'),
)

labels = np.full(len(fl), 'auto')
labels[pd.isna(fl.data.tspd_Targ)] = 'controller'


st = State.from_flight(fl).label(controller=labels)

plot_regions(st, 'controller').show()

plot_controller('tspd', fl.tspd).show()

