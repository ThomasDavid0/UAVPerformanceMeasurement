from flightdata import Flight, State




def get_last_log():
    from pathlib import Path
    return sorted(list(Path('sitl/logs').glob('*.BIN')))[-1]





make_controller_field = lambda name:  {f'{name}_{v}': v for v in 'Targ,Curr,err,dt,P,I,D,Total'.split(',')}
make_q_field = lambda name:  {f'{name}_{v}': v for v in 'wxyz'}


