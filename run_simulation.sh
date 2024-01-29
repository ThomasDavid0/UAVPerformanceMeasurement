#!/bin/sh
sim_vehicle.py -v ArduPlane --console --map \
    --custom-location=51.423419263383636,-2.671503820236826,116.9,80 \
    --add-param-file=mav.parm \
    --speedup=3 

