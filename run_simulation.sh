#!/bin/sh
python bundler.py -s src/main_parabolic.lua

./sim_wrapper.py & (cd sitl; sh run_sim_vehicle.sh) && fg


