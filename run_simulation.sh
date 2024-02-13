#!/bin/sh
echo "Running simulation: $1"

case $1 in
    gturn)
        echo "bundling gturn"
        python bundler.py -s src/main_gturn.lua
        ;;
    parabola)
        echo "bundling parabola"
        python bundler.py -s src/main_parabolic.lua
        ;;
    *)
        echo "running previos bundle"
        ;;
esac

echo "running"
./sim_wrapper.py & (cd sitl; sh run_sim_vehicle.sh) && fg


