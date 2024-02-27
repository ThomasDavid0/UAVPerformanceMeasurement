#!/bin/sh

while getopts p:l: flag
do
    case "${flag}" in
        p) pymavlink=${OPTARG};;
        l) lua=${OPTARG};;
        
    esac
done

if test -f "$lua"; then
    echo "bundling $lua"
    lua_bundler $lua sitl/scripts/bundle.lua
else
    echo "lua file $lua does not exist"
fi

if test -f "$pymavlink"; then
    echo "gcs script:$pymavlink"
    $pymavlink & (cd sitl; sh run_sim_vehicle.sh) && fg
else
    echo "gcs script $pymavlink does not exist"
    (cd sitl; sh run_sim_vehicle.sh)
fi





