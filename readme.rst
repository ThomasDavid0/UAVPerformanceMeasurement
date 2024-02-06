ArduPlane Performance Testing LUA Scripts
-----------------------------------------

This is a WIP project intended to assess the performance characteristics of a fixed wing UAV using ArduPlane.

LUA scripts are included to perform manoeuvres designed to seperate the drag (both lift dependent and cd0) from
thrust. Python code is also included to use the PyFlightCoach packages to analyse process the data.

The project is setup so that it should run in sitl and jsbsim (assuming you have set sitl up according to 
https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html). 


The lua scripts are written with modules, which are not supported by ardupilot. The intention is to use a lua
bundler to put them into single files. https://github.com/Benjamin-Dobell/luabundler

To bundle the lua scripts:

.. code-block:: console
    
    luabundler bundle src/script.lua -p "$PWD/src/?.lua" -o scripts/bundle.lua



To run the simulation:

.. code-block:: console

    $ sh run_simulation

Then in MavProxy:

.. code-block:: console

    mode auto
    arm throttle


To run on an Aircraft:

set scr_enable to 1 and reboot

Copy the scripts to the scripts directory on the aircraft

