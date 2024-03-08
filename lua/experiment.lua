local flightmodes = require('modules/mappings/modes')

local experiments = {}
experiments[0] = require('experiments/pitch_angle_controller_tuning')
experiments[1] = require('experiments/alt_controller_tuning')
experiments[2] = require('experiments/roll_angle_controller_tuning')
experiments[3] = require('experiments/speed_controller_tuning')


function update()
    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()

        local exp = experiments[0].setup()

        if cmd then
            exp:run()
        else
            if exp then
                exp:reset()
            end
        end
    end
    return update, 1000.0/40
end


return update()
