local flightmodes = require('modules/mappings/modes')
local PT = require('modules/param_table')

local experiments = {}
experiments[0] = require('experiments/exp0_pitch_angle_controller_tuning')
experiments[1] = require('experiments/exp1_alt_controller_tuning')
experiments[2] = require('experiments/exp2_roll_angle_controller_tuning')
experiments[3] = require('experiments/exp3_speed_controller_tuning')
experiments[4] = require('experiments/exp4_yaw_controller_tuning')
experiments[5] = require('experiments/exp5_pitch_g_controller_tuning')
experiments[6] = require('experiments/exp6_coordinated_turn')
experiments[7] = require('experiments/exp7_gturn')
experiments[8] = require('experiments/exp8_speed_acc_controller_tuning')
experiments[9] = require('experiments/exp9_parabola')

local pt = PT.new(69, "SCRTEXP_", 7)

local expid = pt:param('EXPID', 0)
local running_exp = nil
function update()
    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        local eid = expid:get()
        local exp = experiments[eid].setup(id, cmd)

        if cmd then
            if running_exp == eid then
                logger:write('TEXP', 'id', 'i', eid )
                exp:run()
            end
        else
            if exp then
                exp:reset()
                running_exp = eid
            end
        end
    end
    return update, 1000.0/40
end


return update()
