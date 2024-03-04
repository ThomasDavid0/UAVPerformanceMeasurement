
local State = require('state')
local functions = require('mappings/functions')
local flightmodes = require('mappings/modes')


local speed_controller = require('controllers/speed_controller')
-- PID.new('TSPD', 15, 15, 0, 0, 100)

local started = millis()
local s0 = nil
local step_size = 5

function update()

    if vehicle:get_mode() == flightmodes.AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        local state = State.readCurrent()

        if cmd then
            local target_speed = s0
            if millis() - started > 10000 then
                target_speed = s0 + step_size
                started = millis()
                step_size = -step_size
                s0 = target_speed
            end

            local thr = speed_controller:update(
                0.0,
                target_speed,
                state:flow():length()
            )

            vehicle:set_target_throttle_rate_rpy(thr, 0, 0, 0)
            vehicle:set_rudder_offset(0, true)

        else
            started = millis()
            s0 = state:flow():length()
            --gcs:send_text(6, string.format("throttle = %i", SRV_Channels:get_output_scaled(functions.throttle)))
            speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        end
    end
    return update, 1000.0/40
end


return update()


