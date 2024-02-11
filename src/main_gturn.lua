
local PID = require('pid')
local P = require('geometry/point')
local State = require('state')
local Turn = require('turn')

local MODE_AUTO = 10

local active_turn = nil

local speed_controller = PID.new('TSPD', 13, 2, 0.0, 10, 100)
local roll_controller = PID.new('TRLL', 2.3, 0.0, 0.14, -180, 180)
local pitch_controller = PID.new('TPIT', 6, 0, 2, -180, 180)
local yaw_controller = PID.new('TYAW', 100, 0, 0.0, -180, 180)

local reset_time = 0
local yaw = 20

function update()

    if vehicle:get_mode() == MODE_AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            if active_turn then

                local current_state = State.readCurrent()

                local thr = speed_controller:update(
                    active_turn:arspd(),
                    current_state:arspd()
                )

                local roll = roll_controller:update(
                    math.deg(active_turn:roll_angle()),
                    math.deg(current_state:roll_angle())
                )
                
                local w_z_err = (active_turn:alt() - current_state:pos():z()) / math.cos(current_state:roll_angle())
                local pitch_err = current_state:att():transform_point(P.z(w_z_err)):z() / current_state:arspd()
                
                local pitch = pitch_controller:update(0.0,  math.deg(pitch_err))
                
                
                if millis() / 1000 - reset_time > 1 then
                    yaw = -yaw
                    reset_time = millis() / 1000
                    gcs:send_text(6, string.format("LUA: setting yaw to %s", yaw))
                end
                --local yaw = yaw_controller:update(0.0, -current_state:vel():y())

                logger.write('TINF', 'id,cmd,loadfactor', 'iif', id, cmd, active_turn:load_factor())

                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, yaw)
            else
                active_turn = Turn.initialise(id, cmd)
            end
        else
            active_turn = nil
        end
    end
    return update, 1000.0/40
end


return update()


