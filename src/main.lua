
local controller = require('controller')
local P = require('geometry/point')
local State = require('state')
local t = require('turn')

local MODE_AUTO = 10


local active_turn = nil

local speed_controller = controller.PID('TSPD', 13, 2, 0.0, 10, 100)
local roll_controller = controller.PID('TRLL', 2.3, 0.0, 0.14, -360, 360)

function update()

    if vehicle:get_mode() == MODE_AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            if active_turn then
                local current_state = State.readCurrent()

                local thr = speed_controller.update(active_turn:arspd(), current_state:arspd())

                local roll = roll_controller.update(active_turn.roll_angle()*180/math.pi, current_state.roll_angle() * 180 / math.pi)
                
                local w_z_err = (active_turn.alt() - current_state.pos():z()) / math.cos(current_state.roll_angle())
                local b_z_err = current_state.att().transform_point(P.xyz(0,0,w_z_err)):z()
                local pitch = -180 * (b_z_err * 2 - current_state.vel():z() * 2.8) / (current_state.arspd() * math.pi)

                local yaw = 0.0

                logger.write(
                    'TINF', 'id,cmd,loadfactor', 'iif',
                    id, cmd, active_turn.load_factor()
                )

                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, yaw)
            else
                active_turn = t.initialise(id, cmd)
            end
        else
            active_turn = nil
        end
    end
    return update, 1000.0/40
end


return update()


