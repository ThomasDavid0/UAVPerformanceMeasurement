
local PID = require('pid')
local P = require('geometry/point')
local State = require('state')
local Turn = require('turn')

local MODE_AUTO = 10

local turn = nil

local speed_controller = PID.new('TSPD', 13, 10, 0.0, 10, 100)
local rollalt_controller = PID.new('TRAL', 0.5, 0.6, 0.5, 0, 90) -- control vertical speed with roll angle
local rollangle_controller = PID.new('TRAN', 0.6, 0.2, 0.2, -90, 90) -- control roll angle with roll rate


local pitch_controller = PID.new('TPIT', 4, 2, 4.5, -180, 180)
local pitch_g_controller = PID.new('TPIG', 3, 10, 1.0, -180, 180)
local yaw_controller = PID.new('TYAW', 1.5, 2.5, 1.0, -20, 20)

local stagetimer = 0

function update()

    if vehicle:get_mode() == MODE_AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            if turn then

                local state = State.readCurrent()

                local thr = speed_controller:update(
                    turn:arspd(),
                    state:arspd()
                )
                local yaw = yaw_controller:update(0.0, -state:vel():y())
                local roll = 10.0
                local pitch = 0.0

                if turn:stage() == 0 and turn:roll_angle() < state:roll_angle() then
                    gcs:send_text(6, 'LUA: moving to stage 1')
                    turn:next_stage()
                    stagetimer = millis() / 1000
                elseif turn:stage() == 1 and (millis() / 1000 - stagetimer > 1)then
                    gcs:send_text(6, 'LUA: moving to stage 2')
                    turn:next_stage()
                    rollalt_controller:reset(math.deg(state:roll_angle()))
                    pitch_g_controller:reset(pitch_controller:I())
                    stagetimer = millis() / 1000
                end

                if turn:stage() < 2 then
                    if turn:stage() == 1 then
                        roll = rollangle_controller:update(math.deg(turn:roll_angle()), math.deg(state:roll_angle()))
                    end
                    local w_z_err = (turn:alt() - state:pos():z()) / math.cos(state:roll_angle())
                    local pitch_err = state:att():transform_point(P.z(w_z_err)):z() / state:vel():length()
                    pitch = pitch_controller:update(0.0,  math.deg(pitch_err))
                elseif turn:stage() == 2 then
                    pitch = pitch_g_controller:update(turn:load_factor(), -state:acc():z() / 9.81 )
                    local roll_angle = rollalt_controller:update(0, state:att():transform_point(state:vel()):z())
                    roll = rollangle_controller:update(roll_angle, math.deg(state:roll_angle()))
                end

                logger.write('TINF', 'id,cmd,loadfactor,stage', 'iifi', id, cmd, turn:load_factor(), turn:stage())

                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, yaw)
                vehicle:set_rudder_offset(0, true)

            else
                turn = Turn.initialise(id, cmd)
                speed_controller:reset()
                pitch_controller:reset()
                rollangle_controller:reset()
                yaw_controller:reset()

                gcs:send_text(6, string.format('LUA: %s', turn:summary()))
            end
        else
            turn = nil
        end
    end
    return update, 1000.0/40
end


return update()


