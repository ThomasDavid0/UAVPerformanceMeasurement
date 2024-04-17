
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local Turn = require('modules/turn')

local speed_controller = require('modules/controllers/c71_speed_controller')
local pitch_alt_controller = require('modules/controllers/c74_pitch_alt_controller')
local yaw_controller = require('modules/controllers/c75_yaw_controller')
local turn_climb_controller = require('modules/controllers/c77_turn_climb_controller')
local pitch_g_controller = require('modules/controllers/c76_pitch_g_controller')
local turn = nil

local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:run()
        if turn then
            local _roll = 0
            local _pitch = 0

            if turn:stage() == 0 then
                if state:roll_angle() > turn:roll_angle() then
                    turn:next_stage()
                    pitch_g_controller:reset(pitch_alt_controller:pitch_controller():total())
                    
                else
                    _roll = 10
                    _pitch = pitch_alt_controller:update(
                        turn:alt(),
                        -state:pos():z(),
                        state:roll_angle_deg(),
                        state:pitch_angle_deg()
                    )
                end
            end
            if turn:stage() == 1 then
                _roll = turn_climb_controller:update(
                    0, state:att():transform_point(state:vel()):z(),
                    state:roll_angle_deg()
                )
                _pitch = pitch_g_controller:update(0, turn:load_factor(), -state:acc():z()/9.81)
            end

            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(0.0, turn:arspd(), state:arspd()),
                _roll, _pitch,
                yaw_controller:update(0.0, 0.0, -state:flow():y())
            )
            vehicle:set_rudder_offset(0, true)

        else
            turn = Turn.new(
                0,
                2, --1.2 + 0.4 * ((_id-1) % 5),
                -state:pos():z(),
                state:arspd() --15 + 3 * math.floor((id-1) / 5)
            )
        end
    end

    function self:reset()
        turn = nil
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        pitch_alt_controller:reset()
        pitch_g_controller:reset()
        turn_climb_controller:reset()
        yaw_controller:reset()
    end

    return self
end

return Exp


