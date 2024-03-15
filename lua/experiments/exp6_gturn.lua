
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')
local Turn = require('modules/turn')

local roll_controller = require('modules/controllers/roll_angle_controller')
local speed_controller = require('modules/controllers/speed_controller')
local alt_controller = require('modules/controllers/alt_controller')
local yaw_controller = require('modules/controllers/yaw_controller')


local turn = nil

local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()
    local _id = id

    function self:run()
        if turn then

            if turn:stage() == 0 then
                -- roll to initial roll angle whilst maintaining altitude

            elseif turn:stage() == 1 then
                -- maintain g force with pitch, altitude with roll angle
            end
            
            vehicle:set_target_throttle_rate_rpy(
                speed_controller:update(0.0, turn:arspd(), state:flow():length()),
                roll_controller:update(0.0, 0, math.deg(state:roll_angle())),
                alt_controller:update(0.0, 100, -state:pos():z(), math.deg(state:pitch_angle())),
                yaw_controller:update(0.0, -state:flow():y())
            )
            vehicle:set_rudder_offset(0, true)

        else
            turn = Turn.new(
                1.2, --1.2 + 0.4 * ((_id-1) % 5),
                ahrs:get_relative_position_NED_origin():z(),
                22 --15 + 3 * math.floor((id-1) / 5)
            )
        end
    end

    function self:reset()
        turn = nil
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        alt_controller:reset()
        roll_controller:reset()
        yaw_controller:reset()
    end

    return self
end

return Exp


