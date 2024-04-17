
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')
local Turn = require('modules/turn')

local roll_controller = require('modules/controllers/c72_roll_angle_controller')
local speed_controller = require('modules/controllers/c71_speed_controller')
local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
local yaw_controller = require('modules/controllers/c75_yaw_controller')

local turn

local Exp = {}
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()
    
    function self:reset()
        turn = Turn.new(0, 2, -state:pos():z(), state:arspd())
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        alt_controller:reset()
        roll_controller:reset()
        yaw_controller:reset()
    end

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            speed_controller:update(0.0, turn:arspd(), state:arspd()),
            roll_controller:update(0, turn:roll_angle_deg(), state:roll_angle_deg()),
            alt_controller:update(
                turn:alt(),
                -state:pos():z(),
                state:roll_angle_deg(),
                state:pitch_angle_deg()
            ),
            yaw_controller:update(0.0, 0.0, -state:flow():y())
        )
        vehicle:set_rudder_offset(0, true)
    end

    return self
end

return Exp


