
local State = require('modules/state')
local functions = require('modules/mappings/functions')
local SQ = require('modules/square_wave')

local roll_controller = require('modules/controllers/c72_roll_angle_controller')
local speed_controller = require('modules/controllers/c71_speed_controller')
local alt_controller = require('modules/controllers/c74_pitch_alt_controller')
local yaw_controller = require('modules/controllers/c75_yaw_controller')
local sq = SQ.new(0, 5000, 2)

local Exp = {}
local initial_state
function Exp.setup(id, cmd)
    local self = {}
    local state = State.readCurrent()

    function self:reset()
        sq:reset()
        speed_controller:reset(SRV_Channels:get_output_scaled(functions.throttle))
        alt_controller:reset()
        roll_controller:reset()
        yaw_controller:reset()
        initial_state = state
    end

    function self:run()
        vehicle:set_target_throttle_rate_rpy(
            speed_controller:update(0.0, initial_state:arspd(), state:arspd()),
            roll_controller:update(0.0, 0, state:roll_angle_deg()),
            alt_controller:update(
                -initial_state:pos():z(),
                -state:pos():z(),
                state:roll_angle_deg(),
                state:pitch_angle_deg()
            ),
            yaw_controller:update(0.0, sq:value(), -state:flow():y())
        )
        vehicle:set_rudder_offset(0, true)
    end

    return self
end

return Exp


